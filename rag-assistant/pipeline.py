"""
RAG 文档问答助手 —— 核心 Pipeline
PDF解析 → 文本分片 → 向量化 → 入库 → 语义检索 → 组 Prompt → 调 LLM

设计要点（对应技术方案第 3 节参数）：
- chunk_size=500 / chunk_overlap=50
- Top-K=5
- 相似度低于阈值视为无匹配：OpenAI embedding 默认 0.5；本地 TF-IDF 默认 0.10
- 无匹配直接返回「文档中未找到相关内容」，绝不调模型编造
- 单文件模式：每次上传会先清空向量库再写入

Embedding 双后端：
- openai：text-embedding-3-small（有 Key 时，语义泛化更好）
- local ：jieba 分词 + 语料 TF-IDF（离线可用，中文检索效果好，无需任何 Key）
"""

from __future__ import annotations

import json
import math
import os
import re
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:  # PyMuPDF 2.x 推荐 pymupdf，1.x 只有 fitz
    import pymupdf as fitz
except ImportError:  # pragma: no cover
    import fitz  # type: ignore

try:
    import jieba  # 纯 Python，离线分词
except ImportError:  # pragma: no cover
    jieba = None

# ---------------------------------------------------------------------------
# 参数
# ---------------------------------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent

# 支持同目录 .env（pip 里带 python-dotenv；缺席也不影响）
try:  # pragma: no cover
    from dotenv import load_dotenv

    load_dotenv(BASE_DIR / ".env")
except Exception:  # noqa: BLE001
    pass

CHUNK_SIZE = int(os.getenv("RAG_CHUNK_SIZE", "500"))
CHUNK_OVERLAP = int(os.getenv("RAG_CHUNK_OVERLAP", "50"))
TOP_K = int(os.getenv("RAG_TOP_K", "5"))
NO_MATCH_ANSWER = "文档中未找到相关内容"

STORE_DIR = Path(os.getenv("RAG_STORE_DIR", str(BASE_DIR / "chroma_db")))

# 本地 TF-IDF 后端常见的疑问/虚词，避免它们干扰相似度
STOPWORDS = {
    "的", "了", "是", "在", "和", "与", "及", "或", "有", "为", "对", "把", "被",
    "这", "那", "这个", "那个", "这些", "那些", "它", "他", "她", "我", "你",
    "什么", "怎么", "怎样", "如何", "多少", "哪些", "哪个", "为什么", "请问",
    "吗", "呢", "吧", "啊", "呀", "嘛", "哦", "嗯", "一下", "一个", "可以",
    "么", "们", "就", "都", "也", "很", "还", "会", "要", "能", "用", "做",
}


def _norm_key(name: str) -> str | None:
    v = os.getenv(name, "").strip()
    return v or None


def llm_config() -> dict[str, str]:
    """LLM 配置（OpenAI 兼容接口，DeepSeek 等直接填 base_url 即可）"""
    api_key = _norm_key("RAG_LLM_API_KEY") or _norm_key("OPENAI_API_KEY") or ""
    base_url = (
        _norm_key("RAG_LLM_BASE_URL")
        or _norm_key("OPENAI_BASE_URL")
        or "https://api.openai.com/v1"
    )
    model = _norm_key("RAG_LLM_MODEL") or "gpt-4o-mini"
    return {"api_key": api_key, "base_url": base_url, "model": model}


def embed_backend() -> str:
    """openai / local：显式指定优先；否则有 Key 用 openai，无 Key 用本地"""
    forced = (_norm_key("RAG_EMBED_BACKEND") or "").lower()
    if forced in {"openai", "local"}:
        return forced
    return "openai" if (_norm_key("RAG_LLM_API_KEY") or _norm_key("OPENAI_API_KEY")) else "local"


def sim_threshold(backend: str) -> float:
    """相似度阈值：两种向量的分数分布完全不同，默认值必须分开"""
    env = _norm_key("RAG_SIM_THRESHOLD")
    if env:
        try:
            return float(env)
        except ValueError:
            pass
    return 0.5 if backend == "openai" else 0.10


SIM_THRESHOLD = sim_threshold(embed_backend())


# ---------------------------------------------------------------------------
# 1. PDF 解析：逐页提取，页码从 1 开始
# ---------------------------------------------------------------------------
def extract_pdf_pages(path: str | Path) -> list[tuple[int, str]]:
    pages: list[tuple[int, str]] = []
    with fitz.open(str(path)) as doc:
        for idx, page in enumerate(doc, start=1):
            text = page.get_text("text") or ""
            text = text.replace("\u3000", " ").strip()
            if text:
                pages.append((idx, text))
    return pages


# ---------------------------------------------------------------------------
# 2. 文本分片：句级切分后贪心合并，带字符级重叠
# ---------------------------------------------------------------------------
_SENT_SPLIT = re.compile(r"(?<=[。！？；!?;\n])")


def _sentence_units(text: str) -> list[str]:
    return [p for p in _SENT_SPLIT.split(text) if p and p.strip()] or [text]


def split_text(text: str, size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> list[str]:
    overlap = max(0, min(overlap, size // 2))
    chunks: list[str] = []
    buf = ""
    for unit in _sentence_units(text):
        unit = unit.strip()
        if not unit:
            continue
        if len(buf) + len(unit) + 1 <= size:
            buf = f"{buf}\n{unit}" if buf else unit
            continue
        if buf.strip():
            chunks.append(buf.strip())
        buf = (buf[-overlap:] if overlap and buf else "") + unit
        while len(buf) > size:  # 单句超长 → 硬切
            chunks.append(buf[:size].strip())
            buf = buf[size - overlap:] if overlap else buf[size:]
    if buf.strip():
        chunks.append(buf.strip())
    return [c for c in chunks if c]


def build_chunks(pages: list[tuple[int, str]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    seq = 0
    for page_no, text in pages:
        for piece in split_text(text):
            out.append({"text": piece, "page": page_no, "seq": seq})
            seq += 1
    return out


# ---------------------------------------------------------------------------
# 3. Embedding
# ---------------------------------------------------------------------------
_PUNCT = re.compile(r"[\s\u3000!-/:-@\[-`{-~，。！？；：、“”‘’（）《》【】…—]+")


def tokenize(text: str) -> list[str]:
    """jieba 分词（无 jieba 时退化为字符 bigram）"""
    t = _PUNCT.sub(" ", (text or "").lower()).strip()
    if not t:
        return []
    if jieba is not None:
        words = [w.strip() for w in jieba.cut(t)]
    else:  # pragma: no cover
        words = [t[i : i + 2] for i in range(max(0, len(t) - 1))] or [t]
    return [w for w in words if w and w not in STOPWORDS and not _PUNCT.fullmatch(w)]


class LocalTfidfEmbedder:
    """离线向量化：jieba 分词 + 语料 TF-IDF + L2 归一化（需先用语料 fit）"""

    def __init__(self) -> None:
        self.name = "local-tfidf-jieba" if jieba is not None else "local-tfidf-char2"
        self.vocab: dict[str, int] = {}
        self.idf: np.ndarray = np.zeros(0, dtype=np.float32)
        self._fitted = False

    def fit(self, texts: list[str]) -> None:
        tokenized = [tokenize(t) for t in texts]
        df: Counter[str] = Counter()
        for toks in tokenized:
            df.update(set(toks))
        n = max(1, len(tokenized))
        vocab = {w: i for i, w in enumerate(sorted(df))}
        idf = np.zeros(len(vocab), dtype=np.float32)
        for w, i in vocab.items():
            idf[i] = math.log((n + 1) / (df[w] + 1)) + 1.0
        self.vocab, self.idf, self._fitted = vocab, idf, True

    def encode(self, texts: list[str]) -> np.ndarray:
        if not self._fitted:
            self.fit(texts or [" "])
        out = np.zeros((len(texts), len(self.vocab)), dtype=np.float32)
        for row, text in enumerate(texts):
            tf: Counter[str] = Counter()
            for w in tokenize(text):
                i = self.vocab.get(w)
                if i is not None:
                    tf[i] += 1
            for i, c in tf.items():
                out[row, i] = (1.0 + math.log(c)) * self.idf[i]
            norm = float(np.linalg.norm(out[row]))
            if norm:
                out[row] /= norm
        return out


class OpenAIEmbedder:
    def __init__(self, model: str, base_url: str, api_key: str):
        from openai import OpenAI  # 延迟导入

        self.client = OpenAI(api_key=api_key, base_url=base_url or None)
        self.model = model
        self.name = f"openai:{model}"

    def fit(self, texts: list[str]) -> None:  # 无需训练，保持接口一致
        return None

    def encode(self, texts: list[str]) -> np.ndarray:
        if not texts:
            return np.zeros((0, 1), dtype=np.float32)
        vectors: list[list[float]] = []
        for i in range(0, len(texts), 64):
            resp = self.client.embeddings.create(model=self.model, input=texts[i : i + 64])
            vectors.extend([d.embedding for d in resp.data])
        arr = np.asarray(vectors, dtype=np.float32)
        norms = np.linalg.norm(arr, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        return arr / norms


def make_embedder():
    if embed_backend() == "openai":
        cfg = llm_config()
        model = _norm_key("RAG_EMBED_MODEL") or "text-embedding-3-small"
        return OpenAIEmbedder(model=model, base_url=cfg["base_url"], api_key=cfg["api_key"])
    return LocalTfidfEmbedder()


# ---------------------------------------------------------------------------
# 4. 向量库：Chroma（首选）/ NumpyStore（无 chromadb 时兜底）
# ---------------------------------------------------------------------------
class NumpyStore:
    """极简本地向量库：chunks.jsonl + vectors.npy"""

    name = "numpy-file"

    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=True)
        self.chunks_file = self.path / "chunks.jsonl"
        self.vectors_file = self.path / "vectors.npy"
        self._chunks: list[dict[str, Any]] = []
        self._vectors: np.ndarray | None = None
        self._load()

    def _load(self) -> None:
        if self.chunks_file.exists() and self.vectors_file.exists():
            self._chunks = [
                json.loads(line)
                for line in self.chunks_file.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            self._vectors = np.load(self.vectors_file)

    def reset(self) -> None:
        self._chunks = []
        self._vectors = None
        for f in (self.chunks_file, self.vectors_file):
            if f.exists():
                f.unlink()

    def add(self, chunks: list[dict[str, Any]], vectors: np.ndarray) -> None:
        if not chunks:
            return
        start = len(self._chunks)
        rows = []
        with self.chunks_file.open("a", encoding="utf-8") as fh:
            for i, ch in enumerate(chunks):
                rec = {**ch, "id": f"c{start + i}"}
                rows.append(rec)
                fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._chunks.extend(rows)
        prev = self._vectors if self._vectors is not None else np.zeros((0, vectors.shape[1]), np.float32)
        self._vectors = np.vstack([prev, vectors])
        np.save(self.vectors_file, self._vectors)

    def search(self, qvec: np.ndarray, top_k: int) -> list[tuple[float, dict[str, Any]]]:
        if self._vectors is None or not self._chunks:
            return []
        sims = self._vectors @ qvec.reshape(-1)
        order = np.argsort(-sims)[:top_k]
        return [(float(sims[i]), self._chunks[i]) for i in order]

    def all_texts(self) -> list[str]:
        return [c["text"] for c in self._chunks]

    def count(self) -> int:
        return len(self._chunks)


class ChromaStore:
    name = "chroma"

    def __init__(self, path: Path):
        import chromadb  # 延迟导入

        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=True)
        self.client = chromadb.PersistentClient(path=str(self.path))
        self.collection = self.client.get_or_create_collection(
            name="doc_chunks", metadata={"hnsw:space": "cosine"}
        )

    def reset(self) -> None:
        try:
            self.client.delete_collection("doc_chunks")
        except Exception:
            pass
        self.collection = self.client.get_or_create_collection(
            name="doc_chunks", metadata={"hnsw:space": "cosine"}
        )

    def add(self, chunks: list[dict[str, Any]], vectors: np.ndarray) -> None:
        if not chunks:
            return
        base = self.collection.count()
        self.collection.add(
            ids=[f"c{base + i}" for i in range(len(chunks))],
            embeddings=[v.tolist() for v in vectors],
            documents=[c["text"] for c in chunks],
            metadatas=[{"page": c["page"], "seq": c["seq"]} for c in chunks],
        )

    def search(self, qvec: np.ndarray, top_k: int) -> list[tuple[float, dict[str, Any]]]:
        if self.collection.count() == 0:
            return []
        res = self.collection.query(
            query_embeddings=[qvec.tolist()],
            n_results=min(top_k, self.collection.count()),
            include=["documents", "metadatas", "distances"],
        )
        docs = (res.get("documents") or [[]])[0]
        metas = (res.get("metadatas") or [[]])[0]
        dists = (res.get("distances") or [[]])[0]
        hits: list[tuple[float, dict[str, Any]]] = []
        for doc, meta, dist in zip(docs, metas, dists):
            hits.append(
                (
                    float(1.0 - dist),  # cosine space: distance = 1 - similarity
                    {"text": doc, "page": int(meta.get("page", 1)), "seq": int(meta.get("seq", 0))},
                )
            )
        return hits

    def all_texts(self) -> list[str]:
        if self.collection.count() == 0:
            return []
        res = self.collection.get(include=["documents"])
        return list(res.get("documents") or [])

    def count(self) -> int:
        return self.collection.count()


def make_store() -> Any:
    forced = (_norm_key("RAG_STORE") or "").lower()
    if forced == "numpy":
        return NumpyStore(STORE_DIR)
    try:
        return ChromaStore(STORE_DIR)
    except Exception:
        return NumpyStore(STORE_DIR)


# ---------------------------------------------------------------------------
# 5. LLM 生成
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = (
    "你是一个严谨的文档问答助手。你只能依据【文档片段】中的内容回答，"
    "禁止使用片段之外的知识，禁止编造。"
    f"如果片段中没有足以回答问题的信息，你必须只输出这一句：{NO_MATCH_ANSWER}"
)


def build_prompt(question: str, hits: list[tuple[float, dict[str, Any]]]) -> str:
    blocks = [
        f"[片段{i}] (第{ch['page']}页, 相似度{score:.3f})\n{ch['text']}"
        for i, (score, ch) in enumerate(hits, start=1)
    ]
    return (
        f"【文档片段】\n{chr(10).join(blocks)}\n\n"
        f"【问题】{question}\n\n"
        "请用中文简洁作答；如果回答依据了某个片段，在句末用 (第X页) 标注页码。"
        f"如果片段里没有答案，只输出「{NO_MATCH_ANSWER}」。"
    )


def call_llm(question: str, hits: list[tuple[float, dict[str, Any]]]) -> str:
    cfg = llm_config()
    if not cfg["api_key"]:
        # 没有 Key：退化为抽取式回答，保证全链路可跑通且不编造
        lines = ["（未配置大模型 API Key，以下为检索到的原文片段）", ""]
        for i, (_, ch) in enumerate(hits, start=1):
            snippet = ch["text"][:220].replace("\n", " ")
            lines.append(f"{i}. (第{ch['page']}页) {snippet}…")
        return "\n".join(lines)

    from openai import OpenAI

    client = OpenAI(api_key=cfg["api_key"], base_url=cfg["base_url"])
    resp = client.chat.completions.create(
        model=cfg["model"],
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": build_prompt(question, hits)},
        ],
        temperature=0.1,
    )
    return (resp.choices[0].message.content or "").strip()


# ---------------------------------------------------------------------------
# 6. Pipeline 门面
# ---------------------------------------------------------------------------
@dataclass
class RagPipeline:
    embedder: Any = None
    store: Any = None
    doc_name: str = ""

    def __post_init__(self) -> None:
        if self.embedder is None:
            self.embedder = make_embedder()
        if self.store is None:
            self.store = make_store()
        # 重启后：用已入库的分片重新 fit 本地词表，保证查询向量空间一致
        texts = self.store.all_texts()
        if texts:
            self.embedder.fit(texts)

    # -- 入库 --------------------------------------------------------------
    def ingest_pdf(self, path: str | Path, filename: str = "") -> dict[str, Any]:
        pages = extract_pdf_pages(path)
        if not pages:
            raise ValueError("PDF 中未提取到任何文字（可能是扫描件或图片型 PDF）")
        chunks = build_chunks(pages)
        if not chunks:
            raise ValueError("文本分片失败，请检查文档内容")

        self.store.reset()  # 单文件模式
        texts = [c["text"] for c in chunks]
        self.embedder.fit(texts)
        vectors = self.embedder.encode(texts)
        self.store.add(chunks, vectors)
        self.doc_name = filename or Path(path).name
        return {
            "chunks": len(chunks),
            "pages": len(pages),
            "chars": sum(len(c["text"]) for c in chunks),
            "doc_name": self.doc_name,
        }

    def reset(self) -> None:
        self.store.reset()
        self.doc_name = ""

    # -- 检索 + 生成 --------------------------------------------------------
    def query(
        self,
        question: str,
        top_k: int = TOP_K,
        threshold: float | None = None,
    ) -> dict[str, Any]:
        question = (question or "").strip()
        if not question:
            raise ValueError("问题不能为空")
        if self.store.count() == 0:
            return {"answer": NO_MATCH_ANSWER, "sources": [], "reason": "empty_store"}

        th = SIM_THRESHOLD if threshold is None else threshold
        qvec = self.embedder.encode([question])[0]
        if not np.any(qvec):
            # 问题里的词在文档语料里一个都没有 → 直接判定无关
            return {"answer": NO_MATCH_ANSWER, "sources": [], "reason": "no_overlap", "best_score": 0.0}

        hits = self.store.search(qvec, top_k)
        kept = [(s, ch) for s, ch in hits if s >= th]
        if not kept:
            return {
                "answer": NO_MATCH_ANSWER,
                "sources": [],
                "reason": f"no_hit_above_threshold({th})",
                "best_score": round(hits[0][0], 4) if hits else None,
            }

        answer = call_llm(question, kept)
        sources = [
            {"page": ch["page"], "text": ch["text"], "score": round(score, 4)}
            for score, ch in kept
        ]
        return {"answer": answer, "sources": sources, "reason": "ok"}

    def status(self) -> dict[str, Any]:
        cfg = llm_config()
        return {
            "doc_name": self.doc_name,
            "chunks": self.store.count(),
            "embedder": getattr(self.embedder, "name", "unknown"),
            "store": getattr(self.store, "name", "unknown"),
            "llm_model": cfg["model"] if cfg["api_key"] else None,
            "llm_configured": bool(cfg["api_key"]),
            "params": {
                "chunk_size": CHUNK_SIZE,
                "chunk_overlap": CHUNK_OVERLAP,
                "top_k": TOP_K,
                "threshold": SIM_THRESHOLD,
            },
        }


_pipeline: RagPipeline | None = None


def get_pipeline() -> RagPipeline:
    global _pipeline
    if _pipeline is None:
        _pipeline = RagPipeline()
    return _pipeline
