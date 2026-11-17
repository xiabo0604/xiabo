# RAG 文档问答助手（MVP）

上传一份可复制文字的 PDF → 自然语言提问 → 基于文档内容回答，并给出**页码 + 原文引用**。
文档里没有的问题，直接回答「文档中未找到相关内容」，不编造。

对应方案：`../RAG文档问答助手MVP产品方案.md`、`../RAG文档问答助手技术方案.md`

---

## 1. 功能范围（P0）

| 功能 | 状态 |
|---|---|
| 单文件 PDF 上传（仅可提取文字，不做扫描件 OCR） | ✅ |
| 逐页文本解析并保留页码映射 | ✅ |
| 文本分片（500 字 / 重叠 50）+ 向量化 + 本地向量库 | ✅ |
| 问题向量化 → Top-5 语义检索 | ✅ |
| 组装 Prompt 调 LLM，仅依据检索片段回答 | ✅ |
| 回答附页码 + 原文引用；无匹配回复固定文案 | ✅ |
| 单页 Web（上传 / 提问 / 回答 + 引用折叠） | ✅ |

P0 之外（多文档、登录、多轮记忆、Word/Excel、Rerank、导出分享）未实现，与方案一致。

---

## 2. 目录结构

```
rag-assistant/
├── main.py              # FastAPI 入口（/api/upload、/api/query、/api/status、/api/reset）
├── pipeline.py          # PDF解析 → 分片 → 向量化 → 入库 → 检索 → 生成
├── static/index.html    # 单页前端
├── samples/             # 示例 PDF
├── chroma_db/           # 向量库持久化目录（自动生成）
├── requirements.txt
├── .env.example
└── start.bat            # Windows 一键启动
```

---

## 3. 安装与启动

```bash
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt      # Windows
# source .venv/bin/activate && pip install -r requirements.txt   # macOS / Linux

copy .env.example .env                              # 按需填写 Key
.venv\Scripts\python -m uvicorn main:app --host 127.0.0.1 --port 8000
```

浏览器打开 <http://127.0.0.1:8000>，或双击 `start.bat`。

### 三种可选配置

| 场景 | 配置 | 说明 |
|---|---|---|
| 只要跑通（无任何 Key） | 什么都不填 | 本地 TF-IDF 检索 + 抽取式回答（返回检索到的原文片段），**完全离线** |
| 本地检索 + DeepSeek 作答（推荐） | `RAG_LLM_API_KEY=sk-xxx`<br>`RAG_LLM_BASE_URL=https://api.deepseek.com/v1`<br>`RAG_LLM_MODEL=deepseek-chat`<br>`RAG_EMBED_BACKEND=local` | DeepSeek 没有 embedding 接口，检索留在本地 |
| 全 OpenAI | `OPENAI_API_KEY=sk-xxx`<br>`RAG_EMBED_BACKEND=openai` | 语义泛化更好，需联网 |

> 相似度阈值会随后端自动切换：OpenAI embedding 默认 `0.5`，本地 TF-IDF 默认 `0.10`（两者分数分布完全不同）。可用 `RAG_SIM_THRESHOLD` 覆盖。

---

## 4. API

### POST /api/upload
`multipart/form-data`，字段 `file`（PDF，≤25MB）

```json
{"chunks": 3, "pages": 3, "chars": 373, "doc_name": "sample.pdf",
 "status": "success", "size_kb": 4.1, "elapsed_ms": 909}
```
单文件模式：每次上传会先清空向量库。

### POST /api/query
```json
{"question": "熄屏待机电流是多少？"}
```
```json
{"answer": "熄屏待机电流为 38 微安 (第2页)。",
 "sources": [{"page": 2, "text": "……实测待机电流为 38 微安。", "score": 0.2907}],
 "reason": "ok"}
```
无匹配：
```json
{"answer": "文档中未找到相关内容", "sources": [], "reason": "no_overlap"}
```

### 其他
- `GET /api/status`：当前文档、分片数、向量化后端、向量库类型、模型、参数
- `POST /api/reset`：清空当前文档

---

## 5. 实测结果

| 用例 | 结果 |
|---|---|
| 3 页 / 20 页 PDF 上传入库 | ✅ 3 分片 / 40 分片，页码映射正确 |
| 「熄屏待机电流是多少？」 | ✅ `38 微安 (第2页)`，sources 命中 p2 |
| 「蓝牙模组型号是什么」 | ✅ `CH9141 (第3页)` |
| 「第13章的采样周期是多少？」（20 页文档） | ✅ `91 毫秒 (第13页)` |
| 「MOD017 的校验方式是什么」 | ✅ `CRC3 (第17页)` |
| 「今天天气怎么样」「红烧肉怎么做」 | ✅ `文档中未找到相关内容`，sources 为空 |
| 「电池的循环寿命是多少？」（词命中但答案不存在） | ✅ `文档中未找到相关内容`，未编造 |

---

## 6. 设计说明

**为什么本地检索用 jieba + TF-IDF 而不是字符 n-gram**
实测字符 2/3-gram 向量在「离题问题」上仍有 0.09 的相似度，与真正命中的 0.15 太接近，无法稳定判别；换成 jieba 分词 + 语料 IDF 后，离题问题相似度直接为 0.000，阈值判别干净。且完全离线、零下载。

**为什么阈值分后端**
OpenAI embedding 的余弦相似度基座高（相关问题常 0.6+），本地 TF-IDF 命中多在 0.15~0.42。同一个 0.5 挂在本地后端上会让所有问题都判成无匹配，所以按后端给默认值。

**双重防编造**
1. 检索层：低于阈值或无词重叠（`no_overlap`）直接返回固定文案，不进模型；
2. 生成层：System Prompt 强制「只依据片段回答，否则只输出该文案」。

**兼容性兜底**
- 向量库：优先 chromadb，导入失败自动退回内置 `numpy-file`（`chunks.jsonl + vectors.npy`）
- 分词：jieba 缺失时退化为字符 bigram TF-IDF
- 大模型：无 Key 时退化为抽取式回答，全链路仍可跑通

---

## 7. 已知限制

- 不支持扫描件 / 图片型 PDF（无 OCR，按方案 P0 Out）
- 单文档，无文档管理与多轮对话记忆
- 检索为纯向量（本地 TF-IDF 或 OpenAI embedding），无 Rerank / 混合检索
- 长文档 Top-5 召回可能漏掉跨段落信息，属 P1「混合检索 + Rerank」范畴
