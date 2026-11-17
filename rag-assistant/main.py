"""
RAG 文档问答助手 —— FastAPI 入口

接口：
  POST /api/upload  multipart/form-data, field=file   → {status, chunks, pages, chars, doc_name}
  POST /api/query   {"question": "..."}               → {answer, sources:[{page,text,score}]}
  GET  /api/status                                     → 当前文档/向量库/模型状态
  GET  /                                               → 前端页面
"""

from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from pipeline import NO_MATCH_ANSWER, get_pipeline

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
MAX_MB = 25

app = FastAPI(title="RAG 文档问答助手", version="0.1.0")

if STATIC_DIR.is_dir():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


class QueryIn(BaseModel):
    question: str


@app.get("/")
def index():
    page = STATIC_DIR / "index.html"
    if not page.exists():
        raise HTTPException(500, "缺少 static/index.html")
    return FileResponse(page)


@app.get("/api/status")
def status():
    return get_pipeline().status()


@app.post("/api/upload")
async def upload(file: UploadFile = File(...)):
    name = (file.filename or "").strip()
    if not name.lower().endswith(".pdf"):
        raise HTTPException(400, "只支持 PDF 文件")

    started = time.time()
    fd, tmp_name = tempfile.mkstemp(suffix=".pdf")
    os.close(fd)  # Windows 下句柄不关会导致后续 unlink 失败
    tmp_path = Path(tmp_name)
    size = 0
    try:
        with tmp_path.open("wb") as out:
            while chunk := await file.read(1024 * 1024):
                size += len(chunk)
                if size > MAX_MB * 1024 * 1024:
                    raise HTTPException(413, f"文件超过 {MAX_MB}MB 上限")
                out.write(chunk)
        if size == 0:
            raise HTTPException(400, "空文件")

        pipeline = get_pipeline()
        try:
            info = pipeline.ingest_pdf(tmp_path, filename=name)
        except ValueError as exc:
            raise HTTPException(400, str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(500, f"解析失败：{exc}") from exc

        info.update(
            {
                "status": "success",
                "size_kb": round(size / 1024, 1),
                "elapsed_ms": int((time.time() - started) * 1000),
            }
        )
        return info
    finally:
        try:
            tmp_path.unlink(missing_ok=True)
        except OSError:
            pass
        await file.close()


@app.post("/api/query")
def query(payload: QueryIn):
    question = (payload.question or "").strip()
    if not question:
        raise HTTPException(400, "问题不能为空")
    try:
        result = get_pipeline().query(question)
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(502, f"问答失败：{exc}") from exc
    if result["answer"] == NO_MATCH_ANSWER and not result["sources"]:
        result.setdefault("hint", "检索没有达到相似度阈值的片段，已按约定不调用模型")
    return result


@app.post("/api/reset")
def reset():
    get_pipeline().reset()
    return {"status": "success"}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=False)
