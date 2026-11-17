# RAG文档问答助手 技术方案

## 1. 整体架构
```
浏览器(Web UI) → 后端API(FastAPI) → 核心Pipeline
                                    ├─ PDF解析层
                                    ├─ 分片+向量化层
                                    ├─ 向量检索层
                                    └─ LLM问答层
```
单进程本地部署，无数据库服务依赖，向量库以本地文件形式存储。

## 2. 技术栈选型

| 层级 | 选型 | 说明 |
|------|------|------|
| 后端框架 | Python + FastAPI | 轻量异步，自带API文档，MVP最快落地 |
| 前端 | 原生HTML + JS（单页） | 不上框架，一个文件搞定上传+问答+展示 |
| PDF解析 | PyMuPDF (fitz) | 逐页提取文字，天然保留页码信息，速度快 |
| 文本分片 | LangChain RecursiveCharacterTextSplitter | chunk_size=500，chunk_overlap=50 |
| Embedding模型 | OpenAI text-embedding-3-small（或本地bge-small-zh） | 需API Key；无外网则换本地模型 |
| 向量库 | Chroma（本地持久化） | 零部署，Python直调，自带相似度检索 |
| LLM | OpenAI GPT-4o-mini（或任意OpenAI兼容接口） | 需API Key；Prompt中强制"仅基于提供的上下文回答" |
| HTTP客户端 | requests / openai SDK | 调Embedding和Chat接口 |

## 3. 核心Pipeline参数

| 环节 | 参数 | 值 |
|------|------|-----|
| 分片长度 | chunk_size | 500字 |
| 分片重叠 | chunk_overlap | 50字 |
| 检索Top-K | 召回片段数 | 5 |
| 相似度阈值 | 低于此值视为无匹配 | 0.5（余弦距离） |
| Prompt策略 | 无匹配兜底 | 检索结果为空或相似度全部低于阈值 → 直接返回"文档中未找到相关内容" |

## 4. API接口设计

### POST /api/upload
- 入参：multipart/form-data，字段file（PDF文件）
- 处理：解析PDF → 分片 → Embedding → 写入Chroma
- 出参：`{ "status": "success", "chunks": 120 }` 或错误信息

### POST /api/query
- 入参：`{ "question": "xxx" }`
- 处理：问题向量化 → Chroma检索Top-5 → 组装Prompt → 调LLM
- 出参：
```json
{
  "answer": "回答内容",
  "sources": [
    { "page": 12, "text": "引用片段原文..." }
  ]
}
```
- 无匹配时：`{ "answer": "文档中未找到相关内容", "sources": [] }`

## 5. 前端页面结构
- 顶部：文件上传按钮 + 上传状态提示
- 中部：问题输入框 + 提问按钮
- 底部：回答展示区 + 引用来源折叠列表（显示页码+原文片段）

## 6. 环境依赖与工具要求

### 必须安装
- Python 3.10+
- pip包：fastapi, uvicorn, pymupdf, langchain, chromadb, openai

### 需要用户提供
- **LLM API Key**（OpenAI或兼容服务的base_url + key）
- **Embedding API Key**（同LLM平台即可）
- 如无外网：需额外下载本地Embedding模型（bge-small-zh-v1.5），约100MB

### 目录结构
```
rag-assistant/
├── main.py            # FastAPI入口
├── pipeline.py        # PDF解析+分片+入库+检索逻辑
├── static/
│   └── index.html     # 前端页面
├── chroma_db/         # 向量库持久化目录（自动生成）
└── requirements.txt
```

## 7. 启动方式
```bash
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
# 浏览器打开 http://localhost:8000
```
