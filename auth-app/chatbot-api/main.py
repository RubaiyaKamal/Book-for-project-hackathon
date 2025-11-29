from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
from openai import OpenAI
from qdrant_client import QdrantClient

# Environment variables
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "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")
QDRANT_URL = os.getenv("QDRANT_URL", "https://95f917bd-5eae-4a33-bb5b-01706d914e55.europe-west3-0.gcp.cloud.qdrant.io")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.vGM2WFJKbHspSDW2Lw_zGMKEAE2aV_8JMOZQU6Y_Blo")

# Initialize clients lazily to prevent startup errors
openai_client = None
qdrant_client = None

def get_openai_client():
    global openai_client
    if openai_client is None:
        openai_client = OpenAI(api_key=OPENAI_API_KEY)
    return openai_client

def get_qdrant_client():
    global qdrant_client
    if qdrant_client is None:
        qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    return qdrant_client

COLLECTION_NAME = "book_chunks"
EMBEDDING_MODEL = "text-embedding-ada-002"
CHAT_MODEL = "gpt-4o-mini"

from routers import auth
import models
from database import engine

models.Base.metadata.create_all(bind=engine)

app = FastAPI(title="RAG Chatbot API")

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Next.js frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(auth.router)

# Request models
class ChatRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    user_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[str]

# Helper functions
def get_embedding(text: str) -> List[float]:
    """Generate embedding for a text using OpenAI"""
    client = get_openai_client()
    response = client.embeddings.create(
        input=text,
        model=EMBEDDING_MODEL
    )
    return response.data[0].embedding

def search_similar_chunks(query: str, top_k: int = 5) -> List[dict]:
    """Search for similar chunks in Qdrant"""
    query_embedding = get_embedding(query)

    client = get_qdrant_client()
    search_result = client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=top_k
    )

    return [
        {
            "content": hit.payload.get("content", ""),
            "source": hit.payload.get("source", "Unknown"),
            "score": hit.score
        }
        for hit in search_result
    ]

def generate_answer(question: str, context: str, selected_text: Optional[str] = None) -> str:
    """Generate answer using OpenAI chat completion"""

    if selected_text:
        system_prompt = f"""You are a helpful assistant answering questions about a book on Physical AI & Humanoid Robotics.
The user has selected the following text from the book:
---
{selected_text}
---

Answer the question based ONLY on this selected text. If the answer cannot be found in the selected text, say so."""
    else:
        system_prompt = """You are a helpful assistant answering questions about a book on Physical AI & Humanoid Robotics.
Use the provided context from the book to answer questions accurately and concisely."""

    user_prompt = f"""Context from the book:
{context}

Question: {question}

Please provide a clear, concise answer based on the context above."""

    response = get_openai_client().chat.completions.create(
        model=CHAT_MODEL,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ],
        temperature=0.7,
        max_tokens=500
    )

    return response.choices[0].message.content

# API Endpoints
@app.get("/")
async def read_root():
    return {"message": "RAG Chatbot API is running", "status": "healthy"}

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Main RAG endpoint for answering questions
    - If selected_text is provided, answer based only on that text
    - Otherwise, search the vector database for relevant context
    """
    try:
        if request.selected_text:
            # Answer based on selected text only
            answer = generate_answer(
                question=request.question,
                context=request.selected_text,
                selected_text=request.selected_text
            )
            sources = ["Selected Text"]
        else:
            # Search vector database for relevant chunks
            similar_chunks = search_similar_chunks(request.question, top_k=5)

            if not similar_chunks:
                raise HTTPException(status_code=404, detail="No relevant content found")

            # Combine chunks for context
            context = "\n\n".join([chunk["content"] for chunk in similar_chunks])
            sources = list(set([chunk["source"] for chunk in similar_chunks]))

            # Generate answer
            answer = generate_answer(
                question=request.question,
                context=context
            )

        return ChatResponse(answer=answer, sources=sources)

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

@app.get("/health")
async def health_check():
    """Check if all services are connected"""
    try:
        # Check Qdrant connection
        collections = qdrant_client.get_collections()
        qdrant_status = "connected"
    except Exception as e:
        qdrant_status = f"error: {str(e)}"

    return {
        "status": "healthy",
        "qdrant": qdrant_status,
        "openai": "configured"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
