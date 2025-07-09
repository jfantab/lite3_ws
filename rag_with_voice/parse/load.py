### IMPORTS ###

import os
from dotenv import load_dotenv
import uuid

from langchain_openai import OpenAIEmbeddings
from langchain_community.document_loaders import PyMuPDFLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter

import chromadb
from chromadb.utils import embedding_functions

### API KEYS ###

load_dotenv()

os.environ['LANGCHAIN_TRACING_V2'] = 'true'
os.environ['LANGCHAIN_ENDPOINT'] = 'https://api.smith.langchain.com'
os.environ['LANGCHAIN_API_KEY'] = os.getenv('LANGCHAIN_API_KEY')

os.environ["OPENAI_API_KEY"] = os.getenv("OPENAI_API_KEY")

### PATHS ###

HOME = "/home/jfantab/lite3_ws/rag_with_voice"
PREFIX = "pdfs"

PATH = os.path.join(HOME, PREFIX)
pdfs = os.listdir(PATH)

### EMBEDDINGS ###

model_name = "all-distilroberta-v1"
embedding_fn = embedding_functions.SentenceTransformerEmbeddingFunction(model_name=model_name)

### PERSISTENT RAG DB ###

class CustomOpenAIEmbeddings():
    def __init__(self):
        self.embedding_model = OpenAIEmbeddings()
    
    def name(self):  # Must be callable method
        return "openai"
    
    def __call__(self, input):
        return self.embedding_model.embed_documents(input)
    
    def embed_query(self, text):
        return self.embedding_model.embed_query(text)

chroma_client = chromadb.PersistentClient(path=os.path.join(HOME, "chroma_db"))
collection = chroma_client.create_collection(name="new_collection", embedding_function=CustomOpenAIEmbeddings())

### PDF parsing ###

def load_pdf(path):
    loader = PyMuPDFLoader(path)
    txt_splitter = RecursiveCharacterTextSplitter(chunk_size=1000, chunk_overlap=200)
    docs = loader.load_and_split(txt_splitter)
    return docs

for pdf in pdfs:
    docs = load_pdf(os.path.join(PATH, pdf))
    for split in docs:
        id = str(uuid.uuid4())
        collection.add(documents=[split.page_content], ids=[id])
