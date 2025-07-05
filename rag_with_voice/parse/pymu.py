### IMPORTS ###

import os
from dotenv import load_dotenv
import uuid

from langchain import hub
from langchain_core.output_parsers import StrOutputParser
from langchain_core.prompts import ChatPromptTemplate, SystemMessagePromptTemplate, HumanMessagePromptTemplate
from langchain_core.runnables import RunnablePassthrough

from langchain_community.vectorstores import Chroma
from langchain_openai import ChatOpenAI, OpenAIEmbeddings
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

HOME = "/Users/johnlu/Documents/rag_with_voice"
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

collection = chroma_client.get_collection(name="new_collection", embedding_function=CustomOpenAIEmbeddings())

llm = ChatOpenAI(model="o4-mini")
vectorstore = Chroma(
    collection_name="new_collection",
    persist_directory="chroma_db",
    embedding_function=CustomOpenAIEmbeddings()
)
retriever = vectorstore.as_retriever(search_kwargs={"k": 2})
prompt = hub.pull("rlm/rag-prompt")

def format_docs(docs):
    return "\n\n".join(doc.page_content for doc in docs)

rag_chain = (
    {"context": retriever | format_docs, "question": RunnablePassthrough()}
    | prompt
    | llm
    | StrOutputParser()
)