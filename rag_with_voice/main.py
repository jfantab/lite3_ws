from openai_tests.whisper_test import *
from parse.pymu import *
from openai_tests.tts import *

def main():
    record_audio()
    
    transcription = whisper_transcribe()

    result = rag_chain.invoke(transcription)
    print(result)

    dictate(result)

if __name__ == "__main__":
    main()
