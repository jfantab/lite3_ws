import os
from openai import OpenAI
from dotenv import load_dotenv

import pyaudio
import io
import numpy as np
from pydub import AudioSegment

load_dotenv()

def dictate(text):
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    VOICE = "ash"
    FORMAT = pyaudio.paInt16
    RATE = 24000
    CHANNELS = 1

    audio = pyaudio.PyAudio()
    stream = audio.open(format=FORMAT, rate=RATE, channels=CHANNELS, output=True)

    with client.audio.speech.with_streaming_response.create(
            model="gpt-4o-mini-tts",
            voice=VOICE,
            input=text,
            response_format="mp3",
            instructions="Speak in a cheerful and positive tone.",
        ) as response:
            mp3_buffer = io.BytesIO()
            
            # Stream chunks from OpenAI
            for chunk in response.iter_bytes(chunk_size=4096):
                if chunk:
                    mp3_buffer.write(chunk)
                    
                    # Process when we have enough data
                    if mp3_buffer.tell() > 10240:  # 10KB chunks
                        mp3_buffer.seek(0)
                        
                        # Decode MP3 to PCM
                        audio = AudioSegment.from_mp3(mp3_buffer)
                        pcm_data = np.array(audio.get_array_of_samples())
                        
                        # Play audio
                        stream.write(pcm_data.tobytes())
                        
                        # Reset buffer
                        mp3_buffer = io.BytesIO()
            
            # Process remaining audio
            if mp3_buffer.tell() > 0:
                mp3_buffer.seek(0)
                audio = AudioSegment.from_mp3(mp3_buffer)
                pcm_data = np.array(audio.get_array_of_samples())
                stream.write(pcm_data.tobytes())
                