import os
from openai import OpenAI
from dotenv import load_dotenv

import pyaudio
import io
import numpy as np
from pydub import AudioSegment

load_dotenv()

class Responder():

    def __init__(self):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        self.VOICE = "ash"
        self.FORMAT = pyaudio.paInt16
        self.RATE = 24000
        self.CHANNELS = 1
        self.CHUNK_SIZE = 4096  # Size of each chunk to read from the stream

    def dictate(text):
        audio = pyaudio.PyAudio()
        stream = audio.open(format=self.FORMAT, rate=self.RATE, channels=self.CHANNELS, output=True)

        with self.client.audio.speech.with_streaming_response.create(
                model="gpt-4o-mini-tts",
                voice=VOICE,
                input=text,
                response_format="mp3",
                instructions="Speak in a cheerful and positive tone.",
            ) as response:
                mp3_buffer = io.BytesIO()
                
                # Stream chunks from OpenAI
                for chunk in response.iter_bytes(chunk_size=self.CHUNK_SIZE):
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

                stream.stop_stream()
                stream.close()
                audio.terminate()
                    