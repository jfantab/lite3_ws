import os
import base64
import pyaudio
import websockets
import json
from dotenv import load_dotenv
import asyncio

### API CONFIG ###

load_dotenv()

os.environ["OPENAI_API_KEY"] = os.getenv("OPENAI_API_KEY")

OPENAI_API_KEY = os.environ["OPENAI_API_KEY"]

BASE_URL = "wss://api.openai.com/v1/realtime"
url = BASE_URL + "?model=gpt-4o-realtime-preview-2024-12-17"
headers = {
    "Authorization": "Bearer " + OPENAI_API_KEY,
    "OpenAI-Beta": "realtime=v1"
}

### CHECKING DEVICES ###

# audio = pyaudio.PyAudio()
# info = audio.get_host_api_info_by_index(0)
# numdevices = info.get('deviceCount')
# for i in range(0, numdevices):
#         if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
#             print("Input Device id ", i, " - ", audio.get_device_info_by_host_api_device_index(0, i).get('name'))

### SOCKETS ###

async def initialize_session(ws):
    session_update = {
            "type": "session.update",
            "modalities": ["audio", "text"],
            "voice": "alloy", 
            "input_audio_format": "pcm16",
            "output_audio_format": "pcm16",
            "input_audio_transcription": {"model": "whisper-1"},
            "turn_detection": {"type": "server_vad"},
            "temperature": 0.8,
            "max_response_output_tokens": 4096
    }

    await ws.send(json.dumps(session_update))

async def conversation_loop(ws):

    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 24000 
    CHUNK = 1024*4

    audio = pyaudio.PyAudio()
    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    output_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, output=True)

    try:
        while True:
            print("Listening...")
            
            frames = []
            for _ in range(0, int(RATE / CHUNK * 3)):
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)

            audio_bytes = b''.join(frames)
            encoded = base64.b64encode(audio_bytes).decode("utf-8")

            await ws.send(json.dumps({
                "type": "input_audio_buffer.append", 
                "audio": encoded
            }))
            await ws.send(json.dumps({
                "type": "input_audio_buffer.commit"
            }))

            print("Waiting for GPT response...")
            while True:
                result = await ws.recv()
                event = json.loads(result)

                if event["type"] == "response.audio.delta":
                    audio_data = base64.b64decode(event["delta"])
                    
                    output_stream.write(audio_data)
                    
                if event["type"] == "response.done":
                    break
            
    except Exception as e:
        print(f"Exception happened: {e}")
    finally:
        stream.stop_stream()
        stream.close()
        output_stream.stop_stream()
        output_stream.close()
        audio.terminate()

async def start():
    async with websockets.connect(url, additional_headers=headers) as ws:
        await initialize_session(ws)
        await conversation_loop(ws)

if __name__ == "__main__":
    asyncio.run(start())
