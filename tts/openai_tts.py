import os
from pathlib import Path
from openai import OpenAI
import subprocess
from dotenv import load_dotenv

load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

VOICE="ash"

departments = ["mscs", "msse", "nursing", "psych"]

for d in departments:

    with open(f"txt/{d}.txt", "r") as fp:
        text = fp.read()

    speech_file_path_mp3 = Path(__file__).parent / "mp3" / f"{d}.mp3"
    speech_file_path_wav_long = Path(__file__).parent / "wav_long" / f"{d}.wav"
    speech_file_path_wav = Path(__file__).parent / "wav" / f"{d}.wav"

    with client.audio.speech.with_streaming_response.create(
        model="gpt-4o-mini-tts",
        voice=VOICE,
        input=text,
        instructions="Speak in a cheerful and positive tone.",
    ) as response:
        response.stream_to_file(speech_file_path_mp3)
    
    subprocess.run([
        "ffmpeg", "-i",
        speech_file_path_mp3, 
        "-ar", "24000",
        "-ac", "1",
        "-sample_fmt",
        "s16",
        speech_file_path_wav_long
    ])

    subprocess.run([
        "ffmpeg", "-ss", "0", 
        "-i", speech_file_path_wav_long,
        "-t", "5", 
        "-c", "copy",
        speech_file_path_wav
    ])

# ffmpeg -ss $start -i "~/tts/wav_long/$file" -t $duration -c copy "~/tts/wav/$file";

