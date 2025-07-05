import pyaudio
import wave
import whisper

def record_audio():
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK = 1024
    RECORD_SECONDS = 5

    audio = pyaudio.PyAudio()

    print("Recording now...")

    stream = audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK,
        )

    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    stream.stop_stream()
    stream.close()
    audio.terminate()

    print("Recording finished")

    wavFile = wave.open("output.wav", "wb")
    wavFile.setnchannels(CHANNELS)
    wavFile.setsampwidth(audio.get_sample_size(FORMAT))
    wavFile.setframerate(RATE)
    wavFile.writeframes(b''.join(frames))
    wavFile.close()

def whisper_transcribe():
    print("Transcribing")

    model = whisper.load_model("base")
    result = model.transcribe("output.wav")
    print(result["text"])

    return result["text"]

