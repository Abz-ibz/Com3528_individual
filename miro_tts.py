# miro_tts.py

from gtts import gTTS
import os
import sys
import tempfile
import subprocess

def speak(text):
    with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
        tts = gTTS(text)
        tts.save(fp.name)
        wav_path = fp.name.replace(".mp3", ".wav")
        subprocess.run(["ffmpeg", "-y", "-i", fp.name, wav_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["aplay", wav_path])

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 miro_tts.py 'Hello world'")
    else:
        speak(" ".join(sys.argv[1:]))
