import openai
import subprocess
import whisper
import time
from settings.config import OPENAI_API_KEY, MIRo_TTS_COMMAND
from settings.debug_utils import log_info, log_warning, log_error

# === Load Whisper model (CPU-based, use 'base' for speed) ===
model = whisper.load_model("base")

# === GPT call using openai>=1.0.0 ===
def chat_with_gpt(prompt):
    client = openai.OpenAI(api_key=OPENAI_API_KEY)
    log_info("Sending message to GPT...")
    try:
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}]
        )
        reply = response.choices[0].message.content
        log_info("Received GPT response.")
        return reply
    except Exception as e:
        log_error(f"OpenAI API error: {e}")
        return "I'm sorry, I couldn't connect to GPT just now."

# === Whisper STT ===
def transcribe_whisper():
    import speech_recognition as sr
    recogniser = sr.Recognizer()
    with sr.Microphone() as source:
        log_info("Listening (Whisper)...")
        audio = recogniser.listen(source)

        # Save to temp WAV file
        with open("temp.wav", "wb") as f:
            f.write(audio.get_wav_data())

    result = model.transcribe("temp.wav")
    log_info(f"[Whisper STT] You said: {result['text']}")
    return result['text']

# === TTS function to speak via MiRo ===
def speak_with_miro(text):
    try:
        cmd = MIRo_TTS_COMMAND.split() + [text]
        subprocess.run(cmd, check=True)
    except Exception as e:
        log_error(f"Failed to run TTS command: {e}")

# === Main interaction loop ===
def main():
    log_info("MiRo is ready to chat with Whisper!")
    try:
        while True:
            user_input = transcribe_whisper()
            if not user_input.strip():
                log_warning("No input detected. Try again.")
                continue
            gpt_reply = chat_with_gpt(user_input)
            print(f"GPT: {gpt_reply}")
            speak_with_miro(gpt_reply)
            time.sleep(1)
    except KeyboardInterrupt:
        log_info("Session ended by user.")

if __name__ == '__main__':
    main()
