# emotion_extractor.py
# Extracts emotional tone from user input using OpenAI's GPT

import openai
import os

# Load your OpenAI API key (you can set this as an environment variable for security)
openai.api_key = os.getenv("OPENAI_API_KEY")

# Define possible emotions
EMOTION_TAGS = ["happy", "sad", "nostalgic", "frustrated", "curious", "neutral"]

def extract_emotion(user_input, conversation_context=""):
    """
    Returns a single-word emotional tag based on user's latest input.
    Only the tag is returned, not a full GPT response.
    """
    system_prompt = f"""
    You are an emotion classification assistant.
    Determine the emotional tone of the following user input.
    Respond with only one of the following emotions: {', '.join(EMOTION_TAGS)}.
    If unclear, respond with 'neutral'.
    """

    full_prompt = f"Context: {conversation_context}\n\nUser: {user_input}"

    try:
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": full_prompt}
            ],
            max_tokens=5,
            temperature=0.0
        )
        emotion = response.choices[0].message['content'].strip().lower()
        if emotion in EMOTION_TAGS:
            return emotion
        else:
            return "neutral"
    except Exception as e:
        print(f"[ERROR] Emotion extraction failed: {e}")
        return "neutral"


if __name__ == "__main__":
    user_input = input("Enter a message: ")
    detected_emotion = extract_emotion(user_input)
    print(f"Detected emotion: {detected_emotion}")
