# emotion_extractor.py

import openai
import logging
import re
from config import (
    OPENAI_API_KEY,      # API key for OpenAI GPT access
    GPT_MODEL,           # Model selection (e.g., 'gpt-4')
    EMOTION_TAGS,        # Allowed emotion tags (strict list)
    DEBUG_GPT            # Debugging flag for verbose output
)
from gpt_prompt import GPT_PROMPT_TEMPLATE  # Externalised prompt template for cleaner logic

# Set OpenAI API key
openai.api_key = OPENAI_API_KEY

# Configure logging format
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

def extract_emotion(text):
    """
    Extracts the dominant emotional tone from a given input string
    using OpenAI's GPT API. Returns a strict tag match or 'neutral'.
    """
    # Fill the prompt template with the allowed emotion list and user input
    prompt = GPT_PROMPT_TEMPLATE.format(
        emotion_list=', '.join(EMOTION_TAGS),
        user_input=text
    )

    try:
        # Query GPT model with system and user messages
        response = openai.ChatCompletion.create(
            model=GPT_MODEL,
            messages=[
                {"role": "system", "content": "You are an expert emotion recogniser."},
                {"role": "user", "content": prompt}
            ],
            temperature=0
        )

        # Log full response if debugging is on
        if DEBUG_GPT:
            logging.debug(f"GPT full response object: {response}")

        # Extract the raw output from GPT
        raw_output = response.choices[0].message['content'].strip().lower()

        # Log raw extracted content
        if DEBUG_GPT:
            logging.debug(f"GPT raw content: {raw_output}")

        # Match response against valid emotion tags
        matched_emotion = re.match(r'^(' + '|'.join(re.escape(tag) for tag in EMOTION_TAGS) + r')$', raw_output)

        if matched_emotion:
            emotion = matched_emotion.group(1)
            logging.info(f"Extracted emotion: {emotion}")
            return emotion
        else:
            logging.warning(f"Invalid emotion response: '{raw_output}'. Defaulting to 'neutral'.")
            # Log unclassified results for manual review
            with open("logs/fallback_emotion_log.txt", "a") as f:
                f.write(f"INPUT: {text}\nRESPONSE: {raw_output}\n\n")
            return "neutral"

    except Exception as e:
        logging.error(f"Error extracting emotion: {e}")
        # Log the failure input and error message
        with open("logs/fallback_emotion_log.txt", "a") as f:
            f.write(f"INPUT: {text}\nERROR: {str(e)}\n\n")
        return "neutral"

if __name__ == "__main__":
    # Simple test cases for local validation
    test_inputs = [
        "I miss my family so much lately.",
        "That time we danced all night was amazing!",
        "I’m not really in the mood to talk."
    ]

    for t in test_inputs:
        print(f"Input: {t}\nDetected Emotion: {extract_emotion(t)}\n")
