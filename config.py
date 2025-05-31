# config.py
# Central configuration file for MiRoGPT system

import os

### === OPENAI API === ###
# You can set this as an environment variable for security
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "your-default-fallback-key")

# === SYSTEM PATHS ===
PROFILE_DIR = "profiles/"
LOG_DIR = "logs/"
FALLBACK_LOG = os.path.join(LOG_DIR, "fallback_invalid_emotion.jsonl")

# === ROS TOPICS ===
ROS_TOPIC_EARS = "/miro/command/ears"
ROS_TOPIC_HEAD = "/miro/command/head"
ROS_TOPIC_MOTOR = "/miro/command/motor"
ROS_TOPIC_ILLUM = "/miro/command/illum"
ROS_TOPIC_VOICE = "/miro/command/voice"
ROS_TOPIC_EMOTION = "/miro/emotion_tag"

# === EMOTION TAGS ===
EMOTION_TAGS = ["happy", "sad", "nostalgic", "neutral", "angry", "curious"]

# === FACE RECOGNITION ===
FACE_MATCH_THRESHOLD = 0.6
CAMERA_SOURCE = 0  # default webcam; replace with ROS topic if using MiRo-E feed

# === GPT SETTINGS ===
GPT_MODEL = "gpt-4"
MAX_TOKENS = 100
TEMPERATURE = 0.7
DEBUG_GPT = True

# === SYSTEM BEHAVIOR ===
IDLE_TIMEOUT = 30  # seconds before MiRo goes into idle if no user
SESSION_MAX_LENGTH = 300  # max seconds for one reminiscence session

# === DEBUG FLAGS ===
DEBUG_FACE_RECOGNITION = True
DEBUG_PROFILE_IO = True
DEBUG_EMOTION_LOGGING = True
