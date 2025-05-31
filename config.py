# config.py
# Central configuration file for MiRoGPT system

import os

# === OPENAI API ===
# Use environment variable for security
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "your-default-fallback-key")

# === SYSTEM PATHS ===
PROFILE_DIR = "profiles/"
LOG_DIR = "logs/"
FALLBACK_LOG = os.path.join(LOG_DIR, "fallback_invalid_emotion.jsonl")
SNAPSHOT_DIR = "face_id/snapshots/"  # Folder for user face image data

# === ROS TOPICS ===
ROS_TOPIC_EARS = "/miro/command/ears"
ROS_TOPIC_HEAD = "/miro/command/head"
ROS_TOPIC_MOTOR = "/miro/command/motor"
ROS_TOPIC_ILLUM = "/miro/command/illum"
ROS_TOPIC_VOICE = "/miro/command/voice"
ROS_TOPIC_EMOTION = "/miro/emotion_tag"

# === EMOTION TAGS ===
EMOTION_TAGS = [
    "happy", "sad", "nostalgic", "neutral", "angry", "curious"
]

# === FACE RECOGNITION ===
FACE_MATCH_THRESHOLD = 0.6
CAMERA_SOURCE = 0  # Default webcam; replace with ROS topic if using MiRo-E camera

# === GPT SETTINGS ===
GPT_MODEL = "gpt-4"
MAX_TOKENS = 100
TEMPERATURE = 0.7
DEBUG_GPT = True

# === SYSTEM BEHAVIOR ===
IDLE_TIMEOUT = 30              # Seconds before MiRo enters idle mode
SESSION_MAX_LENGTH = 300       # Max seconds for one reminiscence session
SEARCH_TIMEOUT = 90            # Face search max duration (seconds)
SEARCH_INTERVAL = 5            # Delay between each sweep attempt (seconds)

# === DEBUG FLAGS ===
DEBUG_FACE_RECOGNITION = True
DEBUG_PROFILE_IO = True
DEBUG_EMOTION_LOGGING = True
