# config.py
# Unified and enhanced configuration module for MiRoGPT

import os

# === File Paths ===
# New (absolute paths using __file__)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FACES_DIR = os.path.join(BASE_DIR, "face_id", "user_images")  # Directory for user profiles (known users)
SNAPSHOTS_DIR = os.path.join(BASE_DIR, "face_id", "snapshots")  # Directory for temporary unknown face captures
PROFILES_DIR = "profiles"                  # User profile memory storage (JSON)
LOGS_DIR = "logs"                          # Session logs and summaries
FALLBACK_LOG = os.path.join(LOGS_DIR, "fallback_invalid_emotion.log")

# === GPT Configuration ===
GPT_MODEL = "gpt-4"
OPENAI_API_KEY_ENV_VAR = "OPENAI_API_KEY"
OPENAI_API_KEY = os.getenv(OPENAI_API_KEY_ENV_VAR, "your-default-fallback-key")
MAX_TOKENS_DIALOGUE = 150
MAX_TOKENS_EMOTION = 60
TEMPERATURE = 0.7
DEBUG_GPT = True

# === Emotion Tags ===
EMOTION_TAGS = ["happy", "sad", "angry", "neutral", "nostalgic", "curious"]

# === Face Recognition Parameters ===
FACE_MATCH_THRESHOLD = 0.6
CAMERA_SOURCE = "/miro/sensors/camr/compressed"  # Default webcam; replace with ROS topic if using MiRo-E camera

# === ROS Topics (MiRo Commands) ===
TOPIC_CMD_VEL = "/miro/command/velocity"
TOPIC_CMD_EARS = "/miro/command/ears"
TOPIC_CMD_HEAD = "/miro/command/head"
TOPIC_CMD_LIGHT = "/miro/command/illum"
TOPIC_CMD_TONE = "/miro/command/tone"
TOPIC_EMOTION_TAG = "/miro/emotion_tag"  # Custom topic for emotion broadcasting

# === System Behavior ===
IDLE_TIMEOUT_SEC = 30              # Seconds before MiRo enters idle mode
SESSION_MAX_LENGTH = 300           # Max seconds for one reminiscence session
SEARCH_TIMEOUT = 90                # Face search max duration (seconds)
SEARCH_INTERVAL = 5                # Delay between each sweep attempt (seconds)

# === Mode Flags ===
REMINISCENCE_MODE = False   # Will be toggled by command phrase
EXPLORE_MODE = True         # Default fallback when user not recognised or reminiscence not triggered

# === Debug Flags ===
DEBUG_FACE_RECOGNITION = True
DEBUG_PROFILE_IO = True
DEBUG_EMOTION_LOGGING = True
DEBUG_ROS_TOPICS = False
