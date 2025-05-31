# config.py
# Central configuration file for MiRoGPT system

import os

### === OPENAI API === ###
# You can set this as an environment variable for security
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "your-default-fallback-key")

### === FACE ID === ###
FACES_DIR = "face_id/faces"  # Directory for known face images
FACE_MATCH_THRESHOLD = 0.5   # Distance threshold for accepting a face match

### === PROFILES === ###
PROFILES_DIR = "profiles/"   # Folder where user profile JSONs are stored
DEFAULT_PROFILE_NAME = "alex"  # Fallback if no face is matched

### === EMOTIONS === ###
EMOTION_TAGS = ["happy", "sad", "nostalgic", "frustrated", "curious", "neutral"]

### === ROS TOPICS === ###
VOICE_TOPIC = "/miro/command/voice"
HEAD_TOPIC = "/miro/command/head"
MOTOR_TOPIC = "/miro/command/motor"
EARS_TOPIC = "/miro/command/ears"
ILLUM_TOPIC = "/miro/command/illum"

### === SEARCH SETTINGS === ###
SEARCH_TIMEOUT = 90        # Seconds to search for a user
SEARCH_INTERVAL = 5        # Seconds between rotation steps
SEARCH_ANGLE_INCREMENT = 30 # Angle step size (degrees) to simulate scanning

### === GPT MODEL === ###
GPT_MODEL = "gpt-4"  # Can switch to "gpt-3.5-turbo" for lower cost
MAX_GPT_TOKENS = 5
GPT_TEMPERATURE = 0.0

### === LOGGING === ###
LOG_DIR = "logs/"           # Folder to store session logs
SESSION_LOG_FORMAT = "%Y-%m-%d_%H-%M-%S"
