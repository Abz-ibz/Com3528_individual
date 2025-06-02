import os
import datetime
import logging
from config import (
    LOGS_DIR,
    DEBUG_FACE_RECOGNITION,
    DEBUG_PROFILE_IO,
    DEBUG_EMOTION_LOGGING,
    DEBUG_ROS_TOPICS
)

# Ensure logs directory exists
os.makedirs(LOGS_DIR, exist_ok=True)

# Define log file path
DEBUG_LOG_PATH = os.path.join(LOGS_DIR, "miro_debug.log")

# Configure root logger
logging.basicConfig(
    filename=DEBUG_LOG_PATH,
    filemode='a',
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    level=logging.DEBUG
)

# Console output as fallback
console = logging.StreamHandler()
console.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(message)s')
console.setFormatter(formatter)
logging.getLogger('').addHandler(console)

def log_info(message):
    logging.info(message)

def log_warning(message):
    logging.warning(message)

def log_error(message):
    logging.error(message)

def log_critical(message):
    logging.critical(message)

def log_face_recognition(message):
    if DEBUG_FACE_RECOGNITION:
        logging.debug(f"[FACE] {message}")

def log_profile_io(message):
    if DEBUG_PROFILE_IO:
        logging.debug(f"[PROFILE_IO] {message}")

def log_emotion(message):
    if DEBUG_EMOTION_LOGGING:
        logging.debug(f"[EMOTION] {message}")

def log_ros(message):
    if DEBUG_ROS_TOPICS:
        logging.debug(f"[ROS] {message}")
