import logging
import sys
import os
from datetime import datetime

# === Ensure logs directory exists ===
LOG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'logs'))
os.makedirs(LOG_DIR, exist_ok=True)

# === Create timestamped log file ===
timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
log_file = os.path.join(LOG_DIR, f'face_node_log_{timestamp}.log')

# === Setup logging ===
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler(sys.stdout)
    ],
    force=True  # Ensures all modules adopt this format
)

# === Color codes (terminal only) ===
COLOR_RESET = "\033[0m"
COLOR_INFO = "\033[94m"
COLOR_WARN = "\033[93m"
COLOR_ERROR = "\033[91m"
COLOR_FACE = "\033[95m"
COLOR_EMOTION = "\033[96m"
COLOR_GPT = "\033[92m"
COLOR_CONVO = "\033[90m"

# === Logging functions ===
def log_info(message):
    logging.info(f"{COLOR_INFO}{message}{COLOR_RESET}")

def log_warning(message):
    logging.warning(f"{COLOR_WARN}{message}{COLOR_RESET}")

def log_error(message):
    logging.error(f"{COLOR_ERROR}{message}{COLOR_RESET}")

def log_face_recognition(message):
    logging.debug(f"{COLOR_FACE}[FaceRec] {message}{COLOR_RESET}")

def log_face_tracking(message):
    logging.debug(f"{COLOR_FACE}[FaceTrack] {message}{COLOR_RESET}")

def log_centering(message):
    logging.debug(f"{COLOR_FACE}[Centering] {message}{COLOR_RESET}")

def log_emotion(message):
    logging.debug(f"{COLOR_EMOTION}[Emotion] {message}{COLOR_RESET}")

def log_gpt_response(message):
    logging.debug(f"{COLOR_GPT}[GPT] {message}{COLOR_RESET}")

def log_conversation(message):
    logging.debug(f"{COLOR_CONVO}[Convo] {message}{COLOR_RESET}")
