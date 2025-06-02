# debug_utils.py — unified logging utilities for MiRo face tracking system

import logging
import sys

# === Setup basic logger ===
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(levelname)s] %(message)s',
    stream=sys.stdout,
    force=True  # Ensures it overrides any existing logging configs
)

# === ANSI color codes for clarity (can disable if needed) ===
COLOR_RESET = "\033[0m"
COLOR_INFO = "\033[94m"
COLOR_WARN = "\033[93m"
COLOR_ERROR = "\033[91m"
COLOR_FACE = "\033[95m"

# === Wrapper functions ===

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
