# debug_utils.py
"""
Debugging Utilities for MiRoGPT
Provides standardized logging and fallback mechanisms for diagnostics and post-hoc analysis.
"""

import logging
import os
import json
from datetime import datetime

# Attempt to import config; provide clear error if not found
try:
    from config import (
        LOG_DIR,
        FALLBACK_LOG,
        DEBUG_GPT,
        DEBUG_PROFILE_IO,
        DEBUG_FACE_RECOGNITION,
        DEBUG_EMOTION_LOGGING,
    )
except ImportError as e:
    raise ImportError("❌ Failed to import from config.py. Ensure config.py exists and paths are correct.") from e

# === Ensure log directory exists ===
try:
    os.makedirs(LOG_DIR, exist_ok=True)
except Exception as e:
    print(f"[ERROR] Failed to create log directory '{LOG_DIR}': {e}")
    LOG_DIR = "."  # Fallback to current directory

# === Configure logging output ===
try:
    logging.basicConfig(
        filename=os.path.join(LOG_DIR, "mirogpt_debug.log"),
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
except Exception as e:
    print(f"[ERROR] Logging configuration failed: {e}")

# === Generic logging wrappers ===

def log_debug(msg):
    """Log a debug message with optional console print."""
    try:
        logging.debug(msg)
        print(f"[DEBUG] {msg}")
    except Exception as e:
        print(f"[ERROR] Debug logging failed: {e}")

def log_info(msg):
    """Log an info message with optional console print."""
    try:
        logging.info(msg)
        print(f"[INFO] {msg}")
    except Exception as e:
        print(f"[ERROR] Info logging failed: {e}")

def log_warning(msg):
    """Log a warning message with optional console print."""
    try:
        logging.warning(msg)
        print(f"[WARNING] {msg}")
    except Exception as e:
        print(f"[ERROR] Warning logging failed: {e}")

def log_error(msg):
    """Log an error message with optional console print."""
    try:
        logging.error(msg)
        print(f"[ERROR] {msg}")
    except Exception as e:
        print(f"[ERROR] Error logging failed: {e}")

# === Specific usage logs ===

def log_gpt_response(prompt, response):
    """Logs full GPT prompt + response if DEBUG_GPT is enabled."""
    if DEBUG_GPT:
        try:
            log_debug("--- GPT PROMPT ---")
            log_debug(prompt)
            log_debug("--- GPT RESPONSE ---")
            log_debug(str(response))
        except Exception as e:
            print(f"[ERROR] GPT response logging failed: {e}")

def log_fallback_emotion(input_text, response_obj):
    """Logs invalid GPT output to fallback file for offline analysis."""
    if DEBUG_GPT:
        fallback_data = {
            "timestamp": datetime.now().isoformat(),
            "input_text": input_text,
            "response": str(response_obj)
        }
        try:
            with open(FALLBACK_LOG, "a") as f:
                f.write(json.dumps(fallback_data) + "\n")
            log_warning("Logged fallback GPT response.")
        except Exception as e:
            print(f"[ERROR] Fallback log write failed: {e}")

def log_profile_action(action, profile_name):
    """Logs profile reads, writes, or updates."""
    if DEBUG_PROFILE_IO:
        try:
            log_info(f"[Profile] Action: {action} – User: {profile_name}")
        except Exception as e:
            print(f"[ERROR] Profile logging failed: {e}")

def log_face_recognition(event):
    """Logs face recognition-related messages."""
    if DEBUG_FACE_RECOGNITION:
        try:
            log_info(f"[FaceRec] {event}")
        except Exception as e:
            print(f"[ERROR] Face recognition logging failed: {e}")

def log_emotion_event(tag, source):
    """Logs final tagged emotion and context (e.g., from GPT or direct input)."""
    if DEBUG_EMOTION_LOGGING:
        try:
            log_info(f"[Emotion] Detected '{tag}' from: {source}")
        except Exception as e:
            print(f"[ERROR] Emotion event logging failed: {e}")
