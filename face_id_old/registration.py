"""
registration.py
Registers a new user's face by capturing a snapshot from the camera
and saving it to a structured directory for future recognition use.
"""

import cv2
import os
import time
from datetime import datetime

# Load config and debug logging
try:
    from config import CAMERA_SOURCE
    from debug_utils import log_info, log_warning, log_error
except ImportError as e:
    raise ImportError("Could not import config or debug_utils. Ensure paths and environment are set.") from e

# === Directory to save user snapshots ===
SNAPSHOT_DIR = os.path.join(os.path.dirname(__file__), "snapshots")

def ensure_user_folder(username):
    """
    Creates a subdirectory for the given username if it doesn't exist.
    """
    try:
        user_path = os.path.join(SNAPSHOT_DIR, username)
        os.makedirs(user_path, exist_ok=True)
        return user_path
    except Exception as e:
        log_error(f"[Register] Failed to create snapshot directory: {e}")
        return None

def sanitize_username(raw_name):
    """
    Cleans and formats the username input.
    """
    return raw_name.strip().replace(" ", "_")

def register_new_face(username):
    """
    Captures a snapshot of the user's face and saves it to disk.

    Args:
        username (str): The name of the user to register.

    Returns:
        bool: True if capture was successful, False otherwise.
    """
    try:
        log_info(f"[Register] Starting registration for: {username}")

        # Try to access camera
        cap = cv2.VideoCapture(CAMERA_SOURCE)
        if not cap.isOpened():
            log_error("[Register] Failed to access camera.")
            return False

        print("\n[INFO] Please look directly at MiRo and hold still...")
        time.sleep(2)  # Let the user adjust

        ret, frame = cap.read()
        if not ret or frame is None:
            log_error("[Register] Failed to capture frame.")
            cap.release()
            return False

        # Save image with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder = ensure_user_folder(username)
        if not folder:
            cap.release()
            return False

        filename = os.path.join(folder, f"{timestamp}.jpg")
        cv2.imwrite(filename, frame)
        log_info(f"[Register] Snapshot saved at: {filename}")

        cap.release()
        return True

    except Exception as e:
        log_error(f"[Register] Unexpected error during face registration: {str(e)}")
        return False

# === Test Run (CLI Debug Mode) ===
if __name__ == "__main__":
    print("\n=== MiRoGPT Face Registration CLI ===")
    name_input = input("Enter new user's name: ")
    username = sanitize_username(name_input)

    success = register_new_face(username)
    if success:
        print("[SUCCESS] User face registered.")
    else:
        print("[FAILURE] Registration failed.")
