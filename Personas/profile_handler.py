# profile_handler.py

"""
Handles loading, saving, and updating user memory profiles for MiRoGPT.
Integrates with debug_utils and config for centralized control.
"""

import os
import json
from datetime import datetime

# Import shared constants and debug logging utilities
from config import PROFILE_DIR
from debug_utils import (
    log_info,
    log_warning,
    log_error,
    log_profile_action
)

class ProfileHandler:
    """
    Class for managing user profiles in JSON format.
    Supports creation, loading, updating, and persistent storage.
    """

    def __init__(self, profiles_dir=PROFILE_DIR):
        self.profiles_dir = profiles_dir
        os.makedirs(self.profiles_dir, exist_ok=True)  # Ensure directory exists

    def _get_profile_path(self, username):
        """
        Build the full path to a user's profile JSON file.
        """
        return os.path.join(self.profiles_dir, f"{username}.json")

    def load_profile(self, username):
        """
        Load a user profile from disk. If not found, create a blank profile.
        """
        path = self._get_profile_path(username)
        try:
            with open(path, "r") as file:
                data = json.load(file)
            log_profile_action("Loaded", username)
            return data
        except FileNotFoundError:
            # Log and fallback to creating a new profile
            log_warning(f"Profile for {username} not found. Creating new.")
            return self.create_blank_profile(username)
        except Exception as e:
            # Catch and log unexpected I/O or format issues
            log_error(f"Failed to load profile for {username}: {e}")
            return None

    def save_profile(self, username, profile_data):
        """
        Save a user profile JSON to disk. Overwrites existing file.
        """
        path = self._get_profile_path(username)
        try:
            with open(path, "w") as file:
                json.dump(profile_data, file, indent=4)
            log_profile_action("Saved", username)
        except Exception as e:
            log_error(f"Failed to save profile for {username}: {e}")

    def update_profile(self, username, updates_dict):
        """
        Merge new data into the user's profile. Supports:
        - appending to lists (e.g., interaction_log, memories)
        - overwriting fields (e.g., age, condition)
        """
        profile = self.load_profile(username)
        if not profile:
            log_error(f"Cannot update profile — {username} profile missing.")
            return

        for key, value in updates_dict.items():
            # If updating interaction log, append structured entry
            if isinstance(value, dict) and key == "interaction_log":
                if key not in profile:
                    profile[key] = []
                profile[key].append(value)
            # If it's a list field (e.g., medical_history), append to it
            elif isinstance(profile.get(key), list):
                profile[key].append(value)
            else:
                # Otherwise, treat as scalar replacement
                profile[key] = value

        # Update timestamp for audit trail
        profile["last_updated"] = datetime.now().isoformat()
        self.save_profile(username, profile)

    def create_blank_profile(self, username):
        """
        Create a fresh profile for a new user, pre-filled with required fields.
        """
        template = {
            "name": username,
            "age": None,
            "ethnicity": None,
            "condition": None,
            "medical_history": [],
            "reminders": [],
            "memories": [],
            "interaction_log": [],
            "last_updated": datetime.now().isoformat()
        }
        self.save_profile(username, template)
        return template

# === Manual test stub for development ===
if __name__ == "__main__":
    handler = ProfileHandler()
    
    # Example test: simulate user interaction update
    sample_update = {
        "interaction_log": {
            "timestamp": datetime.now().isoformat(),
            "input": "Do you remember the church bells?",
            "response": "Yes, that was before our Sunday roast.",
            "emotion": "nostalgic"
        }
    }

    handler.update_profile("George_Taylor", sample_update)
    profile = handler.load_profile("George_Taylor")
    print(json.dumps(profile, indent=4))
