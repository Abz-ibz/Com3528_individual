# config.py — central configuration file for MiRoGPT facial recognition & tracking

import os

# === Camera Sources ===
CAMERA_SOURCE_LEFT = "/miro/sensors/caml/compressed"  # Index for left eye
CAMERA_SOURCE_RIGHT = "/miro/sensors/camr/compressed"  # Index for right eye

# === Directory Paths ===
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
FACES_DIR = os.path.join(BASE_DIR, 'faces')
MODELS_DIR = os.path.join(BASE_DIR, 'models')

# === Face Recognition ===
FACE_MATCH_THRESHOLD = 0.6  # Lower is stricter
SHAPE_PREDICTOR_PATH = os.path.join(MODELS_DIR, "shape_predictor_68_face_landmarks.dat")
FACE_RECOGNITION_MODEL_PATH = os.path.join(MODELS_DIR, "dlib_face_recognition_resnet_model_v1.dat")

# === Topics (custom ROS messages or used topics) ===
TOPIC_FACE_ID = '/miro/face_id'
TOPIC_FACE_CENTER = '/miro/face_center'
TOPIC_FACE_TRACK = '/miro/face_track'

# === Debugging Flags ===
DEBUG_FACE_RECOGNITION = True
DEBUG_FACE_TRACKING = True
DEBUG_FACE_CENTERING = True

# === Tracking and Centering Parameters ===
CENTERING_GAIN = 0.7         # Tuning gain for correction
MAX_HEAD_YAW = 0.6           # Maximum head yaw in radians (≈ 34.4°)
MAX_HEAD_PITCH = 0.4         # Maximum pitch adjustment

# Camera / Frame dimensions
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Tracking/centering tolerance
CENTERING_TOLERANCE = 0.1

# === Display / Logging Preferences ===
SHOW_LIVE_FEED = True
USE_VERBOSE_LOGS = True

# === General ROS Parameters ===
ROS_RATE = 10  # Hz
