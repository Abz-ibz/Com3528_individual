# config.py — unified configuration file for MiRoGPT

import os
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# === Base Directory Structure ===
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
FACES_DIR = os.path.join(BASE_DIR, 'faces')
MODELS_DIR = os.path.join(BASE_DIR, 'models')
SNAPSHOTS_DIR = os.path.join(FACES_DIR, 'snapshots')
PATIENTS_DIR = os.path.join(FACES_DIR, 'patients')
LOGS_DIR = os.path.join(BASE_DIR, 'logs')

# === API & MiRo Integration ===
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "missing-key")
MIRo_TTS_COMMAND = "python3 /home/student/pkgs/mdk-230105/catkin_ws/src/com3528_individual/miro_tts.py"

# === Camera Sources (ROS topics) ===
CAMERA_SOURCE_LEFT = "/miro/sensors/caml/compressed"
CAMERA_SOURCE_RIGHT = "/miro/sensors/camr/compressed"

# === ROS Topics ===
TOPIC_FACE_ID = "/miro/face_id"
TOPIC_FACE_TRACK = "/miro/face_track"
TOPIC_FACE_CENTER = "/miro/face_center"

# === ROS Settings ===
ROS_RATE = 10  # Hz

# === Frame and Centring Parameters ===
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CENTERING_TOLERANCE = 0.1
CENTERING_GAIN = 0.7
MAX_HEAD_YAW = 0.6
MAX_HEAD_PITCH = 0.4

# === Face Recognition Parameters ===
FACE_MATCH_THRESHOLD = 0.6
SHAPE_PREDICTOR_PATH = os.path.join(MODELS_DIR, "shape_predictor_68_face_landmarks.dat")
FACE_RECOGNITION_MODEL_PATH = os.path.join(MODELS_DIR, "dlib_face_recognition_resnet_model_v1.dat")

# === Debug Flags ===
DEBUG_FACE_RECOGNITION = True
DEBUG_FACE_TRACKING = True
DEBUG_FACE_CENTERING = True
DEBUG_CONVERSATION_FLOW = True
DEBUG_EMOTION_TAGGING = True

# === Visual / Logging Options ===
SHOW_LIVE_FEED = True
USE_VERBOSE_LOGS = True
