#!/usr/bin/env python3

import os
import sys
import rospy
import openai
import logging
from std_msgs.msg import String
from com3528_individual.msg import Emotion

# Add project root to path for shared modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from settings.debug_utils import log_info, log_error
from settings.config import OPENAI_API_KEY, TOPIC_GPT_TRANSCRIPT, TOPIC_GPT_RESPONSE, TOPIC_GPT_EMOTION

# ========== Logging ==========
logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(message)s', force=True)

# ========== Set OpenAI Key ==========
openai.api_key = OPENAI_API_KEY

class MiRoConversationNode:
    def __init__(self):
        rospy.init_node('miro_conversation_node', anonymous=False)
        log_info("[INIT] MiRo GPT Conversation Node launched")

        # Publisher for GPT response (text-to-speech interface)
        self.pub_response = rospy.Publisher(TOPIC_GPT_RESPONSE, String, queue_size=10)

        # Publisher for emotion metadata
        self.pub_emotion = rospy.Publisher(TOPIC_GPT_EMOTION, Emotion, queue_size=10)

        # Subscriber to GPT transcript topic
        rospy.Subscriber(TOPIC_GPT_TRANSCRIPT, String, self.handle_user_input)

        rospy.spin()

    def handle_user_input(self, msg):
        user_input = msg.data.strip()
        if not user_input:
            return

        log_info(f"[INPUT] User: {user_input}")

        try:
            # Query GPT for reply and emotion tagging
            gpt_response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": (
                        "You are a reminiscence therapy robot. Your job is to respond warmly to a user's memory, "
                        "extract the dominant emotion they expressed (e.g., joy, sadness, nostalgia, confusion), and explain why. "
                        "Then format your response as JSON with keys: reply, emotion, reason."
                    )},
                    {"role": "user", "content": user_input}
                ]
            )

            content = gpt_response['choices'][0]['message']['content']
            log_info(f"[GPT] Raw output: {content}")

            # Attempt to parse structured content (GPT should return JSON-like structure)
            import json
            result = json.loads(content)

            reply = result.get("reply", "I'm here to listen.")
            emotion = result.get("emotion", "neutral").lower()
            reason = result.get("reason", "")

            # Publish GPT reply
            self.pub_response.publish(reply)
            log_info(f"[REPLY] GPT: {reply}")

            # Publish emotion as ROS message
            msg_emotion = Emotion()
            msg_emotion.tag = emotion
            msg_emotion.reason = reason
            msg_emotion.confidence = 1.0  # Placeholder – could be refined
            self.pub_emotion.publish(msg_emotion)

            log_info(f"[EMOTION] Tagged as '{emotion}' – {reason}")

        except Exception as e:
            log_error(f"[ERROR] Failed to process GPT response: {e}")


if __name__ == '__main__':
    try:
        MiRoConversationNode()
    except rospy.ROSInterruptException:
        pass
