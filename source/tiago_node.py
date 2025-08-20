import rospy
import actionlib 
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import threading
import json
import random
from openai import OpenAI
from trajectory_msgs.msg import JointTrajectoryPoint 
from gtts import gTTS 
from playsound import playsound 
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
from config import *
from source.agents import llm_call
from source.utils import *
import tempfile

# Disable GLib CRITICAL messages globally
GLib.log_set_handler(None, GLib.LogLevelFlags.LEVEL_CRITICAL, lambda *a: None)
Gst.init(None)

class TiagoNode:
    def __init__(self, task, reasoning, model):
        self.task = task
        self.reasoning = reasoning
        self.model = model

        # Create the PlayMotion action client
        self.client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
        self.client.wait_for_server()

        # Base controller client 
        self.base_client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.base_client.wait_for_server()

        self.stop_nod = threading.Event()
        self.nod_thread = None
        
    def speak(self, text):
        # Create a temporary file
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmp_file:
            temp_path = tmp_file.name

        # Convert text to speech and save
        tts = gTTS(text, lang='en')
        tts.save(temp_path)

        # Play the audio
        playsound(temp_path)

        # Remove the temporary file
        os.remove(temp_path)

    def send_motion(self, motion_name, blocking=True):    
        goal = PlayMotionGoal()
        goal.motion_name = motion_name
        goal.skip_planning = False
        goal.priority = 0
        self.client.send_goal(goal)

        if blocking:
            self.client.wait_for_result()

    def send_nod(self, blocking=True):
        traj_goal = FollowJointTrajectoryGoal()
        traj_goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]

        # Create a simple nodding trajectory: down, up, center
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.3]  # nod down angles (radians)
        point1.time_from_start = rospy.Duration(1.0)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.0, -0.3]  # nod up angles
        point2.time_from_start = rospy.Duration(2.0)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0]  # center position
        point3.time_from_start = rospy.Duration(3.0)

        traj_goal.trajectory.points = [point1, point2, point3]

        self.base_client.send_goal(traj_goal)
        if blocking:
            self.base_client.wait_for_result()

    def nod_loop(self):
        while not self.stop_nod.is_set() and not rospy.is_shutdown():
            self.send_nod()

    def start_nodding(self):
        if self.nod_thread is None or not self.nod_thread.is_alive():
            self.stop_nod.clear()
            self.nod_thread = threading.Thread(target=self.nod_loop)
            self.nod_thread.start()

    def stop_nodding(self):
        self.stop_nod.set()
        if self.nod_thread:
            self.nod_thread.join()

    def run(self):
        tiago_label = "Tiago: "
        patient_label = "Patient: "
        self.send_motion("wave")
        sentence1 = "Hi, I'm Tiago. Welcome to our hospital."
        self.speak(sentence1)
        print(tiago_label+sentence1)
        self.start_nodding()
        user_prompt = input(patient_label)
        self.stop_nodding()

        sentence2 = "Can you give me your full name?"
        print(tiago_label+sentence2)
        self.start_nodding()
        self.speak(sentence2)
        name = input('Patient name: ')
        surname = input('Patient surname: ')
        self.stop_nodding()

        self.send_motion("offer")
        features = get_patient_features(name, surname)

        while features is None:
            print(tiago_label+"Name is incorrect, try again")
            self.speak("Name is incorrect, try again")
            self.start_nodding()
            name = input('Patient name: ')
            surname = input('Patient surname: ')
            self.stop_nodding()
            features = get_patient_features(name, surname)

        self.send_motion("thumb_up_hand")
        sentence3 = "Perfect, thank you."
        self.speak(sentence3)
        print(tiago_label+sentence3)

        self.send_motion("offer")
        print(tiago_label+"Do you need something?")
        self.speak("Do you need something?")
        SYSTEM_PROMPT_PATH = PROMPT_DIR + f"system_prompt{'_reasoning' if self.reasoning=='true' else ''}.txt"
        system_prompt = open(SYSTEM_PROMPT_PATH).read()
        system_prompt = system_prompt.replace("<KB_features>", str(features))

        CBT_techniques = ["cognitive restructuring", "coping strategy"]
        cbt = random.choice(CBT_techniques)
        system_prompt = system_prompt.replace("<CBT_technique>", cbt)
        vlm_answer = get_knowledge_graph(self.task)
        system_prompt = system_prompt.replace("<output_smk>", vlm_answer)

        chat_history = [
            {"role": "system", "content": f"{system_prompt}"},
            {"role": "assistant", "content": sentence1},
            {"role": "user", "content": user_prompt},
            {"role": "assistant", "content": sentence2},
            {"role": "user", "content": name + " " + surname},
            {"role": "assistant", "content": sentence3 + "\n" + "Do you need something?"},
        ]

        while "goodbye" not in user_prompt:
            self.start_nodding()
            user_prompt = input(patient_label)
            self.stop_nodding()

            if "I need help because" in user_prompt:
                if features["priority"] != "Low":
                    self.send_motion("reach_max")
                    print(tiago_label+"Attention please, the patient needs help")
                    self.speak("Attention please, the patient needs help")
                    self.send_motion("offer")
                print(tiago_label+"I'm scanning the environment to find the best solution, be patient")
                self.speak("I'm scanning the environment to find the best solution, be patient")
                self.send_motion("head_tour")
                self.send_motion("pointing_hand")
	
            chat_history.append({"role": "user", "content": user_prompt})
            raw_answer = llm_call(chat_history, self.model)
            answer = parse_answer(raw_answer)
            print(tiago_label+answer)
            self.speak(answer)
            chat_history.append({"role": "assistant", "content": raw_answer})
            if "I need help because" in user_prompt:
                self.send_motion("offer")

        self.send_motion("wave")
        CHAT_HISTORY_DIR= DATA_DIR + "chat_history/"
        with open(CHAT_HISTORY_DIR + f"chat_history_{self.model}_{'reasoning_' if self.reasoning=='true' else ''}{self.task}.txt",
                  "w", encoding="utf-8") as f:
            json.dump(chat_history, f, ensure_ascii=False, indent=2)


