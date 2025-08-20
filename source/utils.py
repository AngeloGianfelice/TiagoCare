import os
from datetime import datetime
from config import * 
import re
import json

def create_task_folder(log_dir):
    # Ensure the output directory exists
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Full path for the new task folder
    task_path = os.path.join(OUTPUT_DIR, log_dir)
    
    # Create the task folder
    os.makedirs(task_path, exist_ok=True)
    
    print(f"Created folder: {task_path}")
    return task_path+'/'

def parse_answer(answer):
    match = re.search(r'ANSWER:\s*(.*)', answer, re.IGNORECASE | re.DOTALL)
    if match:
        return match.group(1).strip()
    return answer

def get_patient_features(name, surname):
    KB_PATH = DATA_DIR + "hospital_patients.json"
    with open(KB_PATH, 'r') as file:
        data = json.load(file)
    for patient in data['patients']:
        if patient['name'] == name and patient['surname'] == surname:
            return patient
    return None

def get_knowledge_graph(task):
    # List all existing task folders in OUTPUT_DIR
    existing_tasks = [
        name for name in os.listdir(OUTPUT_DIR)
        if os.path.isdir(os.path.join(OUTPUT_DIR, name))
    ]

    if task not in existing_tasks:
        raise ValueError(
            f"Task '{task}' does not exist in the output folder, you need to generate it following the Semantic Knowledge Agent pipeline."
            f"Available tasks: {existing_tasks}"
        )

    # Path to the file inside the task folder
    file_path = os.path.join(OUTPUT_DIR, task, "vlm_answer.txt")
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Could not find file: {file_path}")

    # Read and return the file content
    with open(file_path, "r", encoding="utf-8") as file:
        content = file.read().strip()

    return content
