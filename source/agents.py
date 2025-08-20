from openai import OpenAI
from config import *
import base64
import cv2
import os

client = OpenAI(base_url=API_url)
api_key = os.getenv("API_KEY")

client = OpenAI(
    base_url=API_url,
    api_key=api_key
)
    
def llm_call(chat_history, my_model):
    if my_model == 'gemini':
        model = "google/gemini-2.0-flash-exp:free"
    elif my_model == 'llama':
        model = "meta-llama/llama-3.1-405b-instruct:free"
    elif my_model == 'deepseek':
        model = "tngtech/deepseek-r1t2-chimera:free"

    completion = client.chat.completions.create(
        model=model,
        messages=chat_history
    )
    return completion.choices[0].message.content

def vlm_call(prompt, encoded_image):
    agent = client.chat.completions.create(
    model=VLM,
    messages=[
        {
        "role": "user",
        "content": [
            {
            "type": "text",
                "text":f"{prompt}"},
            {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{encoded_image}",
            },
            },
        ],
        }
    ],
    temperature=0.1,
    )
    response = (agent.choices[0].message.content)
    return response

def image_to_buffer(image):
    if os.path.isfile(image):
        with open(image, "rb") as f:
            encoded_image =  base64.b64encode(f.read()).decode('utf-8')
    else:
        _, buffer = cv2.imencode('.jpg', image)
        encoded_image = base64.b64encode(buffer).decode("utf-8")
    return encoded_image

def semantic_knowledge_agent(image,visual_agent_prompt,log_dir):

    encoded_image = image_to_buffer(image)
    vlm_answer = vlm_call(visual_agent_prompt, encoded_image)

    print(vlm_answer)
    # Save vlm_answer to file
    with open(log_dir + "vlm_answer.txt", "w") as file:
        file.write(vlm_answer)
    return 
    

