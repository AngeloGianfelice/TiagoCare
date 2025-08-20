from config import * 
from utils import *
from scene_acquisition import *
from source.agents import *

#SEMANTIC KNOWLEDGE AGENT MODULE PIPELINE 
class semantic_knowledge_agent():
    def __init__(self):
        self.prompt_file= PROMPT_DIR + "visual_agent_prompt.txt" 

    def run(self):
        #Generate task folder in output directory
        log_dir = create_task_folder()

        # Acquire image of the scene
        image = acquire_image(log_dir) #local_acquire_image(log_dir) to give precomputed rgb image
        
        print("Image acquired")
        
        visual_agent_prompt = open(self.prompt_file).read()   

        # Run SMA agent
        semantic_knowledge_agent(image,visual_agent_prompt,log_dir)
        return

        