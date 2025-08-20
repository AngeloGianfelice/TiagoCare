from config import * 
from source.utils import *
from source.scene_acquisition import *
from source.agents import *

#SEMANTIC KNOWLEDGE AGENT MODULE PIPELINE 
class SKA_pipeline():
    def __init__(self,log_dir):
        self.prompt_file= PROMPT_DIR + "visual_agent_prompt.txt" 
        self.log_dir=log_dir

    def run(self):
        #Generate task folder in output directory 
        task_folder = create_task_folder(self.log_dir)
	
        # Acquire image of the scene
        image = acquire_image(task_folder) #local_acquire_image(task_folder) to give precomputed rgb image
        
        print("Image acquired")
        
        visual_agent_prompt = open(self.prompt_file).read()   

        # Run SMA agent
        semantic_knowledge_agent(image,visual_agent_prompt,task_folder)
        return
       

        
