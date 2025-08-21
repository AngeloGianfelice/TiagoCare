# TiagoCare: a semantic-aware robotic healthcare assistant
## 👥 Contributors

| Name          | Email                  |
|---------------|------------------------|
| Angelo Gianfelice  | gianfelice.1851260@studenti.uniroma1.it |
| Stefano Previti    | previti.2151985@studenti.uniroma1.it |

## Introduction
This project focuses on enhancing patient experience in hospitals using the Tiago robotic assistant. Tiago provides **emotional** and **psychological** support, addressing patient needs in scenarios where human assistance may be limited, such as crowded hospitals or staff shortages. The application aims to deliver **human-like** assistance, improving patient well-being through contextual understanding and interaction.

## Objectives
Tiago integrates a **Vision-Language Model (VLM)** and a **Large Language Model (LLM)** to support patients in negative psychological states. By applying ***Cognitive Behavioral Therapy (CBT)*** techniques, Tiago encourages patients to reflect and reason through their feelings using various prompts (e.g., advice, past experiences). Additionally, Tiago can **infer** knowledge from the environment to provide urgent assistance when required.

## Summary of Results
The project demonstrates Tiago as **both** a functional and emotional support tool in medical settings. Trials across various experimental scenarios evaluated performance using multiple metrics, covering psychological aspects (emotion detection, engagement) and functional aspects (resource efficiency, safety). The best architecture achieved an overall score of 5/5, confirming Tiago’s effectiveness in ***enhancing patient care***.

For a more detailed description of out project, please refer to the ***tiagocare.pdf*** file.

## Installation
Make sure to have docker installed on you system, since the project will run only inside the [https://gitlab.com/brienza1/empower_docker](https://gitlab.com/brienza1/empower_docker) docker image. This contains all the required dependencies to run simulation on the ***Tiago*** platform. You can simply follow this steps:
### Step 0: Pull Docker Image
```bash
docker pull registry.gitlab.com/brienza1/empower_docker:latest
```

### Step 1: Download Utility Script

Download the script to run the Docker container from [https://drive.google.com/file/d/1u0zmkBnbIykiW3yd0eru2ORXmSPhGDY2/view?usp=sharing](https://drive.google.com/file/d/1u0zmkBnbIykiW3yd0eru2ORXmSPhGDY2/view?usp=sharing)

### Step 2: Run Docker

Run the container with the following command:

```bash
./start_docker.sh -it -v /dev/snd:/dev/snd -v <local_folder>:<container_folder> <docker_image>
```
  **<local_folder>** → folder on your PC to mount.

  **<container_folder>** → folder inside the container (can be in a ROS workspace).

  **<docker_image>** → Docker image to start.

  Optional: You can mount a folder inside the container for easy access to files.

### Step 3: Attach to Running Container

For each terminal you want to use with the running container:
```bash
xhost +
docker exec -it <container_id> /bin/bash
```

**<container_id>** → ID of the running container.

### Step 4: Setup ROS Workspace

Inside the container, source the workspace:
```bash
source /tiago_public_ws/devel/setup.bash
```
### Step5: Cloning our repo and installing requirements
To use our code inside the docker you must firtly pull this repo inside it:
```bash
git clone https://github.com/AngeloGianfelice/TiagoCare
```
Then, navigate to the repo's root folder and install all the required python libraries using the following command:
```bash
pip install -r requirements.txt
```
## Usage
### Environment setup
Firstly you must set your OpenAI api key by running the command:
```bash
export OPENAI_API_KEY="<your_key>"
```
Before running our code you must also copy the ***custom_hospital.world*** file inside the ***/tiago_public_ws/src/pal_gazebo_worlds/worlds/***
This will load the custom world we developed for the project.
### Running the code
To run the code you has to simply execute the ***tiagocare.py*** script, which can take up to five parameters, depending on the mode chosen.
The script can be run in **two modes**:

1. **interaction**: Runs the TiagoCare interaction mode.
2. **ska**: Runs the Semantic Knowledge Agent pipeline.

The script requires command-line arguments to specify the mode and related parameters.

---

```bash
python your_script.py --mode <mode> [other arguments]
```
Arguments
| Argument      | Type | Default   | Description                                   |
|---------------|------|-----------|-----------------------------------------------|
| `--mode`      | str  | required  | Mode to run: `interaction` or `ska`          |
| `--log_dir`   | str  | new_task  | Logging directory for SKA mode                |
| `--task`      | str  | task1     | Task for interaction mode (`task1`, `task2`, `task3`) |
| `--reasoning` | str  | true      | Use reasoning (`true` or `false`)            |
| `--model`     | str  | gemini    | Model to use (`gemini`, `deepseek`, `llama`) |

**1. Running in interaction mode**
   
   To spawn Tiago in the default position inside the gazebo environment use the command:
   
  ```bash
  roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-hey5 world:=custom_hospital
  ```

  If you want Tiago to spawn in our task-specific coordinates simply add to the roslaunch command the ***gzpose*** parameter as follows:
  - **task1**: gzpose:="-x 2.09 -y 3.68 -z 0.0 -R 0.0 -P 0.0 -Y 0.0"
  - **task2**: gzpose:="-x 4.29 -y 9.29 -z 0.00 -R 0.0 -P 0.0 -Y -1.74"
  - **task3**: gzpose:="-x -1.39 -y 8.63 -z -0.001 -R 0.0 -P 0.0 -Y 2.98"

  In another terminal run the tiagocare script similarly to the example below
  ```bash
  python tiagocare.py --mode interaction --task task2 --reasoning true --model llama
  ```
  This will start the interaction with the patient of the chosen task, using the desired model either in ***vanilla*** or in ***reasoning*** mode.
  
***2. Running in ska mode***

  Fistly start Ros by running the ***roscore*** command.
  To run the semantic knowledge agent pipeline you simply have to record Tiago's RGB camera sensor input.
  In order to achieve this, in another terminal, you must use the command:
  ```bash
    rosbag record -O <your_bag_name>.bag --duration=15 /xtion/rgb/image_rect_color
  ```
  Once you have you ros bag of your task you can run the tiagocare script in the ska mode, also passing as input the name of the directory in the output folder where you want your ska task output to be.
  ```bash
  python tiagocare.py --mode ska --log_dir session1
  ```
 This will wait for you to replay the rosbag you just recorded. You can do that in another terminal by simply running the command:
  ```bash
  rosbag play your_bag_name.bag
  ```
 The output directory you specified will contain both the rgb image of Tiago's field of view and the computed scene's knowledge graph.

