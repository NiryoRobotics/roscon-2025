## Introduction

Welcome to the ROSCON FR/DE 2025 "Get Your Hands Dirty" workshop! Thank you for joining us and for your enthusiasm to build, test, and iterate hands-on with ROS 2 and MoveIt 2.

- Date: November 18, 2025
- Location: Strasbourg/Straßburg
- Duration: approximately 4 hours
- Team setup: please form teams of 4 before beginning

Over the next few hours, you and your teammates will explore a ready-to-run robotics stack, integrate motion planning, and iterate on real-world integration tasks. The goal is to learn by doing, share insights, and have fun building together.

Special thanks to the Niryo Core Team : Thomas Deffontaines & Romain Krikorian (Integration Team), Justin Mottier, Thomas Degallaix & Pierre Hantson (Software Team)

Let’s get started!

## Hardware setup

Please ensure the following before we begin:

- Participant equipment (you bring):
  - At least two different computers (laptops) for teamwork and multi-machine ROS 2 networking

- Provided by organizers (we bring):
  - 2x Niryo Ned3 Pro robots
  - 2x conveyor belts with IR sensors
  - 1x Raspberry Pi 4 connected to a screen via HDMI
  - 1x Ethernet switch and 3x Ethernet cables

Connectivity checklist:
- Verify each conveyor belt is connected to the Ned3 Pro back panel (tool connector) and that its IR sensor is connected to the conveyor electronics as specified by Niryo.
- Network 1 (QC line): Raspberry Pi + QC robot + QC conveyor + Computer 1 are connected together via the Ethernet switch.
- Network 2 (Packaging line): Packaging robot + Computer 2 are directly connected via Ethernet (point-to-point).
- Make sure both computers are NOT connected to the same Wi‑Fi network at the same time to avoid cross-talk (ROS 2 is multi-machine by default).
- Alternatively, you can set distinct ROS 2 domains to prevent interference, for instance:
  - Network 1: ROS_DOMAIN_ID=11
  - Network 2: ROS_DOMAIN_ID=22

Network layout overview:

![Workshop network layout](src/assets/network_layout.svg)

## Software setup

There are two ways to run the workshop environment: locally (if you already have ROS 2 Jazzy on your machine) or using our Docker image (recommended for consistency).

### Locally

In a new workspace, clone the ROS 2 driver on your machine:

```bash
mkdir -p ~/ros2_drivers_ws/src
cd ~/ros2_drivers_ws/src
git clone https://github.com/NiryoRobotics/ned-ros2-driver.git
```

Install ROS 2 dependencies with rosdep:

```bash
cd ~/ros2_drivers_ws
rosdep install --from-paths src --ignore-src -r -y
```

Create a Python virtual environment and install Python dependencies:

```bash
cd ~/ros2_drivers_ws
python3 -m venv mysupervenvname --system-site-packages
source mysupervenvname/bin/activate
pip install -r src/ned-ros2-driver/requirements.txt
```

Create another workspace for the ROSCON workshop:

```bash
mkdir -p ~/roscon_ws
cd ~/roscon_ws
git clone https://github.com/NiryoRobotics/roscon-2025.git
```

### With Docker (recommended)

Run the provided image with Docker, or build a new one using the included Dockerfile.

- Ensure Docker has access to your host network (for ROS 2 discovery across devices).
- Ensure GUI access if you plan to run graphical tools (e.g., export X11 or use Wayland/XQuartz depending on your OS).
- If needed, ask GPT for a tailored `docker run` command for your operating system.

Example considerations (not a one-size-fits-all command):
- `--network host` (Linux) or proper port/bridge config on other OSes
- X11 forwarding: `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`
- Mount your workspaces: `-v ~/ros2_drivers_ws:/root/ros2_drivers_ws -v ~/roscon_ws:/root/roscon_ws`

### Raspberry Pi

The Raspberry Pi runs Ubuntu 24 with everything pre-installed. The password is `robotics`. You do not need to perform the software setup steps on the Raspberry Pi.

### Common steps (Local and Docker)

Set up the ROS 2 driver configuration:

```bash
nano ~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml
```

Never forget to source `ros2_drivers_ws` and `roscon_ws` before running the code.

- Do not add a namespace.
- Add your robot IP here (QC robot for Computer 1, or Packaging robot for Computer 2).
- Find your robot IP by connecting via SSH:

```bash
ssh niryo@ned3pro-[robot_ssid].local
ip a
```

Copy the IP of the Ethernet interface, then save `drivers_list.yaml`.

Build both workspaces and source their setups:

```bash
source /opt/ros/jazzy/setup.bash 
cd ~/ros2_drivers_ws
colcon build
source install/setup.bash

cd ~/roscon_ws
colcon build
source install/setup.bash
```

Tip: add the `source` lines to your `~/.bashrc` so you don’t have to source them in every new terminal.

Run the ros2 driver to enable communication with the robots : 
```bash
ros2 launch niryo_ned_ros2_driver driver.launch.py
```

## Context

A pharmaceutical company based in Basel recruited two interns to perform a robotics integration with Niryo Ned3 Pro robots.  
The goal of this integration is to mimic the behavior of an entire factory that sorts chemical products in vials between safe and unsafe categories (classified according to their color) and packs them into boxes to be shipped to pharmacies all over the world. 

In this workshop, you will read their internship report. They will explain how they performed their solution.  
Your first goal will be to follow their path to recreate this solution. We will call it the **"Naive Solution"**, as it works but in an imperfect way.  

Your second goal will be to read the professor's comments on their report. They suggested ways to enhance the performance of the solution in order to meet the requirements expected by the company.  

Finally, as a bonus, feel free to apply the professor's feedback not only to the corresponding part of the solution but also to other parts, and add your own improvements to exceed the company's requirements.

Here are the profiles of both students : 

### Paul-Louis Chardonnay



Education : Final year of Master's degree in Mechatronics Engineering at the University of Lille

Skills : Experience with ROS 2 and MoveIt 2, understands the basics of robotics hardware and automation.

Personality :
- Adept of the "good enough" principle, so that he can meet on time with his friends for the "apéro" at 6pm to the local bar (the only time he is not late)
- Lied on his resume to get the internship, especially about his experience with Moveit 2 (he just know the name of the package) and his level in german (he just know the word "Hallo", but they pay more there).

He will be in charge of the Packaging line integration, as he has "already" worked with Moveit 2. And as a frenchman, he will be able to communicate with the Niryo integration team if he needs help with the trajectories and the hardware.

Gameplay : Paul-Louis implemented a lazy solution that is not very deep. He just did the bare minimum to pass the internship. Nevertheless, the feedback will be performed by the Niryo Team, it will thus be more guided than a professor would, and it is also more industry oriented. This character is perfect for persons who want to discover how ROS2 tools can be used to perform a real industrial integration, and know more about trajectories and industrial planning.

### Hans-Günther Biermann



Education : Bachelor's degree in Computer Science at the University of Stuttgart

Skills : Comfortable with Python and programming in general, took an optional course in computer vision applied to robotics so he can handle basic ROS 2 tasks.

Personality :
- Drinks a beer for breakfast at 6.30am every day to enhance his concentration and work all day long without stopping.
- Always thinks about performance and efficiency, overthinking is the best quality to be a good engineer in his opinion.

He will be in charge of the Quality Check line integration, as he has strong background in computer vision.

Gameplay : Hans-Günther implemented a deep solution that is over-engineered for the task, leading to a unefficient solution with a complex architecture. The feedback will be performed by a university professor that does not have engough time to provide a guided feedback leading to a perfect solution. This character is perfect for persons who want to learn how to design and lead a complex integration project, based on ROS2 and that uses principles of good Python programming, such as encapsulation, multi-threading... It also talks about AI integration in an industrial context.

For the following parts of the workshop, we will devide your team in two groups : Those who will read and follow the work of Paul-Louis, and the others who will read and follow the work of Hans-Günther. 

To start the reproducing the Naive solution, please start from the packages located inside the `src/workshop` folder.

Each markdown file contains the instructions to reproduce the Naive solution on Hans-Günther's or Paul-Louis's line.
