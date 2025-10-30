## Introduction

Welcome to the ROSCON FR/DE 2025 "Get Your Hands Dirty" workshop! Thank you for joining us and for your enthusiasm to build, test, and iterate hands-on with ROS 2 and MoveIt 2.

- Date: November 17, 2025
- Location: Strasbourg/Straßburg
- Duration: approximately 4 hours
- Team setup: please form teams of 4 before beginning

Over the next few hours, you and your teammates will explore a ready-to-run robotics stack, integrate motion planning, and iterate on real-world integration tasks. The goal is to learn by doing, share insights, and have fun building together.

Special thanks to the Niryo Core Team : Thomas Deffontaines (Integration Team), Justin Mottier, Thomas Degallaix, Christopher Dedeurwaerder & Pierre Hantson (Software Team)

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

**Setup overview:**
![Workshop setup](assets/factory_cell_schematic.png)

## Software setup

There are two ways to run the workshop environment: using a devcontainer (recommended for consistency and ease of setup) or installing locally (if you already have ROS 2 Jazzy on your machine).

### Using a Devcontainer (Recommended)

**Prerequisites**: 
- Install [Docker](https://docs.docker.com/get-docker/) on your machine
- Install [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

**Option 1: Using the Pre-built Image from Registry (Recommended)**

This is the fastest and easiest option. The image is already built and ready to use.

1. Clone the workshop repository:
   ```bash
   git clone https://github.com/NiryoRobotics/roscon-2025.git
   cd roscon-2025
   ```

2. Open in VS Code:
   ```bash
   code .
   ```

3. Open the devcontainer:
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
   - Select `Dev Containers: Reopen in Container`
   - Wait for the container to start (first time may take a few minutes to download the image)

4. The workspace will open at `/root/niryo_workshop` with all dependencies pre-installed!

**Option 2: Building the Image Locally from Dockerfile**

If you prefer to build the image yourself or need to customize the Dockerfile:

1. Clone the workshop repository:
   ```bash
   git clone https://github.com/NiryoRobotics/roscon-2025.git
   cd roscon-2025
   ```

2. Edit `.devcontainer/devcontainer.json`:
   - **Comment out** the `"image"` line:
     ```jsonc
     // "image": "ghcr.io/niryorobotics/roscon-2025-workshop:latest",
     ```
   - **Uncomment** the `"build"` section:
     ```jsonc
     "build": {
         "dockerfile": "Dockerfile",
         "context": "..",
         "args": {
             "WORKSPACE_NAME": "niryo_workshop"
         }
     },
     ```

3. Open in VS Code and start the devcontainer:
   ```bash
   code .
   ```
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
   - Select `Dev Containers: Reopen in Container`
   - Wait for the build to complete (first build may take 10-15 minutes)

4. The workspace will open at `/root/niryo_workshop` with all dependencies built locally!

**Troubleshooting Devcontainer Issues**:
- **GUI/Display issues**: Make sure Docker has access to your X11 display (Linux) or use XQuartz (Mac) / VcXsrv (Windows)
- **Network issues**: Ensure Docker has network access (`--net=host` is configured in the devcontainer)
- **Rebuild container**: If something goes wrong, use `Dev Containers: Rebuild Container` from the command palette

### Local Installation (Without Docker) 

Create a new workspace and clone the Ned ROS 2 driver in the source folder:

```bash
mkdir -p ~/niryo_workshop/
cd ~/niryo_workshop/
git clone https://github.com/NiryoRobotics/ned-ros2-driver.git
```

Clone the ROSCON workshop

```bash
git clone https://github.com/NiryoRobotics/roscon-2025.git
```

Install ROS 2 dependencies with rosdep:

```bash
cd ~/niryo_workshop
rosdep install --from-paths src --ignore-src -r -y
```

Create a Python virtual environment and install Python dependencies:

```bash
cd ~/niryo_workshop
python3 -m venv venv --system-site-packages
source venv/bin/activate
pip install -r src/ned-ros2-driver/requirements.txt
```

Build the ROS 2 workspace:

```bash
cd ~/niryo_workshop
source /opt/ros/jazzy/setup.bash
colcon build
```

Add environment setup to your `~/.bashrc` for automatic sourcing in new terminals:

```bash
# Add these lines to your ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/niryo_workshop/venv/bin/activate" >> ~/.bashrc
echo "source ~/niryo_workshop/install/setup.bash" >> ~/.bashrc
echo "cd ~/niryo_workshop" >> ~/.bashrc
```

**Important - ROS Domain Configuration**: 
During the workshop, multiple robot cells will be operating on the same network. To communicate with your assigned hardware (robot, Raspberry Pi, etc.) while staying isolated from other teams, you need to configure your ROS Domain ID.

**Each hardware cell has a specific ROS_DOMAIN_ID** that will be indicated on the robot or Raspberry Pi at your workstation.

Edit your `~/.bashrc` file (located at `~/.bashrc` locally or `/root/.bashrc` in the devcontainer):

```bash
# Change LOCALHOST to SUBNET to discover devices on the local network
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Set the ROS_DOMAIN_ID provided for your hardware/cell
export ROS_DOMAIN_ID=XX  # Replace XX with the domain ID written on your robot/Raspberry Pi
```

**Configuration steps**:
1. **Find your domain ID**: Check the label on your robot or Raspberry Pi for the assigned `ROS_DOMAIN_ID`
2. **Change discovery range**: Set `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` in your `~/.bashrc`
3. **Set your domain ID**: Add `export ROS_DOMAIN_ID=XX` (replace XX with your assigned number)
4. **Apply changes**: Run `source ~/.bashrc` or open a new terminal

**Why this works**: 
- `SUBNET` allows discovery of ROS 2 nodes across the local network (needed to find robots and Raspberry Pi devices)
- `ROS_DOMAIN_ID` provides isolation by ensuring you only communicate with devices in your assigned domain
- This allows multiple teams to work simultaneously without interference

**Note**: If you switch between different parts of the workshop (quality check vs packaging) or move to a different robot cell, remember to update the `ROS_DOMAIN_ID` and source your bashrc again!

### Final Setup Steps (Both Devcontainer and Local)

Now that your environment is ready, you need to configure the robot connection and initialize the hardware.

**1. Configure the ROS 2 Driver**

Edit the driver configuration file to add your robot's IP address:

```bash
# Path to the configuration file
~/niryo_workshop/src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml
```

Open this file and add the IP address of your robot's Ethernet interface (check the label on your robot or table).

**Example configuration:**

```yaml
rosbridge_port: 9090
robot_namespaces:
  - ""
robot_ips:
  - "192.168.1.100"  # Replace with your robot's IP address (found on the robot label)
```

**Note**: 
- The robot namespace can remain empty (`""`) for single robot setups
- Make sure to save the file after editing

**2. Initialize the Conveyor Belt**

First, launch the ROS 2 driver to enable communication with the robot:

```bash
ros2 launch niryo_ned_ros2_driver driver.launch.py
```

Then, in a **new terminal**, initialize the conveyor belt by calling this service (only needed once per robot):

```bash
ros2 service call /niryo_robot/conveyor/ping_and_set_conveyor niryo_ned_ros2_interfaces/srv/SetConveyor "{cmd: 1, id: 9}"
```

**Note**: This initialization only needs to be done once per robot, not every time you run your program.

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
- Lied on his resume to get the internship, especially about his experience with Moveit 2 (he just know the name of the framework) and his level in german (he just know the word "Hallo", but they pay more there).

He will be in charge of the Packaging line integration, as he has "already" worked with Moveit 2. And as a frenchman, he will be able to communicate with the Niryo integration team if he needs help with the trajectories and the hardware.

Gameplay : Paul-Louis implemented a lazy solution that is not very sophisticated. He just did the bare minimum to pass the internship. Nevertheless, the feedback will be performed by the Niryo Team, it will thus be more guided than a professor would, and it is also more industry oriented. This character is perfect for persons who want to discover how ROS2 tools can be used to perform a real industrial integration, and know more about trajectories and industrial planning.

Click [here](workshop/workshop_packaging_manager/pl_chardonnay_report.md) to read his report.

### Hans-Günther Biermann



Education : Bachelor's degree in Computer Science at the University of Stuttgart

Skills : Comfortable with Python and programming in general, took an optional course in computer vision applied to robotics so he can handle basic ROS 2 tasks.

Personality :
- Drinks a beer for breakfast at 6.30am every day to enhance his concentration and work all day long without stopping.
- Always thinks about performance and efficiency, overthinking is the best quality to be a good engineer in his opinion.

He will be in charge of the Quality Check line integration, as he has strong background in computer vision.

Gameplay : Hans-Günther implemented a solution that is over-engineered for the task, leading to an unefficient solution with a complex architecture. The feedback will be performed by a university professor that does not have engough time to provide a guided feedback leading to a perfect solution. This character is perfect for persons who want to learn how to design and lead a complex integration project, based on ROS2 and that uses principles of good Python programming, such as encapsulation, multi-threading... It also talks about AI integration in an industrial context.

Click [here](workshop/workshop_quality_check_manager/hg_biermann_report.md) to read his report.

## Next Step

For the following parts of the workshop, we will divide your team in two groups : Those who will read and follow the work of Paul-Louis, and the others who will read and follow the work of Hans-Günther. 

To start the reproducing the Naive solution, please start from the packages located inside the `workshop` folder.

Each markdown file contains the instructions to reproduce the Naive solution on Hans-Günther's or Paul-Louis's line.
