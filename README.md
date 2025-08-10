# 🎯 Red Ball Tracker

**Red Ball Tracker** is a real-time **red ball tracking** project built with ROS and Python.  
The project captures video from a Windows camera using `camera_windows.py`, streams it over ROS topics via `camera_bridge.py`, and performs red ball detection and tracking with `red_ball_tracker.py`.

---

## Features
- Video capture from Windows camera (`camera_windows.py`)  
- ROS nodes implemented in Python  
- Real-time red ball detection using OpenCV  
- Image streaming via ROS topics  

---

## Project Structure
- `camera_windows.py` : Python node capturing video from Windows camera  
- `camera_bridge.py` : Python node publishing camera images to ROS topics  
- `red_ball_tracker.py` : Python node detecting and tracking the red ball  

---

## Installation and Operation

1. Install ROS:  
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full

## Installation and Usage

```bash
# Install ROS (e.g. Noetic)
sudo apt update
sudo apt install ros-noetic-desktop-full

# Install required Python libraries
pip install opencv-python numpy

# Clone the project into your catkin workspace and build
cd ~/catkin_ws/src
git clone https://github.com/yourusername/red_ball_tracker.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash

## Start ROS core
roscore

# Terminal 1: Run camera_windows node
rosrun red_ball_tracker camera_windows.py

# Terminal 2: Run camera_bridge node
rosrun red_ball_tracker camera_bridge.py

# Terminal 3: Run red_ball_tracker node
rosrun red_ball_tracker red_ball_tracker.py

