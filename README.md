# Final Project Internship RoboCon ITS Team

## Operating System: Ubuntu Jellyfish

### 1. Create Workspace
First, create a new workspace directory for your ROS2 project

```bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
```

### 2. Clone Repository
Clone the repository from GitHub to your workspace
```bash
git clone https://github.com/kveeyy/fp-internship-robocon.git finalproject
```

### 3. Move into your workspace directory (~/ros2_ws)
```bash
cd ~/ros2_ws
```

### 4. Install Dependencies
Initialize rosdep and install the necessary dependencies for the project
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Project
Now, build the project using colcon:
```bash
colcon build
```

## 6. Source the Setup Files
Before launching, source the necessary setup files
```bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
```

## 7. Launch the Simulation
Finally, launch the simulation using the following command:
```bash
ros2 launch simulasi-2025 display.launch.py
```
