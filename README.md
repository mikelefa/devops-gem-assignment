# POLARIS GEM DevOps Assignment

## 1. Prerequisites

- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo (compatible with ROS Noetic)
- Docker
- Jenkins (with Kubernetes plugin)

## 2. Clone the POLARIS GEM Simulator

```bash
# Create and initialize a catkin workspace
mkdir -p ros_ws/src
cd ros_ws
catkin_make

# Clone the simulator into src/
cd src
git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Build the simulator
cd ..
catkin_make
source devel/setup.bash

# Test: launch a basic simulation to verify it runs locally
roslaunch polaris_gem_ros gem.launch
```

> **Note**: Adjust your `ROS_PACKAGE_PATH` or workspace location as needed.

## 3. Docker Setup

### 3.1 Build Docker Image

From the project root directory, run:

```bash
docker build -t polaris_gem_sim -f docker/Dockerfile .
```

### 3.2 Run Docker Container

```bash
docker run -it --rm --net=host --privileged   -v /tmp/.X11-unix:/tmp/.X11-unix   -e DISPLAY=$DISPLAY   polaris_gem_sim
```

This forwards the X11 display for RViz and Gazebo GUIs.

## 4. Repository Structure

```
.
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── jenkins/
│   ├── Jenkinsfile
│   └── pod-template.yaml
├── ros_ws/
│   ├── .catkin_workspace
│   └── src/
│       └── POLARIS_GEM_e2/
├── scripts/
│   ├── pure_pursuit_sim.py
│   └── evaluate_cross_track_error.py
├── demo/
│   └── Gem_Sim_automation_run.mp4
└── README.md
```

## 5. Running the Pure Pursuit Simulation (Local)

Inside the Docker container (or your sourced workspace):

```bash
# Launch simulation without GUI for automation
roslaunch gem_gazebo gem_gazebo_rviz.launch gui:=false use_rviz:=false

# Launch sensor info
roslaunch gem_gazebo gem_sensor_info.launch

# Run the pure pursuit algorithm
rosrun gem_pure_pursuit_sim pure_pursuit_sim.py
```

## 6. Evaluation Script

After simulation, evaluate the cross-track error:

```bash
# Package: gem_pure_pursuit_sim
rosrun gem_pure_pursuit_sim evaluate_cross_track_error.py
```

This script outputs the average and maximum cross-track errors for your path following.

## 7. Jenkins Pipeline

The `jenkins/Jenkinsfile` defines the CI/CD pipeline with these stages:

1. **Build**  
   Compile all ROS packages inside the Docker image.
2. **Simulate**  
   - `roslaunch gem_gazebo gem_gazebo_rviz.launch gui:=false use_rviz:=false`  
   - `roslaunch gem_gazebo gem_sensor_info.launch`  
   - `rosrun gem_pure_pursuit_sim pure_pursuit_sim.py`
3. **Evaluate**  
   - `rosrun gem_pure_pursuit_sim evaluate_cross_track_error.py`
4. **Notify**  
   Send pass/fail results via email or Slack.

> **Note**: Jenkins agents use the `jenkins/pod-template.yaml` for Kubernetes-based builds. Ensure your Jenkins has the Kubernetes plugin configured and proper credentials for cluster access.

## 8. Demo Video

A sample automation run is available at:

```
demo/Gem_Sim_automation_run.mp4
```

---

_End of document._
