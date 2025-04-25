# DevOps Automation for POLARIS GEM Simulator

This repository demonstrates the end-to-end DevOps automation of the [POLARIS_GEM_e2 simulator](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2), as part of the Senior DevOps Automation Engineer assignment for SPRINT_GROUND.

## Repository Overview

This project includes:

- A Dockerized environment to run ROS and the simulator
- A Jenkins pipeline to automate build, test, and deployment
- An evaluation script to assess simulation accuracy
- Automated notification system for test results

---

## Setup Instructions

### 1. Clone the POLARIS GEM Simulator

Follow the official setup instructions here:  
[https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2)

Make sure the simulator runs locally before proceeding.

---

## Docker Setup

### Build Docker Image

```bash
docker build -t polaris_gem_sim .
```

### Run Docker Container

```bash
docker run -it --rm --net=host --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  polaris_gem_sim
```

This container includes ROS, Gazebo, all necessary drivers, and dependencies for running the simulator.

---

## Evaluation Script

An evaluation script monitors the **cross-track error** output during simulation. It validates whether the error stays under `1m` throughout the run.

### Run Evaluation

```bash
rosrun evaluation check_cross_track_error.py
```

- The script subscribes to a custom ROS topic publishing the cross-track error.
- Returns `PASS` if the error remains below 1m, otherwise `FAIL`.

---

## Jenkins Pipeline

The Jenkinsfile provided supports:

- Building ROS packages
- Launching the Gazebo simulator
- Running the path-tracking algorithm
- Evaluating success
- Sending result notifications

### Pipeline Stages:

1. **Build**: Compile ROS packages inside Docker
2. **Simulate**: Launch `gem_gazebo` with:
   ```bash
   roslaunch gem_gazebo gem_gazebo_rviz.launch gui:=false use_rviz:=false
   roslaunch gem_gazebo gem_sensor_info.launch
   rosrun gem_pure_pursuit_sim pure_pursuit_sim.py
   ```
3. **Evaluate**: Run the script and collect result
   ```bash
   rosrun gem_pure_pursuit_sim evaluate_cross_track_error.py
   ```
4. **Notify**: Send result via email/Slack

Note: Agents are configured using Kubernetes pod templates via Jenkins Kubernetes plugin.

---

## Notifications

The Jenkins pipeline sends a notification (e.g., email) summarizing:

- Build status
- Simulation outcome
- Evaluation results

---

## Video Demonstration

The accompanying video includes:

1. Launching the stack in Docker
2. Building and logging ROS stack
3. Running simulation and evaluation
4. Final notification display via email

---

## File Structure

```
.
├── Dockerfile                # Docker environment with ROS & dependencies
├── Jenkinsfile               # Jenkins pipeline definition
├── evaluation/
│   └── evaluate_cross_track_error.py  # Success criteria script
|   └── scripts/pure_pursuit_sim.py  # Modifications to simulation to support eval script
├── README.md
```

---

## References

- [POLARIS GEM e2 Simulator](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2)
- [Jenkins Kubernetes Plugin](https://www.jenkins.io/doc/pipeline/steps/kubernetes/)
