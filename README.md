# 🤖 Real-World Trajectory Optimization & Tracking

Real-world deployment of trajectory optimization and closed-loop trajectory tracking on an **AgileX LIMO mobile robot** using **ROS1, OptiTrack, Python, and PID control**.

This project investigates the transition from simulation-based trajectory optimization to physical robot deployment, with a focus on **trajectory tracking, real-time feedback, and sim-to-real performance**.

---

## 🛠️ Technologies

`Python` `ROS1` `ROS Noetic` `PID Control` `OptiTrack` `Trajectory Optimization` `NumPy` `Matplotlib` `Ubuntu`

---

## 🎯 Project Overview

The system generates optimized reference trajectories and executes them on a physical **AgileX LIMO** mobile robot.

Real-time pose measurements from an **OptiTrack motion-capture system** provide feedback to a dual-PID controller responsible for controlling the robot's motion along the reference trajectory.

The project was used to investigate differences between theoretically optimized trajectories and real-world execution, including the effects of model mismatch, physical constraints, and system latency.

### Control Pipeline

```text
Trajectory Optimization
        ↓
Reference Trajectory
        ↓
Dual PID Controller
        ↓
ROS Velocity Commands
        ↓
AgileX LIMO
        ↓
Physical Motion
        ↓
OptiTrack Pose Feedback
        └──────────────→ Tracking Error → PID Controller
```

---

## 📊 Experimental Results

### Optimized Trajectory & Robot Path

The reference trajectory and robot motion can be visualized in the experiment world view.

![Trial 2 World Trajectory](assets/images/trial2_world.png)

### Trajectory Tracking Performance

Tracking data from the physical experiment was analyzed to compare the desired trajectory against the robot's actual response.

![Trial 2 Tracking Graphs](assets/images/trial2_graphs.png)

The experiments achieved approximately **92% theoretical optimal-path efficiency**, while analysis identified approximately **8% sim-to-real trajectory deviation**.

A major source of tracking error was the difference between the idealized trajectory model used during optimization and the physical car-like dynamics of the LIMO platform.

---

## 🎥 Physical Robot Demonstration

### Trajectory Tracking Experiment

This experiment shows the AgileX LIMO executing the optimized trajectory using real-time feedback control.

https://github.com/user-attachments/assets/REPLACE_WITH_VIDEO_LINK

> `assets/videos/trial2_run.mp4`

### Real-Time Robot Position Tracking

The following demonstration shows the robot's position being updated during physical execution using OptiTrack feedback.

https://github.com/user-attachments/assets/REPLACE_WITH_VIDEO_LINK

> `assets/videos/live_robot_position.mp4`

---

## 🖥️ Simulation

The trajectory optimization and tracking algorithms were first evaluated in simulation before deployment onto the physical robot.

![Simulation](assets/images/simulation_zoomed.png)

Simulation provided a controlled environment for evaluating trajectories and controller behavior before conducting physical experiments.

---

## 🔬 Sim-to-Real Analysis

Physical deployment revealed several differences that were not fully represented by the simulation model.

Key sources of deviation included:

- Robot kinematic and dynamic constraints
- Model mismatch between the trajectory optimizer and physical LIMO
- Steering limitations during sharp turns
- Higher tracking error during high-speed trajectory segments
- Communication and control-loop latency

These results demonstrate the importance of incorporating realistic vehicle dynamics when transferring trajectory optimization algorithms from simulation to physical robotic systems.

---

## 🧠 Key Takeaways

This project provided hands-on experience with:

- Trajectory optimization
- Closed-loop trajectory tracking
- PID controller development and tuning
- ROS-based robot communication
- OptiTrack motion capture
- Real-time pose feedback
- Experimental robotics
- Sim-to-real validation
- Robot data analysis and visualization

---

## 🚀 Future Work

Future improvements include:

- Developing a trajectory model that more accurately represents the LIMO's car-like dynamics
- Exploring more advanced trajectory-tracking controllers
- Improving tracking through sharp turns and high-speed segments
- Further analyzing sim-to-real performance
- Evaluating additional optimized trajectories and operating conditions

---

## ⚙️ Setup & Running the Project

For installation, ROS configuration, dependencies, and instructions for running the project, see:

**[Setup Guide](SETUP.md)**

---

## 📁 Repository Structure

```text
realworld_LIMO_hytoperm/
├── README.md
├── SETUP.md
├── assets/
│   ├── images/
│   │   ├── trial2_world.png
│   │   ├── trial2_graphs.png
│   │   └── simulation_zoomed.png
│   └── videos/
│       ├── trial2_run.mp4
│       └── live_robot_position.mp4
├── hytoperm_implinetation/
├── limo_tutorial/
└── vrpn_client_ros/
```

---

## 👤 Author

**Justen Li**  
Robotics MSE — Johns Hopkins University  
B.S. Mechanical Engineering, Robotics Concentration — Boston University
