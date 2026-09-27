# realworld_LIMO_hytoperm
Applying Jonas Hytoperm sim code to real-world physical LIMO robot
Go to "master" branch

# 🤖 Real-World Trajectory Optimization & Tracking

Real-world deployment of trajectory optimization and tracking algorithms on an
**AgileX LIMO mobile robot** using **Python, ROS1, OptiTrack, and PID control**.

The project extends simulation-based trajectory optimization to a physical robotic
platform, investigating the challenges of real-time trajectory execution,
controller performance, and sim-to-real model mismatch.

## 🛠️ Technologies

- Python
- ROS1 / ROS Noetic
- OptiTrack Motion Capture
- PID Control
- Trajectory Optimization
- NumPy
- Matplotlib
- Linux / Ubuntu

## 🚀 Key Features

- Executes optimized trajectories on a physical AgileX LIMO robot
- Uses real-time OptiTrack pose feedback for closed-loop control
- Implements dual PID control for trajectory tracking
- Compares planned trajectories against real-world robot motion
- Analyzes sim-to-real trajectory deviation and model mismatch

## ⚙️ System Overview

The trajectory optimizer generates a time-dependent reference trajectory for the
robot to follow.

During physical execution, OptiTrack provides real-time global pose measurements.
The tracking controller compares the robot's current state with the reference
trajectory and generates velocity and steering commands for the LIMO through ROS.

The overall pipeline is:

**Trajectory Optimization → Reference Trajectory → PID Tracking → ROS Commands → LIMO**

**LIMO → OptiTrack Pose Feedback → Tracking Error → PID Controller**

## 🎯 Control & Trajectory Tracking

A dual-PID control architecture is used to track the optimized trajectory.

The controller continuously evaluates the difference between the desired trajectory
and the measured robot pose, using this feedback to update the robot's motion
commands.

This allowed the optimized trajectories developed in simulation to be evaluated
on a physical robotic platform.

## 📊 Results

- Achieved **92% of theoretical optimal path efficiency** during physical trajectory tracking
- Identified approximately **8% sim-to-real trajectory deviation**
- Observed increased tracking error during sharp turns and higher-speed trajectory segments
- Identified model mismatch between the trajectory model and the LIMO's physical motion constraints as a major source of error

These experiments highlight the importance of incorporating realistic vehicle
dynamics when transferring trajectory optimization algorithms from simulation
to physical robots.

## 🔬 What I Learned

This project provided hands-on experience with:

- Real-time robotic control
- PID controller development and tuning
- Trajectory optimization and tracking
- ROS-based robot communication
- Motion-capture feedback
- Experimental robotics
- Sim-to-real validation
- Debugging physical autonomous systems

One of the biggest lessons was that a trajectory that performs well in simulation
does not necessarily transfer directly to a physical robot. Vehicle dynamics,
latency, actuator limitations, and modeling assumptions all influence real-world
tracking performance.

## 💻 Running the Project

### Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3
- AgileX LIMO
- OptiTrack motion-capture system

### Setup

Clone the repository:

    git clone <repository-url>

Additional setup instructions for the LIMO, ROS environment, and OptiTrack
configuration will be documented here.

## 🎥 Demo

<!-- Add GIF/video of LIMO following an optimized trajectory here -->

Coming soon.

## 🔮 Future Work

- Improve the vehicle model to better represent LIMO dynamics
- Implement more advanced trajectory-tracking controllers
- Improve performance through sharp turns and high-speed segments
- Further investigate sim-to-real differences
- Evaluate additional trajectory optimization and control methods
