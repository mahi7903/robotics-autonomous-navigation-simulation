# Autonomous Robotics Navigation Simulation

Sensor-driven autonomous robot navigation system built in Python using CoppeliaSim (V-REP).  
This project demonstrates line tracking, maze navigation, border detection, and autonomous target-oriented control using structured simulation architecture.

---

## Project Overview

This system simulates an autonomous mobile robot capable of:

- Line tracking using sensor feedback
- Border detection and directional correction
- Maze navigation logic
- Sensor-based reactive movement
- Target-oriented autonomous navigation
- Shortest-path scene experimentation

The project integrates control logic, sensor processing, and simulation API interaction within a modular structure.

---

## Architecture

Project structure follows a clean engineering layout:
robotics-autonomous-navigation-simulation/
├── src/ # Core robot control logic
├── notebooks/ # Development & experimentation notebooks 
├── scenes/ # CoppeliaSim scene files (.ttt)
├── assests/ # Supporting images
├── demo/ # Short demonstration video
└── .gitignore

The design separates simulation setup, control logic, and navigation strategies to maintain clarity and scalability.
---

## Core Engineering Concepts Applied

- Sensor-driven feedback loops
- Reactive control systems
- PID-inspired correction logic
- Modular robot control abstraction
- Simulation-to-code API integration
- Autonomous directional decision-making

This project focuses on structured problem solving and system-level thinking rather than isolated scripts.

---

## Demonstration

A short demo video is included in the `demo/` folder demonstrating autonomous navigation behaviour within the simulation environment.

---

## Technologies Used

- Python
- CoppeliaSim (V-REP)
- Remote API integration
- Sensor-based robotics control

---

## How to Run

1. Install CoppeliaSim.
2. Open the desired scene file from the `scenes/` directory.
3. Run the corresponding control logic from `src/` or notebooks.
4. Ensure Remote API is properly configured within CoppeliaSim.

Note: Remote API libraries are not included in this repository and must be configured separately.

---

## Engineering Focus

This project emphasizes:

- Scalable system organization
- Clean separation of control logic
- Real-world robotics simulation practices
- Structured experimentation through notebooks

---

## Author

Mahi Chudela  
First Class BSc Computer Science Graduate  
Focused on Software Engineering, cloud-ready architecture, and system-level problem solving.