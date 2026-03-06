<h1 align="center">🤖Eurobot 2026: Secondary Robot Firmware🤖</h1>
This repository is a sample of the firmware developed for the secondary robot of the Eurobot 2026 competition team. Due to the team's privacy policies, the full code is private.

## 🚀 Hardware Overview
The robot is designed on an omnidirectional base with a high-speed architecture based on Teensy 4.0, optimized for executing concurrent tasks and precise motor control.

- MCU: Teensy 4.0.

- Locomotion: 3x Omni Wheels with high-resolution drivers.

- Perception: 8x VL53L1X (ToF) sensors via I2C multiplexer.

- Actuators: Lifting system by Stepper and vacuum suction system.

## 🛠️ Software Architecture
The firmware is organized modularly into 4 main libraries, following principles of high cohesion and low coupling. Each module manages its own states and resources:

- `omni-controller` (Sample Available): It is the core of movement. It uses the TeensyStep4 library to manage acceleration ramps and motor synchronization.
  
- `tof-controller`: Obstacle detection through an I2C multiplexer connected to 8 laser sensors.
  
- `elevator-controller`: Control of the vertical lifting system through stepper motors (TeensyStep4) for millimetric positioning and end-of-travel switches for calibration.
  
- `pneumatic-controller`: Control of suction gripper systems (solenoid valve and vacuum pump).

## 📂 Organization & Project Structure
To maintain maintainable and scalable code, the project is divided into configuration and route logic files:

- `config.h` / `config.cpp`: The heart of the parameterization. Here the data structures, physical pin assignments (Pinout), and critical system constants are defined.
  
- `routes.h` / `routes.cpp`: Definition of the paths and movement sequences that the robot must execute during the competition.

## Contact & hiring note
If you're interested in hiring or reviewing more of my work, please check the public repos in my GitHub profile or contact me directly. I originally developed this project as tooling for my university robotics team; additional robot-specific integrations live in private team repositories.

If you need more details, feel free to contact me via my GitHub profile.
