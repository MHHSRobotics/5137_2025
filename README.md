# FRC Team 5137 Robot Code (2025)

This repository contains the robot code for FRC Team 5137's 2025 competition robot. The project is built using WPILib and features a sophisticated swerve drive system along with various game piece manipulation mechanisms.

## Overview

The robot is designed with multiple subsystems working together to compete in the FRC 2025 game. Key features include:

- Swerve drive system with field-oriented control
- Vision-assisted autonomous navigation
- Game piece manipulation systems (Arm, Intake)
- Advanced control systems using LQR (Linear Quadratic Regulator)

## Subsystems

### Swerve Drive
- Field-oriented and robot-oriented driving modes
- Vision integration for pose estimation
- PathPlanner integration for autonomous path following
- System identification capabilities for tuning
- Supports both simulation and real hardware

### Arm
- Single-jointed robotic arm with advanced control
- LQR-based position control
- Gravity compensation and feedforward control
- Position and velocity safety limits
- Simulation support

### Intake
- Motor-controlled intake mechanism
- Variable speed control
- Simple start/stop functionality

## Control Architecture

### Robot Container
The `RobotContainer` class serves as the main orchestrator, managing:
- Controller bindings
- Subsystem initialization
- Command composition
- Autonomous routine selection

### Control Systems
- Advanced state-space control using LQR
- Vision-based pose estimation
- Pathfinding and autonomous navigation
- Telemetry and data logging

## Dependencies

- WPILib
- Phoenix 6 (CTRE)
- PathPlanner
- PhotonVision

## Development

### Building
This project uses Gradle for build management. To build the project:

```bash
./gradlew build
```

### Deployment
To deploy to the robot:

```bash
./gradlew deploy
```

### Simulation
The project includes simulation support for testing:

```bash
./gradlew simulateJava
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

[Insert License Information]

## Team

FRC Team 5137 - [Team Name]