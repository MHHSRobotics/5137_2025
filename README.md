# Team 5137 - Iron Kodiaks - 2025 FRC Robot Code

This repository contains the robot code for FRC Team 5137 (Iron Kodiaks)'s 2025 competition robot. The robot is built using WPILib's 2025 framework and features a sophisticated control system with multiple subsystems.

## Robot Features

- **Swerve Drive System**: Advanced swerve drive implementation with field-oriented control
- **Vision Processing**: Computer vision capabilities for autonomous operation and target tracking
- **Manipulator Systems**:
  - Elevator mechanism
  - Articulated arm with wrist control
  - Intake system
  - Hanging mechanism
- **LED Feedback System**: Visual feedback system for robot state indication
- **Game Piece Management**: Specialized systems for handling game pieces
- **Autonomous Capabilities**: Path planning and autonomous routines using PathPlanner

## Technical Details

- **Programming Language**: Java 17
- **Build System**: Gradle with GradleRIO 2025.2.1
- **Control System**: 
  - PS5 Controllers for driver and operator control
  - Advanced command-based programming structure
  - LQR-based position control for mechanisms
- **Major Dependencies**:
  - WPILib 2025
  - PathPlanner library
  - CTRE Phoenix 6 libraries
  - PhotonVision

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

## Development Setup

1. Install WPILib 2025 VSCode
2. Clone this repository
3. Open the project in VSCode with the WPILib extension
4. Build using Gradle: `./gradlew build`
5. Deploy to robot: `./gradlew deploy`
6. Simulate: `./gradlew simulateJava`

## Project Structure

- `src/main/java/frc/robot/`
  - `commands/`: Command classes for robot actions
  - `subsystems/`: Robot subsystem implementations
  - `constants/`: Configuration constants
  - `gamepieces/`: Game piece handling logic
  - `elastic/`: Elastic system implementations
  - `other/`: Additional utility classes
  - `states/`: Robot state definitions
  - `visualization/`: Visualization and telemetry

## Contributing

Team members should:
1. Create a new branch for features
2. Test code thoroughly
3. Submit pull requests for review
4. Ensure code follows team coding standards
