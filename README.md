# Team 5137 - Iron Kodiaks - 2025 FRC Robot Code

This repository contains the robot code for FRC Team 5137 (Iron Kodiaks)'s 2025 competition robot. The robot is built using WPILib's 2025 framework and features a sophisticated control system with multiple subsystems.

## Robot Features

- **Swerve Drive System**: Advanced swerve drive implementation for precise movement and control
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
- **Major Dependencies**:
  - WPILib 2025
  - PathPlanner library
  - CTRE Phoenix libraries
  - PhotonVision

## Development Setup

1. Install WPILib 2025 VSCode
2. Clone this repository
3. Open the project in VSCode with the WPILib extension
4. Build using Gradle: `./gradlew build`

## Project Structure

- `src/main/java/frc/robot/`
  - `commands/`: Command classes for robot actions
  - `subsystems/`: Robot subsystem implementations
  - `constants/`: Configuration constants
  - `gamepieces/`: Game piece handling logic
  - `elastic/`: Elastic system implementations
  - `other/`: Additional utility classes

## Contributing

Team members should:
1. Create a new branch for features
2. Test code thoroughly
3. Submit pull requests for review
4. Ensure code follows team coding standards