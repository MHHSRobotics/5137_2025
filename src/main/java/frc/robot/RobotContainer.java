package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorCommands;

public class RobotContainer {
  Elevator elevator;
  ElevatorCommands elevatorCommands;
  public RobotContainer() {
    elevator=new Elevator();
    elevatorCommands=new ElevatorCommands(elevator);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
