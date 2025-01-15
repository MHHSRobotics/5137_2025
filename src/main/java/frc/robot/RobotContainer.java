package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorCommands;

public class RobotContainer {
  private Elevator elevator;
  private ElevatorCommands elevatorCommands;
  private PS5Controller driver = new PS5Controller(0);
  private PS5Controller operator = new PS5Controller(1);

  public RobotContainer() {
    elevator = new Elevator();
    elevatorCommands = new ElevatorCommands(elevator);
    configureBindings();
  }

  private void configureBindings() {
    //elevator.setManualControl(true);
    elevator.setDefaultCommand(elevatorCommands.setSpeed(()->-operator.getLeftY()));
    // TODO
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
