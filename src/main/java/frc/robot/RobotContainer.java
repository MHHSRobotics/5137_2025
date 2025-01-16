package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
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
    elevator.setDefaultCommand(elevatorCommands.setGoal(()->1-operator.getLeftY()));
    EventLoop event=new EventLoop();
    event.bind(()->elevatorCommands.moveToL4());
    operator.triangle(event);

    EventLoop event2=new EventLoop();
    event2.bind(()->elevatorCommands.moveToL3());
    operator.circle(event2);

    EventLoop event3=new EventLoop();
    event3.bind(()->elevatorCommands.moveToL2());
    operator.square(event3);

    EventLoop event4=new EventLoop();
    event4.bind(()->elevatorCommands.moveToL1());
    operator.cross(event4);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
