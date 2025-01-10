package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class ElevatorCommands {
    Elevator elevator;
    public ElevatorCommands(Elevator elevator){
        this.elevator=elevator;
    }

    public InstantCommand setSpeed(DoubleSupplier speed){
        return new InstantCommand(()->elevator.setSpeed(speed.getAsDouble()));
    }

    public InstantCommand setGoal(DoubleSupplier goal){
        return new InstantCommand(()->elevator.setGoal(goal.getAsDouble()));
    }

    public InstantCommand seManualControl(BooleanSupplier manualControl){
        return new InstantCommand(()->elevator.setManualControl(manualControl.getAsBoolean()));
    }
}
