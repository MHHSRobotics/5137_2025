package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ElevatorConstants;
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

    public InstantCommand setManualControl(BooleanSupplier manualControl){
        return new InstantCommand(()->elevator.setManualControl(manualControl.getAsBoolean()));
    }

    public InstantCommand moveToL1(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L1goal));
    }
    
    public InstantCommand moveToL2(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L2goal));
    }

    public InstantCommand moveToL3(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L3goal));
    }

    public InstantCommand moveToL4(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L4goal));
    }

    public InstantCommand moveToIntake(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.intakeGoal));
    }

    public InstantCommand moveToGroundIntake(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.groundIntakeGoal));
    }
}
