package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommands {
    Elevator elevator;
    public ElevatorCommands(Elevator elevator){
        this.elevator = elevator;
    }

    public Command setSpeed(DoubleSupplier speed){
        return new InstantCommand(()->elevator.setSpeed(speed.getAsDouble()),elevator);
    }

    public Command setGoal(DoubleSupplier goal){
        return new InstantCommand(()->elevator.setGoal(goal.getAsDouble()),elevator);
    }

    public Command moveToL1(){
        System.out.println("x");
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L1goal),elevator);
    }
    
    public Command moveToL2(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L2goal),elevator);
    }

    public Command moveToL3(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L3goal),elevator);
    }

    public Command moveToL4(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.L4goal),elevator);
    }

    public Command moveToIntake(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.intakeGoal),elevator);
    }

    public Command moveToGroundIntake(){
        return new InstantCommand(()->elevator.setGoal(ElevatorConstants.groundIntakeGoal),elevator);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir){
        return elevator.sysIdRoutine.quasistatic(dir);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir){
        return elevator.sysIdRoutine.dynamic(dir);
    }
}
