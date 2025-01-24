package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.constants.ArmConstants;

public class ArmCommands {

    private Arm arm;

    public ArmCommands(Arm arm) {
        this.arm = arm;
    }

    public InstantCommand setSpeed(DoubleSupplier speed) {
        return new InstantCommand(()-> arm.setSpeed(speed.getAsDouble()), arm);
    }
    
    public InstantCommand setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> arm.setGoal(goal.getAsDouble()), arm);
    }

    public InstantCommand moveToL1() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.L1goal), arm);
    }

    public InstantCommand moveToL2() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.L2goal), arm);
    }

    public InstantCommand moveToL3() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.L3goal), arm);
    }

    public InstantCommand moveToL4() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.L4goal), arm);
    }

    public InstantCommand moveToSource() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.sourceGoal), arm);
    }
    
    public InstantCommand moveToGround() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.groundGoal), arm);
    }

    public InstantCommand moveToInside() {
        return new InstantCommand(() -> arm.setGoal(ArmConstants.startingPose), arm);
    }
}
