package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.IntakeConstants;

/**
 * The MultiCommands class is responsible for creating complex commands that involve multiple subsystems.
 * These commands are typically composed of several simpler commands that run in parallel or sequence.
 */
public class MultiCommands {
    // Command groups for each subsystem
    private SwerveSystemCommands swerveSystemCommands;
    private IntakeCommands intakeCommands;
    @SuppressWarnings("unused")
    private SwerveCommands swerveCommands;
    @SuppressWarnings("unused")
    private HangCommands hangCommands;


    /**
     * Constructor for MultiCommands.
     */
    public MultiCommands(SwerveSystemCommands swerveSystemCommands, SwerveCommands swerveCommands, IntakeCommands intakeCommands, HangCommands hangCommands) {
        this.swerveSystemCommands = swerveSystemCommands;
        this.swerveCommands = swerveCommands;
        this.intakeCommands = intakeCommands;
        this.hangCommands = hangCommands;
    }

    /**
     * Command to retrieve coral based on the robot's current pose.
     */
    public Command moveToDefault() {
        return new ParallelCommandGroup(
            swerveSystemCommands.moveToDefault(),
            intakeCommands.stop()
        );
    }

    /**
     * Command to retrieve coral based on the robot's current pose.
     */
    public Command getCoralFromSource() {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToSource(),
            new ParallelCommandGroup(intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),swerveSystemCommands.simCoralIntake())
        );
    }

    /*
    public Command getCoralFromGround(Supplier<Pose2d> pose) {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToGround(pose),
            new ParallelCommandGroup(intakeCommands.intake(),swerveSystemCommands.simCoralIntake())
        );
    }*/

    public Command placeCoral(Supplier<Integer> level,Supplier<Integer> branch) {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToBranch(level,branch),
            new WaitCommand(0.5),
            new ParallelCommandGroup(intakeCommands.outtake(),swerveSystemCommands.simCoralOuttake())
        );
    }

    public Command placeCoral(int level) {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToLevel(level),
            intakeCommands.intake(() -> 0.1),
            new ParallelCommandGroup(intakeCommands.outtake(),swerveSystemCommands.simCoralOuttake())
        );
    }

    public Command getAlgae(Supplier<Integer> side) {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToAlgae(side),
            new ParallelCommandGroup(intakeCommands.intake(),swerveSystemCommands.simAlgaeIntake())
            //,swerveCommands.driveBack()
        );
    }

    public Command getAlgae() {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToAlgae(),
            new ParallelCommandGroup(intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),swerveSystemCommands.simAlgaeIntake())
            //,swerveCommands.driveBack()
        );
    }

    public Command placeAlgae() {
        return new SequentialCommandGroup(
            swerveSystemCommands.moveToProcessor(),
            new ParallelCommandGroup(intakeCommands.outtake(),swerveSystemCommands.simAlgaeOuttake())
        );
    }
}