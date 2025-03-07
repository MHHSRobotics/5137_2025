package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.other.RobotUtils;
import frc.robot.states.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * The MultiCommands class is responsible for creating complex commands that involve multiple subsystems.
 * These commands are typically composed of several simpler commands that run in parallel or sequence.
 */
public class MultiCommands {
    // Subsystems
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Swerve swerve;
    
    // Command groups for each subsystem
    private IntakeCommands intakeCommands;
    @SuppressWarnings("unused")
    private SwerveCommands swerveCommands;
    @SuppressWarnings("unused")
    private HangCommands hangCommands;

    // State variables for commands
    private RobotState sourceState;
    private RobotState levelState;

    /**
     * Constructor for MultiCommands.
     */
    public MultiCommands(Arm arm, Elevator elevator, Wrist wrist, Swerve swerve, 
                         SwerveCommands swerveCommands, IntakeCommands intakeCommands, 
                         HangCommands hangCommands) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.intakeCommands = intakeCommands;
        this.hangCommands = hangCommands;
    }

    /**
     * Sets the target state for all subsystems.
     * 
     * @param state The state to set
     */
    private void setTargetState(RobotState state) {
        if (arm != null) {
            arm.setGoal(state.armPosition);
        }
        if (elevator != null) {
            elevator.setGoal(state.elevatorPosition);
        }
        if (wrist != null) {
            wrist.setGoal(state.wristPosition);
        }
        if (swerve != null && state.robotPosition != null) {
            swerve.setTargetPose(state.robotPosition.alliancePos());
        }
    }

    /**
     * Checks if all mechanisms are at their setpoints.
     * 
     * @return true if all mechanisms are at their setpoints, false otherwise
     */
    private boolean atSetpoint() {
        return (arm == null || arm.atSetpoint()) &&
               (elevator == null || elevator.atSetpoint()) &&
               (wrist == null || wrist.atSetpoint()) &&
               (swerve == null || swerve.atTarget());
    }

    /**
     * Find the closest state from an array of states based on robot position.
     * 
     * @param states Array of possible states
     * @return The closest state based on robot position
     */
    private RobotState getClosestState(RobotState[] states) {
        Pose2d currentPose = swerve == null ? new Pose2d() : swerve.getPose();
        return RobotUtils.getClosestState(currentPose, states, SwerveSystemConstants.rotationWeight);
    }

    /**
     * Move the robot to a specific state.
     * 
     * @param state The state to move to
     * @return A command that moves the robot to the specified state
     */
    public Command moveToState(Supplier<RobotState> state) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setTargetState(state.get())),
            waitUntilFinished()
        );
    }

    /**
     * Wait until the robot has reached its target state or a timeout occurs.
     * 
     * @return A command that waits until the robot has reached its target state or times out
     */
    public Command waitUntilFinished() {
        return new ParallelRaceGroup(
            new WaitUntilCommand(this::atSetpoint),
            new WaitCommand(SwerveSystemConstants.timeout)
        );
    }

    /**
     * Command to move to the source position.
     */
    public Command moveToSource() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                sourceState = getClosestState(SwerveSystemConstants.sourceStates);
            }),
            moveToState(() -> sourceState)
        );
    }

    /**
     * Command to move to the ground intake position.
     */
    public Command moveToGround(Supplier<Pose2d> pose) {
        return moveToState(() -> SwerveSystemConstants.groundIntake.withPose(pose.get()));
    }

    /**
     * Command to move to the algae intake position with specified side.
     */
    public Command moveToAlgae(Supplier<Integer> side) {
        return moveToState(() -> SwerveSystemConstants.algaeStates[side.get()]);
    }

    /**
     * Command to move to the default algae intake position.
     */
    public Command moveToAlgae() {
        return moveToState(() -> SwerveSystemConstants.algaeStates[0]);
    }

    /**
     * Command to move to a specific branch for scoring.
     */
    public Command moveToBranch(Supplier<Integer> level, Supplier<Integer> branch) {
        return moveToState(() -> SwerveSystemConstants.scoringStates[level.get()][branch.get()]);
    }

    /**
     * Command to move to the processor position.
     */
    public Command moveToProcessor() {
        return moveToState(() -> SwerveSystemConstants.processor);
    }

    /**
     * Command to move to the default position.
     */
    public Command moveToDefault() {
        return new ParallelCommandGroup(
            moveToState(() -> SwerveSystemConstants.defaultState),
            intakeCommands.stop()
        );
    }

    /**
     * Command to move to a specific level for scoring.
     */
    public Command moveToLevel(int level) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                levelState = SwerveSystemConstants.scoringStates[level][0];
            }),
            moveToState(() -> levelState)
        );
    }

    /**
     * Command to retrieve coral from the source.
     */
    public Command getCoralFromSource() {
        return new SequentialCommandGroup(
            moveToSource(),
            new ParallelCommandGroup(intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed))
        );
    }

    /**
     * Command to retrieve coral from the ground.
     */
    public Command getCoralFromGround(Supplier<Pose2d> pose) {
        return new SequentialCommandGroup(
            moveToGround(pose),
            new ParallelCommandGroup(intakeCommands.intake())
        );
    }

    /**
     * Command to place coral at a specific branch.
     */
    public Command placeCoral(Supplier<Integer> level, Supplier<Integer> branch) {
        return new SequentialCommandGroup(
            moveToBranch(level, branch),
            new WaitCommand(0.5),
            new ParallelCommandGroup(intakeCommands.outtake())
        );
    }

    /**
     * Command to retrieve algae from a specific side.
     */
    public Command getAlgae(Supplier<Integer> side) {
        return new SequentialCommandGroup(
            moveToAlgae(side),
            new ParallelCommandGroup(intakeCommands.intake())
            //,swerveCommands.driveBack()
        );
    }

    /**
     * Command to retrieve algae from the default position.
     */
    public Command getAlgae() {
        return new SequentialCommandGroup(
            moveToAlgae(),
            new ParallelCommandGroup(intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed))
            //,swerveCommands.driveBack()
        );
    }

    /**
     * Command to place algae at the processor.
     */
    public Command placeAlgae() {
        return new SequentialCommandGroup(
            moveToProcessor(),
            new ParallelCommandGroup(intakeCommands.outtake())
        );
    }
}