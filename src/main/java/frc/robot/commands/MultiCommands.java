package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.other.RobotUtils;
import frc.robot.positions.RobotPositions;
import frc.robot.positions.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * The MultiCommands class is responsible for creating complex commands that involve multiple subsystems.
 * These commands are typically composed of several simpler commands that run in parallel or sequence.
 */
@SuppressWarnings("unused")
public class MultiCommands {
    // Subsystems
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Swerve swerve;
    
    // Command groups for each subsystem
    private IntakeCommands intakeCommands;
    private SwerveCommands swerveCommands;
    private HangCommands hangCommands;
    
    // Reference to RobotPublisher for simulation
    private RobotPublisherCommands robotPublisherCommands;

    private Timer timer;

    /**
     * Constructor for MultiCommands.
     */
    public MultiCommands(Arm arm, Elevator elevator, Wrist wrist, Swerve swerve, 
                         SwerveCommands swerveCommands, IntakeCommands intakeCommands, 
                         HangCommands hangCommands, RobotPublisherCommands robotPublisherCommands) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.intakeCommands = intakeCommands;
        this.hangCommands = hangCommands;
        this.robotPublisherCommands = robotPublisherCommands;
        timer = new Timer();
    }

    /**
     * Sets the target state for all subsystems.
     * 
     * @param state The state to set
     */
    private void setTargetState(RobotState state) {
        if (arm != null && state.armPosition!=null) {
            arm.setGoal(state.armPosition);
        }
        if (elevator != null && state.elevatorPosition!=null) {
            elevator.setGoal(state.elevatorPosition);
        }
        if (wrist != null && state.wristPosition!=null) {
            wrist.setGoal(state.wristPosition);
        }
        if (swerve != null && state.robotPath != null) {
            swerve.followPath(state.robotPath);
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
        return RobotUtils.getClosestStateToPose(currentPose, states);
    }

    /**
     * Move the robot to a specific state.
     * 
     * @param state The state to move to
     * @return A command that moves the robot to the specified state
     */
    public Command moveToState(Supplier<RobotState> state) {
        return moveToState(state,Double.MAX_VALUE);
    }
    
    /**
     * Move the robot to a specific state with a timeout.
     * 
     * @param state The state to move to
     * @param timeout The timeout in seconds
     * @return A command that moves the robot to the specified state
     */
    public Command moveToState(Supplier<RobotState> state, double timeout) {
        return new FunctionalCommand(
            () -> { 
                timer.restart(); 
                setTargetState(state.get());
            },
            () -> {},
            (interrupted) -> {},
            () -> (atSetpoint() || timer.hasElapsed(timeout)) && timer.hasElapsed(0.2),
            elevator, arm, wrist);
    }
    
    /**
     * Move the robot to a specific state in a sequenced manner.
     * First moves to a pre-state (typically for positioning), then to the final state.
     * 
     * @param state The final state to move to
     * @param preState The pre-state to move to first
     * @return A command that moves the robot to the specified state in sequence
     */
    public Command moveToStateSequenced(Supplier<RobotState> state, Supplier<RobotState> preState) {
        if (preState.get() != RobotState.NULL) {
            return new SequentialCommandGroup(
                moveToState(() -> preState.get().withPath(state.get().robotPath), 5),
                moveToState(() -> state.get().stageOne()),
                moveToState(() -> state.get().stageTwo())
            );
        } else {
            return new SequentialCommandGroup(
                moveToState(() -> preState.get().withPath(state.get().robotPath), 5),
                moveToState(() -> state.get())
            );
        }
    }

    /**
     * Command to move to the source position.
     */
    public Command moveToSource() {
        return moveToState(() -> RobotPositions.sourceStates[0]);
    }

    /**
     * Command to move to the ground intake position.
     */
    public Command moveToGround(Supplier<Pose2d> pose) {
        return moveToState(() -> RobotPositions.groundIntake);
    }

    /**
     * Command to move to the algae intake position with specified side.
     */
    public Command moveToAlgae(Supplier<Integer> side) {
        return moveToState(() -> RobotPositions.algaeStates[side.get()]);
    }

    /**
     * Command to move to the default algae intake position.
     */
    public Command moveToAlgae() {
        return moveToState(() -> getClosestState(RobotPositions.algaeStates));
    }

    /**
     * Command to move to a specific branch for scoring.
     */
    public Command moveToBranch(Supplier<Integer> level, Supplier<Integer> branch) {
        return moveToStateSequenced(
            () -> RobotPositions.scoringStates[level.get()][branch.get()],
            () -> RobotState.NULL
        );
    }

    /**
     * Command to move to the processor position.
     */
    public Command moveToProcessor() {
        return moveToState(() -> RobotPositions.processorState);
    }

    /**
     * Command to move to the processor position.
     */
    public Command moveToBarge() {
        return moveToState(()->RobotPositions.bargeState);
    }

    /**
     * Command to move to the default position.
     */
    public Command moveToDefault() {
        return new ParallelCommandGroup(
            moveToState(() -> RobotPositions.defaultState),
            intakeCommands.stop()
        );
    }

    /**
     * Command to move to a specific level for scoring.
     */
    public Command moveToLevel(int level) {
        return moveToStateSequenced(
            () -> getClosestState(RobotPositions.scoringStates[level]),
            () -> RobotPositions.preScoringState
        );
    }

    private Command simCoralIntake(){
        if(robotPublisherCommands!=null){
            return robotPublisherCommands.simCoralIntake();
        }else{
            return new InstantCommand();
        }
    }

    private Command simCoralOuttake(){
        if(robotPublisherCommands!=null){
            return robotPublisherCommands.simCoralOuttake();
        }else{
            return new InstantCommand();
        }
    }

    private Command simAlgaeIntake(){
        if(robotPublisherCommands!=null){
            return robotPublisherCommands.simAlgaeIntake();
        }else{
            return new InstantCommand();
        }
    }

    private Command simAlgaeOuttake(){
        if(robotPublisherCommands!=null){
            return robotPublisherCommands.simAlgaeOuttake();
        }else{
            return new InstantCommand();
        }
    }

    /**
     * Command to retrieve coral from the source.
     */
    public Command getCoralFromSource() {
        return new SequentialCommandGroup(
            moveToSource(),
            new ParallelCommandGroup(
                intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),
                simCoralIntake()
            )
        );
    }

    /**
     * Command to retrieve coral from the ground.
     */
    public Command getCoralFromGround(Supplier<Pose2d> pose) {
        return new SequentialCommandGroup(
            moveToGround(pose),
            new ParallelCommandGroup(
                intakeCommands.intake(),
                simCoralIntake()
            )
        );
    }

    /**
     * Command to place coral at a specific branch.
     */
    public Command placeCoral(Supplier<Integer> level, Supplier<Integer> branch) {
        return new SequentialCommandGroup(
            moveToBranch(level, branch),
            intakeCommands.intake(() -> 0.05),
            new ParallelCommandGroup(
                intakeCommands.outtake(),
                simCoralOuttake()
            )
        );
    }

    /**
     * Command to place coral at a specific level.
     */
    public Command placeCoral(int level) {
        return new SequentialCommandGroup(
            moveToLevel(level),
            intakeCommands.intake(() -> 0.05),
            new ParallelCommandGroup(
                intakeCommands.outtake(),
                simCoralOuttake()
            )
        );
    }

    /**
     * Command to retrieve algae from a specific side.
     */
    public Command getAlgae(Supplier<Integer> side) {
        return new SequentialCommandGroup(
            moveToAlgae(side),
            new ParallelCommandGroup(
                intakeCommands.intake(),
                simAlgaeIntake()
            )
            //,swerveCommands.driveBack()
        );
    }

    /**
     * Command to retrieve algae from the default position.
     */
    public Command getAlgae() {
        return new SequentialCommandGroup(
            moveToAlgae(),
            new ParallelCommandGroup(
                intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),
                simAlgaeIntake()
            )
            //,swerveCommands.driveBack()
        );
    }

    /**
     * Command to place algae at the processor.
     */
    public Command placeAlgae() {
        return new SequentialCommandGroup(
            moveToProcessor(),
            new ParallelCommandGroup(
                intakeCommands.outtake(),
                simAlgaeOuttake()
            )
        );
    }
}