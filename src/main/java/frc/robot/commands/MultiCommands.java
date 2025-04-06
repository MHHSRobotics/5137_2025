package frc.robot.commands;

import java.time.Instant;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    private BooleanSupplier armInDangerZone;

    private boolean cat = false;
    private boolean cat2 = false;

    private Command activeCommand;

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
        if (swerve != null && state.robotPosition != null) {
            if (state.autoAlign) {
                swerveCommands.driveToPose(state.robotPosition.alliancePos()).schedule();
            } else {
                swerve.setRotationTarget(state.robotPosition.bluePos().getRotation());
            }
        } else if (swerve != null) {
            swerve.autoAligning = false;
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
               (swerve == null || swerve.atTarget() || edu.wpi.first.wpilibj.RobotState.isAutonomous());
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

    private boolean targetSet(RobotState target) {
        return (target.armPosition == null || arm.getGoal() == target.armPosition) &&
        (target.elevatorPosition == null || elevator.getGoal() == target.elevatorPosition) &&
        (target.wristPosition == null || wrist.getGoal() == target.wristPosition);
    }
    
    /**
     * Move the robot to a specific state with a timeout.
     * 
     * @param state The state to move to
     * @param timeout The timeout in seconds
     * @return A command that moves the robot to the specified state
     */
    public Command moveToState(Supplier<RobotState> state, double timeout) {
        return new ParallelRaceGroup(new WaitCommand(timeout),new FunctionalCommand(
            () -> {
                timer.restart();
                setTargetState(state.get());
            },
            () -> {},
            (interrupted) -> {},
            () -> atSetpoint() && timer.hasElapsed(0.2)));
    }

    public Command moveToStateSequenced(Supplier<RobotState> state, boolean wristFirst) {
        return new SequentialCommandGroup(
            moveToState(() -> state.get().withWrist(Units.degreesToRadians(-145)).onlyWrist(), 1).onlyIf(() -> wristFirst),
            moveToState(() -> state.get().noWrist(), 1.5),
            moveToState(() -> state.get().onlyWrist(), 1)
        );
    }

    public Command moveToDefault() {
        return new ConditionalCommand(
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            moveToState(() -> RobotPositions.defaultState.withWrist(Units.degreesToRadians(-145))),
                            moveToState(() -> RobotPositions.defaultState.withWrist(Units.degreesToRadians(-45)).onlyWrist())
                        ),
                        new InstantCommand(() -> cat2 = false)),
                    moveToState(() -> RobotPositions.defaultState.withWrist(Units.degreesToRadians(-45))),
                    () -> cat2
                ),
                intakeCommands.pulseIntake(),
                new InstantCommand(() -> cat = false)),
            new ConditionalCommand(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            moveToState(() -> RobotPositions.defaultState.noElevator().withWrist(Units.degreesToRadians(-145))),
                            intakeCommands.intake(() -> 1.0)
                        ),
                        new ParallelDeadlineGroup(
                            moveToState(() -> RobotPositions.defaultState.noArm()),
                            intakeCommands.pulseIntake()
                        ),
                        intakeCommands.stop()
                    ),
                    new InstantCommand(() -> cat2 = false)
                ),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        moveToState(() -> RobotPositions.defaultState),
                        intakeCommands.pulseIntake()
                    ),
                    intakeCommands.stop()
                ),
                () -> cat2),
            () -> cat)
            .withName("Default");
    }

    public Command moveToPreScoringState() {
        return moveToState(() -> RobotPositions.preScoringState.noElevator());
    }

    public Command moveToProcessor() {
        return moveToStateSequenced(() -> RobotPositions.processorState, true).withName("Processor");
    }

    public Command getCoralFromSource() {
        return new ParallelCommandGroup(
            moveToState(() -> edu.wpi.first.wpilibj.RobotState.isAutonomous() ? RobotPositions.sourceStates[0].noPosition() : getClosestState(RobotPositions.sourceStates), 1),
            intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),
            simCoralIntake()
        ).withName("CoralSource");
    }

    public Command getCoralFromGround(boolean auto) {
        return new ParallelCommandGroup(
            moveToStateSequenced(() -> auto ? RobotPositions.groundCoralAuto : RobotPositions.groundCoralTeleop, true),
            intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),
            simCoralIntake(),
            new InstantCommand(()->cat2=true)
        ).withName("CoralGround");
    }

    public Command getAlgaeFromReef() {
        Supplier<RobotState> state = () -> getClosestState(RobotPositions.algaeStates);
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                moveToState(() -> state.get().onlyWrist()),
                moveToState(() -> state.get().noWrist())
            ),
            intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),
            simAlgaeIntake(),
            new InstantCommand(()->cat=true)
        ).withName("AlgaeReef");
    }

    public Command getAlgaeFromGround() {
        return new ParallelCommandGroup(
            moveToStateSequenced(() -> RobotPositions.groundAlgaeState, true),
            intakeCommands.setSpeed(() -> IntakeConstants.intakeSpeed),
            simAlgaeIntake(),
            new InstantCommand(()->cat=true),
            new InstantCommand(()->cat2=true)
        ).withName("AlgaeGround");
    }

    public Command placeCoral(Supplier<Integer> level, boolean isHorizontal, Supplier<Integer> branch) {
        Supplier<RobotState> state = () -> (isHorizontal ? RobotPositions.scoringStatesHorizontal : RobotPositions.scoringStatesVertical)[level.get()][branch.get()];
        Command moveToPosition = new SequentialCommandGroup(
            moveToState(() -> (state.get().robotPosition != null ? RobotPositions.preScoringState.withPosition(state.get().robotPosition) : RobotPositions.preScoringState).noElevator(), 2.5),
            new ParallelCommandGroup(
                intakeCommands.intake(() -> 0.1),
                moveToState(() -> isHorizontal && level.get() == 3 ? RobotPositions.preScoringState.onlyElevator().noPosition() : state.get().onlyElevator().noPosition())
            ),
            moveToState(() -> state.get().noElevator().noPosition())
        );

        if (isHorizontal && level.get() == 3) {
            moveToPosition = moveToPosition.andThen(moveToState(() -> state.get().onlyElevator().noPosition(), 1));
        }

        return new SequentialCommandGroup(
            moveToPosition,
            intakeCommands.intake(() -> 0.05),
            new ParallelCommandGroup(
                isHorizontal && level.get() != 3 ? level.get() == 0 ? intakeCommands.setSpeed(() -> 0.2, () -> -0.6) : new ParallelCommandGroup(intakeCommands.setSpeed(() -> 0.2, () -> -0.6), moveToState(() -> RobotPositions.preScoringState.noElevator().noPosition())) : intakeCommands.outtake(),
                simCoralOuttake()
            )
        ).withName("CoralScore");
    }

    public Command placeCoralVertical(int level, boolean autoAlign) {
        Supplier<RobotState> state = level == 0 ? () -> RobotPositions.scoringStatesVertical[level][level] : () -> getClosestState(RobotPositions.scoringStatesVertical[level]);
        return new SequentialCommandGroup(
            moveToState(() -> (state.get().robotPosition != null && autoAlign ? RobotPositions.preScoringState.withPosition(state.get().robotPosition) : RobotPositions.preScoringState).noElevator(), 2),
            new ParallelCommandGroup(
                intakeCommands.intake(() -> 0.2),
                moveToState(() -> state.get().onlyElevator().noPosition(), level == 3 ? 1 : 0.2)
            ),
            moveToState(() -> state.get().noElevator().noPosition(), level == 3 ? 1 : 0.5),
            new WaitCommand(0.2).onlyIf(() -> level == 3),
            intakeCommands.intake(() -> 0.05),
            new ParallelCommandGroup(
                intakeCommands.outtake(),
                simCoralOuttake()
            ),
            moveToDefault().onlyIf(() -> !edu.wpi.first.wpilibj.RobotState.isAutonomous())
        );
    }

    public Command placeCoralHorizontal(int level, boolean autoAlign) {
        Supplier<RobotState> state = level == 0 ? () -> RobotPositions.scoringStatesHorizontal[level][level] : () -> getClosestState(RobotPositions.scoringStatesHorizontal[level]);
        return new SequentialCommandGroup(
            moveToState(() -> (state.get().robotPosition != null && autoAlign ? RobotPositions.preScoringState.withPosition(state.get().robotPosition) : RobotPositions.preScoringState).noElevator(), 2.5),
            new ParallelCommandGroup(
                intakeCommands.intake(() -> 0.1),
                moveToState(() -> level == 1 || level == 2 ? state.get().withArm(0.3).noPosition() : (level == 3 ? RobotPositions.preScoringState : state.get()).onlyElevator().noPosition(), 1)
            ),
            moveToState(() -> level == 1 || level == 2 ? state.get().onlyArm().noPosition() : state.get().noElevator().noPosition(), 1),
            moveToState(() -> state.get().onlyElevator().noPosition(), 1).onlyIf(() -> level == 3),
            new WaitCommand(0.2),
            intakeCommands.intake(() -> 0.05),
            new ParallelCommandGroup(
                new ConditionalCommand(
                    moveToState(() -> RobotPositions.defaultState.onlyElevator()).onlyIf(() -> level == 2),
                    moveToState(() -> RobotPositions.preScoringState.noElevator().noPosition()),
                    () -> (level == 1 || level == 2)),
                intakeCommands.setSpeed(() -> 1.0, () -> level <= 1 ? -0.3 : -0.6),
                simCoralOuttake()
            ),
            moveToDefault().onlyIf(() -> level != 1)
        );
    }

    public Command placeCoral(int level, boolean isHorizontal) {
        return (isHorizontal ? placeCoralHorizontal(level, true) : placeCoralVertical(level, true)).withName("CoralScore");
    }

    public Command placeCoralAuto(int level, boolean isHorizontal) {
        return (isHorizontal ? placeCoralHorizontal(level, false) : placeCoralVertical(level, false)).withName("CoralScore");
    }

    public Command scoreBarge() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                moveToState(() -> RobotPositions.bargeStates[0].noElevator()),
                intakeCommands.pulseIntake()
            ),
            new ParallelCommandGroup(
                moveToState(() -> RobotPositions.bargeStates[0].onlyElevator()),
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    moveToState(() -> RobotPositions.bargeStates[1])),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.7),
                        intakeCommands.pulseIntake()
                    ),
                    new ParallelCommandGroup(
                        intakeCommands.outtake(),
                        simAlgaeOuttake()
                    )
                )
            ),
            moveToDefault()
        ).withName("BargeScore");
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
}