package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.subsystems.SwerveSystem.SwerveSystemState;

public class SwerveSystemCommands {
    private SwerveSystem swerveSystem;

    public SwerveSystemCommands(SwerveSystem swerveSystem) {
        this.swerveSystem = swerveSystem;
    }

    /**
     * Creates a command that moves to a specific state
     * 
     * @param state The state to move to
     */
    public Command moveToState(Supplier<SwerveSystem.SwerveSystemState> state) {
        return new SequentialCommandGroup(
            new InstantCommand(()->swerveSystem.setTargetState(state.get())),
            waitUntilFinished()
        );
    }

    /**
     * Creates a command that moves to a specific state, sets swerve first then elevator arm and wrist
     * 
     * @param state The state to move to
     */
    public Command moveToStateSequenced(Supplier<SwerveSystem.SwerveSystemState> state) {
        SwerveSystemState m_state = state.get();
        if (m_state != null) {
            return new SequentialCommandGroup(
                moveToState(() -> new SwerveSystemState(null, null, null, m_state.botPosition)),
                moveToState(() -> new SwerveSystemState(m_state.armPosition, m_state.elevatorPosition, m_state.wristPosition, null))
            );
        } else {
            return new WaitCommand(0.0);
        }
    }

    /**
     * Creates a command that moves to a specific state, sets swerve first and pre arm and wrist positions, then elevator arm and wrist
     * 
     * @param state The state to move to
     */
    public Command moveToStateSequenced(Supplier<SwerveSystem.SwerveSystemState> state, double armPosition, double wristPosition) {
        SwerveSystemState m_state = state.get();
        if (m_state != null) {
            return new SequentialCommandGroup(
                moveToState(() -> new SwerveSystemState(armPosition, null, wristPosition, m_state.botPosition)),
                moveToState(() -> new SwerveSystemState(m_state.armPosition, m_state.elevatorPosition, m_state.wristPosition, null))
            );
        } else {
            return new WaitCommand(0.0);
        }
    }

    public Command waitUntilFinished(){
        return new ParallelRaceGroup(
            new WaitUntilCommand(()->swerveSystem.atSetpoint()),
            new WaitCommand(SwerveSystemConstants.timeout)
        );
    }

    private SwerveSystemState sourceState;
    public Command moveToSource(){
        return new SequentialCommandGroup(
            new InstantCommand(()->{
                sourceState=swerveSystem.getClosestState(SwerveSystemConstants.getSourceStates());
            }),
            moveToState(()->sourceState)
        );
    }

    public Command moveToGround(Supplier<Pose2d> pose){
        return moveToState(()->SwerveSystemConstants.getGroundIntake().withPose(pose.get()));
    }

    public Command moveToAlgae(Supplier<Integer> side){
        return moveToState(()->SwerveSystemConstants.getAlgaeStates()[side.get()]);
    }

    public Command moveToAlgae() {
        return moveToStateSequenced(()->swerveSystem.getClosestState(SwerveSystemConstants.getAlgaeStates()));
    }

    public Command moveToBranch(Supplier<Integer> level,Supplier<Integer> branch){
        return moveToState(()->SwerveSystemConstants.getScoringStates()[level.get()][branch.get()]);
    }

    public Command moveToProcessor(){
        return moveToStateSequenced(()->SwerveSystemConstants.getProcessor());
    }

    public Command moveToBarge(){
        return moveToState(()->SwerveSystemConstants.getBargeState());
    }

    public Command moveToDefault(){
        return moveToState(()->SwerveSystemConstants.getDefaultState());
    }

    public Command moveToLevel(int level){
        return moveToStateSequenced(()->swerveSystem.getClosestState(SwerveSystemConstants.getScoringStates()[level]), SwerveSystemConstants.getPrescoringState().armPosition, SwerveSystemConstants.getPrescoringState().wristPosition);
    }

    public Command coralIntake(){
        return new InstantCommand(()->swerveSystem.coralIntake());
    }

    public Command algaeIntake(){
        return new InstantCommand(()->swerveSystem.algaeIntake());
    }

    public Command outtakeCoral(){
        return new InstantCommand(()->swerveSystem.outtakeCoral());
    }

    public Command outtakeAlgae(){
        return new InstantCommand(()->swerveSystem.outtakeAlgae());
    }
}
