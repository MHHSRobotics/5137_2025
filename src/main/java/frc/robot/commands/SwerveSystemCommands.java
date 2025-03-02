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
     * @param targetPose Optional target pose for states with null positions
     */
    public Command moveToState(Supplier<SwerveSystem.SwerveSystemState> state) {
        return new SequentialCommandGroup(
            new InstantCommand(()->swerveSystem.setTargetState(state.get())),
            waitUntilFinished()
        );
    }

    /**
     * Creates a command that moves to a specific state, sets swerve and elevator first then arm and wrist
     * 
     * @param state The state to move to
     * @param targetPose Optional target pose for states with null positions
     * @param delay Delay between swerve and elevator movement and arm and wrist movement
     */
    public Command moveToState(Supplier<SwerveSystem.SwerveSystemState> state, double delay, double elevatorShift) {
        SwerveSystemState m_state = state.get();
        if (m_state != null) {
            return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new WaitCommand(delay),
                    moveToState(() -> new SwerveSystemState(null, m_state.elevatorPosition + elevatorShift, null, m_state.botPosition, null))),
                moveToState(() -> new SwerveSystemState(m_state.armPosition, null, m_state.wristPosition, null, null))
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
        return moveToState(()->swerveSystem.getClosestState(SwerveSystemConstants.getAlgaeStates()));
    }

    public Command moveToBranch(Supplier<Integer> level,Supplier<Integer> branch){
        return moveToState(()->SwerveSystemConstants.getScoringStates()[level.get()][branch.get()]);
    }

    public Command moveToProcessor(){
        return moveToState(()->SwerveSystemConstants.getProcessor());
    }

    public Command moveToBarge(){
        return moveToState(()->SwerveSystemConstants.getBargeState());
    }

    public Command moveToDefault(){
        return moveToState(()->SwerveSystemConstants.getDefaultState());
    }

    public Command moveToLevel(int level){
        return moveToState(()->swerveSystem.getClosestState(SwerveSystemConstants.getScoringStates()[level]));

    }

    public Command moveToLevel(int level, double delay){
        return moveToLevel(level, delay, 0.0);
    }

    public Command moveToLevel(int level, double delay, double elevatorShift){
        return moveToState(()->swerveSystem.getClosestState(SwerveSystemConstants.getScoringStates()[level]), delay, elevatorShift);
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
