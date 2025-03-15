package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.subsystems.SwerveSystem.SwerveSystemState;

public class SwerveSystemCommands {
    private SwerveSystem swerveSystem;
    private Timer timer;

    public SwerveSystemCommands(SwerveSystem swerveSystem) {
        this.swerveSystem = swerveSystem;
        this.timer = new Timer();
    }

    /**
     * Creates a command that moves to a specific state
     * 
     * @param state The state to move to
     */
    public Command moveToState(Supplier<SwerveSystem.SwerveSystemState> state) {
        return new FunctionalCommand(
            () -> {
                timer.restart();
                swerveSystem.setTargetState(state.get());
            },
            () -> {},
            (Boolean x) -> {},
            () -> isFinished(),
            swerveSystem
        );
    }

        /**
     * Creates a command that moves to a specific state
     * 
     * @param state The state to move to
     * @param timeout Time until command stops running if it doesn't reach setpoint
     */
    public Command moveToState(Supplier<SwerveSystem.SwerveSystemState> state, double timeout) {
        return new FunctionalCommand(
            () -> {
                timer.restart();
                swerveSystem.setTargetState(state.get());
            },
            () -> {},
            (Boolean x) -> {},
            () -> isFinished(timeout),
            swerveSystem
        );
    }

    public Command moveToStateSequenced(Supplier<SwerveSystem.SwerveSystemState> state, Supplier<SwerveSystem.SwerveSystemState> preState) {
        if (preState.get() != SwerveSystemState.NULL) {
            return new SequentialCommandGroup(
                moveToState(() -> preState.get().withPath(state.get().botPath), 1.0),
                moveToState(() -> state.get().stageOne()),
                moveToState(() -> state.get().stageTwo())
            );
        } else {
            return new SequentialCommandGroup(
                moveToState(() -> preState.get().withPath(state.get().botPath), 1.0),
                moveToState(() -> state.get())
            );
        }
    }

    public boolean isFinished(){
        return (swerveSystem.atSetpoint() || timer.hasElapsed(SwerveSystemConstants.timeout)) && timer.hasElapsed(0.1);
    }

    public boolean isFinished(double timeout){
        return (swerveSystem.atSetpoint() || timer.hasElapsed(timeout)) && timer.hasElapsed(0.1);
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

    /*
    TODO: Fix this
    public Command moveToGround(Supplier<Pose2d> pose){
        return moveToState(()->SwerveSystemConstants.getGroundIntake().withPose(pose.get()));
    }*/

    public Command moveToAlgae(Supplier<Integer> side){
        return moveToState(()->SwerveSystemConstants.getAlgaeStates()[side.get()]);
    }

    public Command moveToAlgae() {
        return moveToStateSequenced(()->swerveSystem.getClosestState(SwerveSystemConstants.getAlgaeStates()),()->SwerveSystemState.NULL);
    }

    public Command moveToBranch(Supplier<Integer> level,Supplier<Integer> branch){
        return moveToState(()->SwerveSystemConstants.getScoringStates()[level.get()][branch.get()]);
    }

    public Command moveToProcessor(){
        return moveToStateSequenced(()->SwerveSystemConstants.getProcessor(),()->SwerveSystemState.NULL);
    }

    public Command moveToBarge(){
        return moveToState(()->SwerveSystemConstants.getBargeState());
    }

    public Command moveToDefault(){
        return moveToState(()->SwerveSystemConstants.getDefaultState());
    }

    public Command moveToLevel(int level){
        return moveToStateSequenced(()->swerveSystem.getClosestState(SwerveSystemConstants.getScoringStates()[level]), ()->SwerveSystemConstants.getPrescoringState());
    }

    public Command simCoralIntake(){
        return new InstantCommand(()->swerveSystem.simCoralIntake());
    }

    public Command simAlgaeIntake(){
        return new InstantCommand(()->swerveSystem.simAlgaeIntake());
    }

    public Command simCoralOuttake(){
        return new InstantCommand(()->swerveSystem.simCoralOuttake());
    }

    public Command simAlgaeOuttake(){
        return new InstantCommand(()->swerveSystem.simAlgaeOuttake());
    }
}
