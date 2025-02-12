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
     * @param targetPose Optional target pose for states with null positions
     */
    public Command moveToState(Supplier<SwerveSystem.SwerveSystemState> state) {
        return new SequentialCommandGroup(
            new InstantCommand(()->swerveSystem.setTargetState(state.get())),
            waitUntilFinished()
        );
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

    public Command moveToBranch(Supplier<Integer> level,Supplier<Integer> branch){
        return moveToState(()->SwerveSystemConstants.getScoringStates()[level.get()][branch.get()]);
    }

    public Command moveToProcessor(){
        return moveToState(()->SwerveSystemConstants.getProcessor());
    }

    private SwerveSystemState levelState;
    public Command moveToLevel(int level){
        return new SequentialCommandGroup(
            new InstantCommand(()->{
                levelState=swerveSystem.getClosestState(SwerveSystemConstants.getScoringStates()[level]);
            }),
            moveToState(()->levelState)
        );
    }
}
