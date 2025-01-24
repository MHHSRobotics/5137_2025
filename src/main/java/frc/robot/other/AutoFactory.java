package frc.robot.other;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.elastic.AutoChoice;

public class AutoFactory {
    private AutoChoice choiceOne;
    private AutoChoice choiceTwo;
    private AutoChoice choiceThree;
    private AutoChoice choiceFour;
    private AutoChoice choiceFive;
    private Supplier<Command> groundIntake;
    private Supplier<Command> sourceIntake;
    
    private SendableChooser<Boolean> build;
    private PathPlannerAuto auto;

    @SuppressWarnings("unchecked")
    public AutoFactory(Supplier<Command>... commands) {
        choiceOne = new AutoChoice(1, commands);
        choiceTwo = new AutoChoice(2, commands);
        choiceThree = new AutoChoice(3, commands);
        choiceFour = new AutoChoice(4, commands);
        choiceFive = new AutoChoice(5, commands);
        groundIntake = commands[5];
        sourceIntake = commands[6];

        build = new SendableChooser<Boolean>();
        build.setDefaultOption("AUTO NOT BUILT", false);
        build.addOption("AUTO BUILT", true);
        SmartDashboard.putData("Auto Builder", build);

        build.onChange((Boolean build) -> {
            if (build) {
                buildAuto();
            }
        });
    }

    public Command getCoral(Pose2d path) {
        if (path == null) {
            return new WaitCommand(0.0);
        } else {
            if (path.getY() > 1.75 && path.getY() < 6.3) {
                return new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(path, SwerveConstants.constaints),
                    groundIntake.get()
                );
            } else {
                return new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(path, SwerveConstants.constaints),
                    sourceIntake.get()
                );
            }
        }
    }

    public void buildAuto() {
        auto = new PathPlannerAuto(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceOne.getPose(), SwerveConstants.constaints),
                    choiceOne.getCommand()
                ),
                getCoral(choiceTwo.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceTwo.getPose(), SwerveConstants.constaints),
                    choiceTwo.getCommand()
                ),
                getCoral(choiceThree.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceThree.getPose(), SwerveConstants.constaints),
                    choiceThree.getCommand()
                ),
                getCoral(choiceFour.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceFour.getPose(), SwerveConstants.constaints),
                    choiceFour.getCommand()
                ),
                getCoral(choiceFive.getPickup()),
                new ParallelCommandGroup(
                    AutoBuilder.pathfindToPose(choiceFive.getPose(), SwerveConstants.constaints),
                    choiceFive.getCommand()
                )
            )
        );
    }

    public PathPlannerAuto getAuto() {
        if (auto == null) {
            buildAuto();
        }
        return auto;
    }
}