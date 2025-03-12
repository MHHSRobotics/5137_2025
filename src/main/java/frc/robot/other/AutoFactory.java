package frc.robot.other;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.GeneralConstants;

/**
 * The AutoFactory class is responsible for constructing and managing autonomous routines
 * for the robot. It uses a series of AutoStep objects to build a sequence of commands
 * that can be executed during the autonomous period of a match.
 */
public class AutoFactory {
    // Array to hold the steps (AutoStep objects) that make up the autonomous routine
    private AutoStep[] choices;

    // The final autonomous command that will be executed
    private PathPlannerAuto auto;

    private RobotContainer container;

    /**
     * Constructor for the AutoFactory class.
     * Initializes the autonomous steps and sets up a chooser on the SmartDashboard.
     *
     * @param multiCommands An instance of MultiCommands that provides the necessary commands
     *                      for the autonomous steps.
     */
    public AutoFactory(MultiCommands multiCommands, RobotContainer container) {
        // Initialize the array of AutoStep objects
        choices = new AutoStep[GeneralConstants.numAuto];

        this.container = container;

        // Populate the choices array with AutoStep objects
        for (int i = 0; i < GeneralConstants.numAuto; i++) {
            choices[i] = new AutoStep(i + 1, multiCommands);
        }
    }

    /**
     * Constructs the autonomous routine by combining the commands from each AutoStep
     * into a SequentialCommandGroup, which is then wrapped in a PathPlannerAuto command.
     */
    public void buildAuto() {
        // Create an array to hold the commands from each AutoStep
        Command[] autoCommands = new Command[GeneralConstants.numAuto];

        // Populate the array with commands from each AutoStep
        for (int i = 0; i < GeneralConstants.numAuto; i++) {
            autoCommands[i] = choices[i].getCommand();
        }

        // Create a SequentialCommandGroup from the array of commands and wrap it in a PathPlannerAuto
        auto = new PathPlannerAuto(new SequentialCommandGroup(autoCommands));

        container.resetGyro();
    }

    /**
     * Retrieves the constructed autonomous command. If the command has not been built yet,
     * it will trigger the build process before returning the command.
     *
     * @return The PathPlannerAuto command representing the autonomous routine.
     */
    public PathPlannerAuto getAuto() {
        // Ensure the auto command is built before returning it
        if (auto == null) {
            buildAuto();
        }
        return auto;
    }
}