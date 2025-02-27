package frc.robot.other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MultiCommands;
import frc.robot.constants.FieldGeometry;
import frc.robot.constants.RobotPositions;

/**
 * The `AutoStep` class is responsible for creating and managing three {@link SendableChooser}s
 * for level selection, reef position, and pickup position. These choosers are displayed on the
 * SmartDashboard, allowing users to select different options for autonomous routines.
 */
public class AutoStep {

    private int id; // Unique identifier for this AutoStep instance
    private MultiCommands multiCommands; // Reference to the MultiCommands class for executing commands

    // Choosers for selecting level, reef position, and pickup position
    private SendableChooser<Integer> levelChooser;
    private SendableChooser<Integer> branchChooser;
    private SendableChooser<Pose2d> pickupChooser;

    /**
     * Constructor for the AutoStep class.
     *
     * @param id The unique identifier for this AutoStep instance.
     * @param multiCommands Reference to the MultiCommands class for executing commands.
     */
    public AutoStep(int id, MultiCommands multiCommands) {
        this.id = id;
        this.multiCommands = multiCommands;

        // Initialize the level chooser with default and additional options
        levelChooser = new SendableChooser<Integer>();
        levelChooser.setDefaultOption("L4", 4);
        levelChooser.addOption("L3", 3);
        levelChooser.addOption("L2", 2);
        levelChooser.addOption("L1", 1);
        levelChooser.addOption("Algae", 0);
        levelChooser.addOption("No Auto",-1);
        SmartDashboard.putData("levelChoices/" + id, levelChooser);

        // Initialize the reef and pickup choosers with coral poses
        switchToCoralPoses();

        // Add a listener to the level chooser to switch between coral and algae poses
        levelChooser.onChange((Integer choice) -> {
            if (choice==0) {
                switchToAlgaePoses();
            } else if(choice==-1){
                switchToNoAuto();
            }else{
                switchToCoralPoses();
            }
        });
    }

    /**
     * Initializes the branch and pickup choosers with coral poses.
     */
    private void switchToCoralPoses() {
        // Initialize the branch chooser with default and additional options
        branchChooser = new SendableChooser<Integer>();
        branchChooser.setDefaultOption("A", 0);
        for (int i = 1; i < FieldGeometry.reefSides * 2; i++) {
            branchChooser.addOption(Character.toString('A' + i), i);
        }
        SmartDashboard.putData("branchChoices/" + id, branchChooser);

        // Initialize the pickup chooser with default and additional options
        pickupChooser = new SendableChooser<Pose2d>();
        pickupChooser.setDefaultOption("RightStation Right", RobotPositions.stations[0].alliancePos());
        pickupChooser.addOption("RightStation Left", RobotPositions.stations[1].alliancePos());
        pickupChooser.addOption("LeftStation Right", RobotPositions.stations[2].alliancePos());
        pickupChooser.addOption("LeftStation Left", RobotPositions.stations[3].alliancePos());
        pickupChooser.addOption("Left Ground", RobotPositions.pickups[0].alliancePos());
        pickupChooser.addOption("Center Ground", RobotPositions.pickups[1].alliancePos());
        pickupChooser.addOption("Right Ground", RobotPositions.pickups[2].alliancePos());
        SmartDashboard.putData("pickupChoices/" + id, pickupChooser);
    }

    /**
     * Initializes the reef and pickup choosers with algae poses.
     */
    private void switchToAlgaePoses() {
        // Initialize the reef chooser with default and additional options for algae poses
        branchChooser = new SendableChooser<Integer>();
        branchChooser.setDefaultOption("AB", 0);
        for (int i = 1; i < FieldGeometry.reefSides; i++) {
            branchChooser.addOption(Character.toString('A' + (2 * i)) + Character.toString('A' + (2 * i + 1)), i);
        }
        SmartDashboard.putData("branchChoices/" + id, branchChooser);

        // Initialize the pickup chooser (no options added for algae poses)
        pickupChooser = new SendableChooser<Pose2d>();
        SmartDashboard.putData("pickupChoices/" + id, pickupChooser);
    }

    private void switchToNoAuto(){
        pickupChooser = new SendableChooser<Pose2d>();
        SmartDashboard.putData("pickupChoices/" + id, pickupChooser);

        branchChooser = new SendableChooser<Integer>();
        SmartDashboard.putData("branchChoices/" + id, branchChooser);
    }

    /**
     * Generates a command sequence based on the selected options from the choosers.
     *
     * @return A {@link Command} representing the sequence of actions to be executed.
     */
    public Command getCommand() {
        int level=levelChooser.getSelected();
        if(level==-1){
            return new InstantCommand();
        }else if(level==0){
            return multiCommands.getAlgae(()->branchChooser.getSelected());
        }else{
            return new SequentialCommandGroup(
                multiCommands.getCoralFromGround(()->pickupChooser.getSelected()),
                multiCommands.placeCoral(()->levelChooser.getSelected(), ()->branchChooser.getSelected())
            );
        }
        
    }
}