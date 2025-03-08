package frc.robot.other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotPositions;

public class CageChoice {
    private SendableChooser<Pose2d> cageChoice;

    public CageChoice(){
        // Configure cage position chooser for autonomous
        cageChoice = new SendableChooser<Pose2d>();
        cageChoice.addOption("Left", RobotPositions.leftCage.alliancePos());
        cageChoice.addOption("Center", RobotPositions.centerCage.alliancePos());
        cageChoice.addOption("Right", RobotPositions.rightCage.alliancePos());
        cageChoice.setDefaultOption("Center", RobotPositions.centerCage.alliancePos());
        SmartDashboard.putData("cageChoice", cageChoice);
    }

     /**
     * Gets the selected cage position from the chooser.
     *
     * @return The selected cage position.
     */
    public Pose2d getCage() {
        return cageChoice.getSelected();
    }
}
