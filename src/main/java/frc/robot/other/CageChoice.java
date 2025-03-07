package frc.robot.other;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CageConstants;

public class CageChoice {
    private SendableChooser<Pose2d> cageChoice;

    public CageChoice(){
        // Configure cage position chooser for autonomous
        cageChoice = new SendableChooser<Pose2d>();
        cageChoice.addOption("Left", CageConstants.leftCage);
        cageChoice.addOption("Center", CageConstants.centerCage);
        cageChoice.addOption("Right", CageConstants.rightCage);
        cageChoice.setDefaultOption("Center",CageConstants.centerCage);
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
