package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CageConstants {
    public static final Pose2d leftCage = new Pose2d(new Translation2d(8.5, 7.25), new Rotation2d(3*Math.PI/2));
    public static final Pose2d centerCage = new Pose2d(new Translation2d(8.5, 6.15), new Rotation2d(3*Math.PI/2));
    public static final Pose2d rightCage = new Pose2d(new Translation2d(8.5, 5.05), new Rotation2d(3*Math.PI/2));
}
