package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final Transform3d robotToBLATCamera = new Transform3d(new Translation3d(-0.3248,0.186,0.2794),new Rotation3d(0,Units.degreesToRadians(-12),Units.degreesToRadians(224.9))); // old = 231
    public static final Transform3d robotToBRATCamera = new Transform3d(new Translation3d(-0.3648,-0.266,0.2794),new Rotation3d(0,Units.degreesToRadians(-12),Units.degreesToRadians(-224.7))); // old = -231
    public static final Transform3d robotToFLATCamera = new Transform3d(new Translation3d(0.3175,0.222,0.273),new Rotation3d(0,Units.degreesToRadians(-12),Units.degreesToRadians(-51)));
   //public static final Transform3d robotToFrontObjectCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    public static final String[] classNames = {"Algae","Coral"};

    public static final double objectMarginOfError = 0.1;
}
