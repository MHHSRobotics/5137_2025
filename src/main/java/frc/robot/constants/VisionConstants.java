package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final Transform3d robotToBLATCamera = new Transform3d(new Translation3d(-0.29845,0.24765,0.2413),new Rotation3d(0,-Math.PI/6,Math.PI/2));
     public static final Transform3d robotToBRATCamera = new Transform3d(new Translation3d(-0.2667,-0.29845,0.27305),new Rotation3d(0,0,Math.PI));
     public static final Transform3d robotToFLATCamera = new Transform3d(new Translation3d(0.23495,0.29845,0.292),new Rotation3d(0,-Math.PI/6,0));
   //public static final Transform3d robotToFrontObjectCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    public static final String[] classNames = {"Algae","Coral"};

    public static final double objectMarginOfError = 0.1;
}
