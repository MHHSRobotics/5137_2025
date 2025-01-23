package frc.robot.constants;

public class ArmConstants {

    public static final double armSpeed = 3.0;
    public static final int motorId  = 20;
    public static final int encoderIda = 6; //idk
    public static final int encoderIdb = 7; //idk
    public static double kP = 0; // enter
    public static double kI = 0; // enter
    public static double kD = 0; // enter
    public static double ks = 0;
    public static double kg = 0;
    public static double kv = 0;
    public static final double min = 0; // pi /180
    public static final double max = 100;
    public static final double tolerance = .1;
    public static final double goal1 = -(43*Math.PI)/180; //enter low goal
    public static final double goal2 = (30*Math.PI)/180; //with 5 degree offset to make scoring easier
    public static final double goal3 = (30*Math.PI)/180; //enter level 3 goal
    public static final double goal4 = (5*Math.PI)/180; // enter high goal
    public static final double groundGoal = -(43*Math.PI)/180;
    public static final double sourceGoal = (25*Math.PI)/180;//75 degree angle
    public static final double startingPose = 0; // straight up
    public static final double armOffset = 0.0;
    public static final double gearRatio = 1.0;
    public static final double jkg = 500.0;
    public static final double armLength = 0.594;
}
