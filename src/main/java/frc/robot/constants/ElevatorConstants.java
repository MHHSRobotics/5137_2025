package frc.robot.constants;
public class ElevatorConstants {
    // Motor IDs
    public static final int leftMotorId = 1;
    public static final int rightMotorId = 2;

    // Encoder transform
    public static final double elevatorOffset = 0;
    public static final double metersPerRotation = 1;

    // Feedforward constants
    public static final double ks = 0.1;
    public static final double kg = 0.1;
    public static final double kv = 0.1;

    // PID constants
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 1;

    // Elevator tolerance
    public static final double elevatorTol = 0.1;

    // Elevator goals
    public static final double L1goal = 1.0;
    public static final double L2goal = 2.0;
    public static final double L3goal = 3.0;
    public static final double L4goal = 4.0;
    public static final double intakeGoal = 2.5;
    public static final double groundIntakeGoal = 0.0;

    // Simulation constants
    public static final double elevatorGearing = 100.0; // gear ratio
    public static final double carriageMass = 10.0; // in kg
    public static final double drumRadius = 0.2; // in meters
    public static final double minHeight = 0.0; // in meters
    public static final double maxHeight = 1.3; // in meters
    public static final double startingHeight = 0.0; // in meters
    public static final double simPeriod = 0.02;
}
