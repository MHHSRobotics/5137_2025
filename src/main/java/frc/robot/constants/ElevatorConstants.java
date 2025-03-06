package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Robot;

public class ElevatorConstants {
    // Motor IDs
    public static final int leftMotorId = Robot.isSimulation()?20:0;
    public static final int rightMotorId = Robot.isSimulation()?21:1;

    // Encoder ID
    public static final int encoderId = 2;
    public static final double encoderRatio = 0.5;  // Negate multiplier since encoder was inverted before
    public static final double encoderOffset = -0.0535;

    // PID constants
    public static final double kP = 20;
    public static final double kI = 0;
    public static final double kD = 0.5;

    // Feedforward constants
    public static final double kS = Robot.isSimulation() ? 0.0 : 0.0;
    public static final double kG = 0.44;
    public static final double kV = 0.88;
    public static final double kA = 0.02;

    // Elevator tolerance
    public static final double elevatorTolerance = 0.05;

    // Simulation constants
    public static final double gearRatio = 7.5; // gear ratio
    public static final double carriageMass = 13.0; // in kg
    public static final double drumRadius = 0.022; // in meters
    public static final double minHeight = 0; // in meters
    public static final double maxHeight = 1.12; // in meters
    public static final DCMotor motorSim = DCMotor.getFalcon500Foc(2);

    // Motion Profile constants
    public static final Constraints pidConstraints= new Constraints(2.5,8); // m/s, m/s^2
}
