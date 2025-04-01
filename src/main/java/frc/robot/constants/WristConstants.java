package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class WristConstants {
    // Motor ID
    public static final int motorId = 23;
    public static final double gearRatio = 16.0;

    // Encoder ID
    public static final int encoderId = 0;
    public static final double encoderRatio = -1.0;  // Negate multiplier since encoder was inverted before
    public static final double encoderOffset = 7.05;

    // Limits
    public static final double minAngle = Units.degreesToRadians(-135);
    public static final double maxAngle = Units.degreesToRadians(90);

    public static final double feedOffset = Units.degreesToRadians(90);

    // PID constants
    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = 0.2;    
    
    // LQR constants
    // 1/posWeight^2 = how much to value position error
    public static final double posWeight = 0.02;
    // 1/velWeight^2 = how much to value velocity error
    public static final double velWeight = 0.1;
    // 1/volWeight^2 = how much to value voltage error
    public static final double volWeight = 0.1;

    // Feedforward constants
    public static final double kS = Robot.isSimulation()?0.0:0.1;
    public static final double kG = Robot.isSimulation()?0.0:0.47;
    public static final double kV = 0.25757;
    public static final double kA = 0.050664;

    // Motion Profile constants
    public static final Constraints pidConstraints=new Constraints(Units.degreesToRadians(360),Units.degreesToRadians(360));

    // Tolerance
    public static final double wristTolerance = Units.degreesToRadians(1);

    // Simulation constants
    public static final double momentOfInertia = 0.5219;
    public static final double wristLength = 0.1524;
    public static final DCMotor motorSim = DCMotor.getKrakenX60Foc(1);

    public static final double armDangerMin = Units.degreesToRadians(-110);
    public static final double armDangerMax = Units.degreesToRadians(-45);

    // Trapezoid Profile constants
    public static final double maxGoalVelocity = 1;  // radians per second
    public static final double maxGoalAcceleration = 1;  // radians per second squared
}