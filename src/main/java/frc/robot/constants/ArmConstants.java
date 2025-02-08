package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.other.MotorTransform;

public class ArmConstants {
    // Motor ID
    public static final int motorId = Robot.isSimulation()?22:2;

    // Encoder transform
    public static final double armOffset = -Math.PI/2;
    public static final double gearRatio = 31.5;
    public static final MotorTransform transform = new MotorTransform((2*Math.PI)/gearRatio, armOffset);
    
    // PID constants
    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0.1;

    // Feedforward constants
    public static final double kS = 0.0;
    public static final double kG = 1.83;
    public static final double kV = 0.62;
    public static final double kA = 0.09;

    // Arm tolerance
    public static final double armTolerance = 0.1;
    
    // Simulation constants
    public static final double minAngle = Units.degreesToRadians(0);
    public static final double maxAngle = Units.degreesToRadians(135);
    public static final double momentOfInertia = 1.2;
    public static final double armLength = 0.594;
    public static final DCMotor motorSim = DCMotor.getKrakenX60(1);
}
