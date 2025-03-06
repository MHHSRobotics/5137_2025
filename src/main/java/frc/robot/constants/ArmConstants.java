package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class ArmConstants {
    // Motor ID
    public static final int motorId = Robot.isSimulation()?22:2;
    public static final double gearRatio = 700/9.;
    public static final double armOffset = 0;

    // Encoder ID
    public static final int encoderId = 1;
    public static final double encoderRatio = -28/9.;
    public static final double encoderOffset = 1.625;

    public static final double feedOffset = Units.degreesToRadians(90);

    // Feedforward constants
    public static final double kS = Robot.isSimulation()?0.0:0.135;
    public static final double kG = Robot.isSimulation()?0.0:1.5;
    public static final double kV = 0;
    public static final double kA = 0;

    // Motion Profile constants
    //public static final Constraints pidConstraints=new Constraints(Units.degreesToRadians(180),Units.degreesToRadians(360));

    // Arm tolerance
    public static final double armTolerance = Units.degreesToRadians(2);

    // Hang Position
    public static final double hangPosition = Units.degreesToRadians(-30);
    
    // Simulation constants
    public static final double minAngle = Units.degreesToRadians(-135);
    public static final double maxAngle = Units.degreesToRadians(35);
    public static final double momentOfInertia = 4.8944;
    public static final double armLength = 0.594;
    public static final DCMotor motorSim = DCMotor.getKrakenX60Foc(1);

    // 1/posWeight^2 = how much to value position error
    public static final double posWeight = 0.1;

    // 1/velWeight^2 = how much to value velocity error
    public static final double velWeight = 3;

    // 1/volWeight^2 = how much to value voltage error
    public static final double volWeight = 3;

    public static final double maxGoalVelocity = 10;
    public static final double maxGoalAcceleration = 2;
    // minimizes int_0^inf ((pos-goal)/posWeight)^2 + (vel/velWeight)^2 + (vol/volWeight)^2 dt
}
