package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double translationalDeadband = 0.1;
    public static final double rotationalDeadband = 0.1;
    public static final double odometryFrequency = 20; // ms
    public static final double simLoopPeriod = 0.005; // seconds

    public static final double translationKP = 4.0;
    public static final double translationKI = 0.0;
    public static final double translationKD = 0.5;
    public static final double rotationKP = 2.0;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.5;

    public static final PathConstraints constraints = new PathConstraints(
        MetersPerSecond.of(1),
        MetersPerSecondPerSecond.of(1.0),
        RadiansPerSecond.of(1.5*Math.PI),
        RadiansPerSecondPerSecond.of(Math.PI));

    public static final double coralExpirationTime = 5;

    public static final double transTol = 0.2; // in meters
    public static final double rotTol = Units.degreesToRadians(10); // in radians

    public static final double moveTimeout = 10; // seconds

    public static final double driveBackPower = 0.8;
    public static final double driveBackTime = 0.5; // seconds
    
    // Constants moved from RobotPositions
    
    // Timeout for movement commands
    public static final double timeout = 3; // seconds
    
    // Weight for rotation vs translation when finding closest state
    public static final double rotationWeight = 1.0; // meters per radian

    // Offset from the center of the robot to the arm pivot
    public static final Translation3d armTransOffset = new Translation3d(0.11, -0.18, 0.26);
    
    // Distance for intake operations
    public static final double intakeDistance = 1;
}
