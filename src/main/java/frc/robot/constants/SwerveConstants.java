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

    public static final double translationKP = 5.0;
    public static final double translationKI = 0.0;
    public static final double translationKD = 0.0;
    public static final double rotationKP = 5.0;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.0;

    public static final double transKS = 0.03;
    public static final double rotKS = 0.03;
    public static final double transMin = 0.05;
    public static final double rotMin = 0.05;

    public static final PathConstraints constraints = new PathConstraints(
        MetersPerSecond.of(2),
        MetersPerSecondPerSecond.of(1.5),
        RadiansPerSecond.of(1.5*Math.PI),
        RadiansPerSecondPerSecond.of(Math.PI));

    public static final double coralExpirationTime = 5;

    public static final double transTol = 0.02; // in meters
    public static final double rotTol = Units.degreesToRadians(1); // in radians

    public static final double moveTimeout = 10; // seconds

    public static final double driveBackPower = 0.8;
    public static final double driveBackTime = 0.5; // seconds

    // Offset from the center of the robot to the arm pivot
    public static final Translation3d armTransOffset = new Translation3d(0.11, -0.18, 0.26);
}
