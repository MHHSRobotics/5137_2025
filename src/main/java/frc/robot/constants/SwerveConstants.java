package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double translationalDeadband = 0.1;
    public static final double rotationalDeadband = 0.1;
    public static final double odometryFrequency = 20; // ms
    public static final double simLoopPeriod = 0.005; // seconds

    public static final double translationKP = 1.0;
    public static final double translationKI = 0.0;
    public static final double translationKD = 0.0;
    public static final double rotationKP = 1.0;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.0;

    public static final PathConstraints constraints = new PathConstraints(
        MetersPerSecond.of(0.5),
        MetersPerSecondPerSecond.of(0.5),
        RadiansPerSecond.of(1.5*Math.PI),
        RadiansPerSecondPerSecond.of(Math.PI));

    public static final double coralExpirationTime = 5;

    public static final double transTol = 0.1; // in meters
    public static final double rotTol = Units.degreesToRadians(10); // in radians

    public static final double moveTimeout = 10; // seconds

    public static final double driveBackPower = 0.8;
    public static final double driveBackTime = 0.5; // seconds
}
