package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Contains constants related to controller inputs and bindings.
 */
public final class ControlConstants {
    // Controller port assignments
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final int sysIdControllerPort = 2;
    
    // Joystick deadbands
    public static final double driveDeadband = 0.05;
    public static final double operatorDeadband = 0.1;
    
    // Joystick scaling
    public static final double joystickExponent = 1.0; // Power to apply to joystick inputs
    
    // Mechanism control rates
    public static final double elevatorManualRate = 1.0 / 50.0; // Rate for manual elevator control
    public static final double armManualRate = 1.0 / 50.0; // Rate for manual arm control
    public static final double wristManualRate = 0.02; // Rate for manual wrist control
    
    // Wrist positions
    public static final double wristIntakePosition = Units.degreesToRadians(-90); // Position for intake
    
    // Auto command timing
    public static final double autoInitialWaitTime = 2.0; // Seconds to wait before starting auto
} 