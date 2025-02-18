package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.WristConstants;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.motorSystem.EnhancedEncoder;
import frc.robot.motorSystem.MotorSystem;
import frc.robot.motorSystem.ArmMechanismSim;
import frc.robot.other.RobotUtils;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SwerveSystemConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

/**
 * The Wrist subsystem controls the end effector of the robot arm.
 * It provides position control through a combination of PID and feedforward control,
 * taking into account its position relative to the main arm.
 * 
 * Features:
 * - Position control using ProfiledPIDController for smooth motion
 * - Gravity compensation and feedforward control for accurate positioning
 * - Coordination with main arm position for proper gravity compensation
 * - Simulation support using SingleJointedArmSim
 * - System identification capabilities for tuning
 * - Telemetry output for debugging and monitoring
 * 
 * The wrist's position is measured in radians, where:
 * - 0 radians = aligned with the arm
 * - Positive angles = downward from arm
 * - Negative angles = upward from arm
 * 
 * Note: The wrist's effective gravity vector changes based on the main arm's position,
 * which is accounted for in the feedforward calculations.
 */
public class Wrist extends SubsystemBase {
    
    /** Motor system that handles both the motor and encoder */
    private final MotorSystem motorSystem;
    
    /** PID controller for wrist position control */
    private final ProfiledPIDController controller;
    
    /** Feedforward controller for gravity compensation and dynamics */
    private final ArmFeedforward feedforward;
    
    /** Desired goal position (may be different from the true goal) */
    private double desiredGoal;

    /** Current goal position for the wrist in radians */
    private double goal;

    /** Simulation model for the wrist's physics */
    private final SingleJointedArmSim wristSim;

    /** Adapter to make SingleJointedArmSim compatible with MotorSystem */
    private final ArmMechanismSim mechanismSim;

    /** System identification routine for parameter tuning */
    private final SysIdRoutine sysIdRoutine;

    /** Reference to the main arm this wrist is attached to */
    private final Arm arm;
        
    /**
     * Constructor for the Wrist subsystem.
     * Initializes all control systems, motors, encoders, and simulation components.
     * 
     * @param arm The arm subsystem that this wrist is attached to. Used for
     *            coordinating positions and gravity compensation.
     */
    public Wrist(Arm arm) {
        this.arm = arm;

        // Create motor and encoder
        EnhancedTalonFX wristMotor = new EnhancedTalonFX(
            WristConstants.motorId,
            "rio",
            (2*Math.PI)/WristConstants.gearRatio,
            false,
            true  // Use brake mode for better position holding
        );
        EnhancedEncoder wristEncoder = new EnhancedEncoder(
            WristConstants.encoderId,
            (2*Math.PI)/WristConstants.encoderRatio,
            WristConstants.encoderOffset
        );
        
        // Create motor system
        motorSystem = new MotorSystem(List.of(wristMotor), wristEncoder);
        
        // Initialize PID controller
        controller = new ProfiledPIDController(
            WristConstants.kP,
            WristConstants.kI,
            WristConstants.kD,
            WristConstants.pidConstraints
        );
        controller.setTolerance(WristConstants.wristTolerance);
        
        // Initialize feedforward controller
        feedforward = new ArmFeedforward(
            WristConstants.kS,
            WristConstants.kG,
            WristConstants.kV
        );
        
        // Set initial goal position
        desiredGoal = SwerveSystemConstants.getDefaultState().wristPosition;

        // Initialize simulation components
        wristSim = new SingleJointedArmSim(
            WristConstants.motorSim,
            WristConstants.gearRatio,
            WristConstants.momentOfInertia,
            WristConstants.wristLength,
            WristConstants.minAngle,
            WristConstants.maxAngle,
            false,
            SwerveSystemConstants.getDefaultState().wristPosition
        );
        mechanismSim = new ArmMechanismSim(wristSim);

        // Initialize system identification routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).div(Seconds.of(1.0)),  // Slower ramp rate for wrist
                Volts.of(1),                         // Lower voltage to prevent oscillation
                null                                 // Use default timeout
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("wrist")
                        .voltage(getVolts())
                        .angularPosition(Radians.of(getMeasurement()+WristConstants.feedOffset))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()))
                        .angularAcceleration(RadiansPerSecondPerSecond.of(getAcceleration()));
                },
                this
            )
        );
    }

    /**
     * Get the current wrist position in radians.
     * 
     * @return The current wrist position in radians, where 0 is aligned with the arm,
     *         positive is down, and negative is up.
     */
    public double getMeasurement() {
        return motorSystem.getMeasurement();
    }
    
    /**
     * Set the goal position for the wrist, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians. Will be clamped between
     *                WristConstants.minAngle and WristConstants.maxAngle.
     */
    public void setGoal(double newGoal) {
        desiredGoal = RobotUtils.clamp(newGoal, WristConstants.minAngle, WristConstants.maxAngle);
    }

    /**
     * Get the current goal position.
     * 
     * @return The current goal position in radians.
     */
    public double getDesiredGoal() {
        return desiredGoal;
    }

    /**
     * Get the current goal position.
     * 
     * @return The current goal position in radians.
     */
    public double getGoal() {
        return goal;
    }

    /**
     * Set the voltage applied to the wrist motor.
     * 
     * @param v The voltage to apply to the motor.
     */
    public void setVoltage(Voltage v) {
        motorSystem.setVoltage(v);
    }

    /**
     * Get the current wrist velocity in radians per second.
     * 
     * @return The current angular velocity in radians per second.
     */
    public double getVelocity() {
        return motorSystem.getVelocity();
    }

    /**
     * Get the current voltage applied to the wrist motor.
     * 
     * @return The voltage currently being applied to the motor.
     */
    public Voltage getVolts() {
        return motorSystem.getVolts();
    }

    /**
     * Get the current wrist acceleration in radians per second squared.
     * 
     * @return The current angular acceleration in radians per second squared.
     */
    public double getAcceleration() {
        return motorSystem.getAcceleration();
    }

    /**
     * Check if the wrist is at its target position.
     * 
     * @return true if the wrist is within the tolerance of its goal position,
     *         false otherwise.
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Get the system identification routine for tuning.
     * 
     * @return The SysId routine configured for this wrist.
     */
    public SysIdRoutine getRoutine() {
        return sysIdRoutine;
    }

    /**
     * Determines if the arm's path will intersect the danger zone.
     * 
     * @param currentArmAngle Current angle of the arm in radians
     * @param goalArmAngle Goal angle of the arm in radians
     * @return true if the path intersects the danger zone
     */
    private boolean willArmPathIntersectDangerZone(double currentArmAngle, double goalArmAngle) {
        return (currentArmAngle >= WristConstants.armDangerMin && goalArmAngle <= WristConstants.armDangerMax) ||
            (goalArmAngle >= WristConstants.armDangerMin && currentArmAngle <= WristConstants.armDangerMax);
    }

    /**
     * Determines the safe wrist position based on arm's current and goal positions.
     * Retracts wrist if arm is moving through danger zone.
     */
    private double getSafeGoal(double armAngle, double armGoal) {
        if (willArmPathIntersectDangerZone(armAngle, armGoal)) {
            return WristConstants.minAngle;
        }
        return desiredGoal;
    }

    /**
     * Display telemetry data on SmartDashboard.
     * Outputs current position, goal, velocity, and error information
     * for debugging and monitoring.
     */
    public void telemetry() {
        SmartDashboard.putNumber("wrist/angle", getMeasurement());
        SmartDashboard.putNumber("wrist/degrees", Units.radiansToDegrees(getMeasurement()));
        SmartDashboard.putNumber("wrist/goal", getGoal());
        SmartDashboard.putNumber("wrist/setpoint", controller.getSetpoint().position);
        SmartDashboard.putNumber("wrist/setpointVelocity", controller.getSetpoint().velocity);
        SmartDashboard.putNumber("wrist/velocity", getVelocity());
        SmartDashboard.putNumber("wrist/error", controller.getPositionError());
        SmartDashboard.putData("wrist/controller", controller);
        motorSystem.log("wrist");
    }

    /**
     * Periodic method called every loop iteration.
     * Updates the wrist's position using PID and feedforward control,
     * taking into account the main arm's position for proper gravity compensation.
     */
    @Override
    public void periodic() {
        try {
            motorSystem.periodic();

            // Update telemetry
            telemetry();
        
            // Calculate PID control outputs with arm position compensation
            double armRotation = arm != null ? arm.getMeasurement() : 0;
            double wristRotation = getMeasurement();
            
            // Update goal based on arm's current position and goal
            goal = getSafeGoal(armRotation, arm.getGoal());
            
            // Calculate feedforward with combined arm and wrist angles
            double feed = feedforward.calculate(
                controller.getSetpoint().position + armRotation + WristConstants.feedOffset,
                controller.getSetpoint().velocity
            );
            double voltage = controller.calculate(wristRotation, goal) + feed;

            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));
        } catch (Exception e) {
            DataLogManager.log("Periodic error: " + RobotUtils.getError(e));
        }
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     * Updates the simulated wrist physics using the MotorSystem and ArmMechanismSim.
     */
    @Override
    public void simulationPeriodic() {
        motorSystem.simulationPeriodic(mechanismSim, GeneralConstants.simPeriod);
    }
}