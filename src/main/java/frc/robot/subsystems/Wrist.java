package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;
import frc.robot.motorSystem.MechanismSim;
import frc.robot.motorSystem.ArmMechanismSim;
import frc.robot.motorSystem.AugmentedMotorSystem;
import frc.robot.motorSystem.BetterTalonFX;
import frc.robot.motorSystem.SyncedEncoder;
import frc.robot.other.RobotUtils;
import frc.robot.positions.RobotPositions;
import frc.robot.constants.GeneralConstants;

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
    //private final MotorSystem motorSystem;

    private final AugmentedMotorSystem motorSystem;

    /** Simulation model for the wrist's physics */
    private final MechanismSim wristSim;
        
    /**
     * Constructor for the Wrist subsystem.
     * Initializes all control systems, motors, encoders, and simulation components.
     * 
     * @param arm The arm subsystem that this wrist is attached to. Used for
     *            coordinating positions and gravity compensation.
     */
    public Wrist() {
        // Create motor and encoder
        BetterTalonFX motor = new BetterTalonFX(WristConstants.motorId, "rio")
            .withMotorOutputConfigs(false, true)
            .withRatioConfigs(WristConstants.gearRatio)
            .withPIDFConfigs(WristConstants.constants)
            .withMotionMagicConfigs(Units.degreesToRadians(90), Units.degreesToRadians(90));
        

        SyncedEncoder encoder = new SyncedEncoder(WristConstants.encoderId, WristConstants.encoderRatio, WristConstants.encoderOffset);

        // Create motor system
        motorSystem = new AugmentedMotorSystem(List.of(motor), encoder);

        // Cretae wrist simulation
        wristSim = new ArmMechanismSim(
            WristConstants.motorSim,
            WristConstants.gearRatio,
            WristConstants.momentOfInertia,
            WristConstants.wristLength,
            WristConstants.minAngle,
            WristConstants.maxAngle,
            false,
            RobotPositions.defaultState.wristPosition
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
        motorSystem.setTargetPosition(newGoal);
    }

    /**
     * Get the current goal position.
     * 
     * @return The current goal position in radians.
     */
    public double getGoal() {
        return motorSystem.getTargetPosition();
    }

    public double getError(){
        return motorSystem.getError();
    }

    /**
     * Check if the arm is at its target position.
     * 
     * @return true if the arm is within the tolerance of its goal position,
     *         false otherwise.
     */
    public boolean atSetpoint() {
        return getError()<WristConstants.wristTolerance; // Wrist does not reach setpoint -- greater tolerance than PID
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
            motorSystem.log("wrist");
            SmartDashboard.putBoolean("wrist/atSetpoint", atSetpoint());
        } catch (RuntimeException e) {
            DataLogManager.log("Periodic error: " + RobotUtils.processError(e));
        }
    }
        
    /**
     * Simulation periodic method called every loop iteration in simulation.
     * Updates the simulated wrist physics using the MotorSystem and ArmMechanismSim.
     */
    @Override
    public void simulationPeriodic() {
        motorSystem.simulationPeriodic(wristSim, GeneralConstants.simPeriod);
    }
}