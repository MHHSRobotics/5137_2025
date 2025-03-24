package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.motorSystem.EnhancedEncoder;
import frc.robot.motorSystem.MotorSystem;
import frc.robot.motorSystem.ArmMechanismSim;
import frc.robot.other.RobotUtils;
import frc.robot.positions.RobotPositions;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.GeneralConstants;

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

    /** Current goal position for the wrist in radians */
    private double goal;

    /** Simulation model for the wrist's physics */
    private final SingleJointedArmSim wristSim;

    /** Adapter to make SingleJointedArmSim compatible with MotorSystem */
    private final ArmMechanismSim mechanismSim;

    // Linear quadratic regulator
    //private LinearQuadraticRegulator<N2,N1,N2> lqr;

    private final TrapezoidProfile goalProfile;
    private TrapezoidProfile.State goalState;
        
    /**
     * Constructor for the Wrist subsystem.
     * Initializes all control systems, motors, encoders, and simulation components.
     * 
     * @param arm The arm subsystem that this wrist is attached to. Used for
     *            coordinating positions and gravity compensation.
     */
    public Wrist() {
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
        
        // Create the plant, simulates the wrist movement
        /*LinearSystem<N2,N1,N2> plant = LinearSystemId.createSingleJointedArmSystem(
            WristConstants.motorSim, 
            WristConstants.momentOfInertia*1/3, 
            WristConstants.gearRatio
        );*/

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

        goal = RobotPositions.defaultState.wristPosition;

        
        // Initialize feedforward controller
        feedforward = new ArmFeedforward(
            WristConstants.kS,
            WristConstants.kG,
            WristConstants.kV
        );

        // Initialize simulation components
        wristSim = new SingleJointedArmSim(
            WristConstants.motorSim,
            WristConstants.gearRatio,
            WristConstants.momentOfInertia,
            WristConstants.wristLength,
            WristConstants.minAngle,
            WristConstants.maxAngle,
            false,
            RobotPositions.defaultState.wristPosition
        );
        mechanismSim = new ArmMechanismSim(wristSim);

        // Initialize LQR controller
        /*lqr = new LinearQuadraticRegulator<N2,N1,N2>(
            plant,
            VecBuilder.fill(WristConstants.posWeight, WristConstants.velWeight),  // State cost
            VecBuilder.fill(WristConstants.volWeight),  // Input cost
            GeneralConstants.simPeriod
        );*/

        // Initialize goal profiler
        goalProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                WristConstants.maxGoalVelocity,
                WristConstants.maxGoalAcceleration
            )
        );
        goalState = new TrapezoidProfile.State(getMeasurement(), 0);
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
        goal = RobotUtils.clamp(newGoal, WristConstants.minAngle, WristConstants.maxAngle);
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

    public double getError(){
        return Math.abs(getMeasurement()-getGoal());
    }

    public double getProfileError(){
        return Math.abs(getMeasurement()-goalState.position);
    }

    /**
     * Check if the arm is at its target position.
     * 
     * @return true if the arm is within the tolerance of its goal position,
     *         false otherwise.
     */
    public boolean atSetpoint() {
        return getError()<WristConstants.wristTolerance*10; // Wrist does not reach setpoint -- greater tolerance than PID
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
        SmartDashboard.putNumber("wrist/velocity", getVelocity());
        SmartDashboard.putNumber("wrist/error", getError());
        SmartDashboard.putNumber("wrist/profileError", getProfileError());
        SmartDashboard.putNumber("wrist/profileAngle",goalState.position);
        SmartDashboard.putNumber("wrist/profileVelocity",goalState.velocity);
        SmartDashboard.putBoolean("wrist/atTarget", atSetpoint());
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

            double measurement = getMeasurement();
            @SuppressWarnings("unused")
            double velocity = getVelocity();

            // Update telemetry
            telemetry();
            
             // Generate smooth goal trajectory
             goalState = goalProfile.calculate(
                GeneralConstants.simPeriod,
                goalState,
                new TrapezoidProfile.State(goal, 0)  // Target state
            );

            goal = getGoal();
            
            // Use profile state as the goal for LQR
            /*double voltage = lqr.calculate(
                VecBuilder.fill(measurement, velocity),
                VecBuilder.fill(goalState.position, goalState.velocity)
            ).get(0,0);*/

            double voltage = controller.calculate(measurement, goal);

            // Calculate feedforward and PID control outputs
            voltage+=feedforward.calculate(
                goal + ArmConstants.feedOffset, // Offset so that 0 = horizontal
                0
            );

            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));
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
        motorSystem.simulationPeriodic(mechanismSim, GeneralConstants.simPeriod);
    }
}