package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.motorSystem.EnhancedEncoder;
import frc.robot.motorSystem.MotorSystem;
import frc.robot.motorSystem.ArmMechanismSim;
import frc.robot.constants.GeneralConstants;
import frc.robot.other.RobotUtils;
import frc.robot.positions.RobotPositions;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;

/**
 * The Arm subsystem controls a single-jointed robotic arm using a TalonFX motor controller.
 * It implements advanced control through a combination of Linear Quadratic Regulator (LQR),
 * feedforward control, and slew rate limiting.
 * 
 * Features:
 * - State-space control using Linear Quadratic Regulator (LQR) for optimal control
 * - Gravity compensation and feedforward control for accurate positioning
 * - Slew rate limiting to prevent aggressive voltage changes
 * - Simulation support using SingleJointedArmSim
 * - System identification capabilities for tuning
 * - Telemetry output for debugging and monitoring
 * - Voltage and position safety limits
 * 
 * The arm's position is measured in radians, where:
 * - 0 radians = vertical position
 * - Positive angles = forward from vertical
 * - Negative angles = backward from vertical
 * 
 * Control Architecture:
 * - LQR provides optimal state feedback control based on position and velocity
 * - Feedforward compensates for gravity and system dynamics
 * - Slew rate limiter smooths voltage transitions to prevent jerky motion
 */
public class Arm extends SubsystemBase {
    
    /** Motor system that handles both the motor and encoder */
    private final MotorSystem motorSystem;
    
    /** Feedforward controller for gravity compensation and dynamics */
    private final ArmFeedforward feedforward;
    
    /** Current goal position for the arm in radians */
    private double goal;

    /** Simulation model for the arm's physics */
    private final SingleJointedArmSim armSim;

    /** Adapter to make SingleJointedArmSim compatible with MotorSystem */
    private final ArmMechanismSim mechanismSim;

    // linear quadratic regulator
    // first N2 = dimension of state space of the arm, position and velocity
    // N1 = number of inputs to the arm, voltage
    // second N2 = number of accessible outputs, position and velocity
    private LinearQuadraticRegulator<N2,N1,N2> lqr;

    private final TrapezoidProfile goalProfile;
    private TrapezoidProfile.State goalState;
        
    /**
     * Constructor for the Arm subsystem.
     * Initializes all control systems, motors, encoders, and simulation components.
     * 
     * Sets up:
     * - Motor and encoder with appropriate gear ratios
     * - Linear system plant model for the arm
     * - LQR controller with configured weights for position, velocity and voltage
     * - Slew rate limiter for smooth voltage transitions
     * - Feedforward controller for gravity compensation
     * - Simulation components for testing
     * - System identification routine for parameter tuning
     */
    public Arm() {
        // Create motor and encoder
        EnhancedTalonFX armMotor = new EnhancedTalonFX(
            ArmConstants.motorId, 
            "rio", 
            (2*Math.PI)/ArmConstants.gearRatio, 
            false, 
            true  // Use brake mode for better position holding
        );
        EnhancedEncoder armEncoder = new EnhancedEncoder(
            ArmConstants.encoderId, 
            (2*Math.PI)/ArmConstants.encoderRatio,
            ArmConstants.encoderOffset
        );

        // Create the plant, simulates the arm movement
        LinearSystem<N2,N1,N2> plant=LinearSystemId.createSingleJointedArmSystem(ArmConstants.motorSim, ArmConstants.momentOfInertia, ArmConstants.gearRatio);

        // Create motor system
        motorSystem = new MotorSystem(List.of(armMotor), armEncoder);
        
        // Initialize feedforward controller
        feedforward = new ArmFeedforward(
            ArmConstants.kS, 
            ArmConstants.kG, 
            ArmConstants.kV
        );
        
        // Set initial goal position
        goal = RobotPositions.defaultState.armPosition;

        // Initialize simulation components
        armSim = new SingleJointedArmSim(
            ArmConstants.motorSim, 
            ArmConstants.gearRatio, 
            ArmConstants.momentOfInertia,
            ArmConstants.armLength,
            ArmConstants.minAngle,
            ArmConstants.maxAngle,
            false,
            RobotPositions.defaultState.armPosition
        );
        mechanismSim = new ArmMechanismSim(armSim);

        // Initializes the linear-quadratic regulator
        lqr=new LinearQuadraticRegulator<N2,N1,N2>(plant,VecBuilder.fill(ArmConstants.posWeight,ArmConstants.velWeight),VecBuilder.fill(ArmConstants.volWeight),GeneralConstants.simPeriod);

        goalProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ArmConstants.maxGoalVelocity,
                ArmConstants.maxGoalAcceleration
            )
        );
        goalState = new TrapezoidProfile.State(getMeasurement(), 0);
    }

    /**
     * Get the current arm position in radians.
     * 
     * @return The current arm position in radians, where 0 is vertical,
     *         positive values are forward, and negative values are backward.
     */
    public double getMeasurement() {
        return motorSystem.getMeasurement();
    }
    
    /**
     * Set the goal position for the arm, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians. Will be clamped between
     *                ArmConstants.minAngle and ArmConstants.maxAngle.
     */
    public void setGoal(double newGoal) {
        goal = RobotUtils.clamp(newGoal, ArmConstants.minAngle, ArmConstants.maxAngle);
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
     * Set the voltage applied to the arm motor.
     * 
     * @param v The voltage to apply to the motor.
     */
    public void setVoltage(Voltage v) {
        motorSystem.setVoltage(v);
    }

    /**
     * Gets the current voltage applied to the arm motor.
     * 
     * @return The voltage currently being applied to the motor.
     */
    public Voltage getVolts() {
        return motorSystem.getVolts();
    }

    /**
     * Get the current arm velocity in radians per second.
     * 
     * @return The current angular velocity in radians per second.
     */
    public double getVelocity() {
        return motorSystem.getVelocity();
    }

    /**
     * Get the current arm acceleration in radians per second squared.
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
        return getError()<ArmConstants.armTolerance;
    }

    /**
     * Display telemetry data on SmartDashboard.
     * Outputs current position, goal, velocity, and error information
     * for debugging and monitoring.
     */
    public void telemetry() {
        SmartDashboard.putNumber("arm/angle", getMeasurement());
        SmartDashboard.putNumber("arm/degrees", Units.radiansToDegrees(getMeasurement()));
        SmartDashboard.putNumber("arm/goal", getGoal());
        SmartDashboard.putNumber("arm/velocity", getVelocity());
        SmartDashboard.putNumber("arm/error", getError());
        SmartDashboard.putNumber("arm/profileError", getProfileError());
        SmartDashboard.putNumber("arm/profileAngle",goalState.position);
        SmartDashboard.putNumber("arm/profileVelocity",goalState.velocity);
        motorSystem.log("arm");
    }

    /**
     * Periodic method called every loop iteration.
     * Updates the arm's position using LQR state feedback control and feedforward.
     * The control flow:
     * 1. Updates motor system and gets current state (position/velocity)
     * 2. Updates telemetry data
     * 3. Calculates LQR output based on current state and goal
     * 4. Applies feedforward compensation
     * 5. Limits voltage rate of change
     * 6. Applies final voltage command to motor
     */

    @Override
    public void periodic() {
        try {
            motorSystem.periodic();
            double measurement=getMeasurement();
            double velocity=getVelocity();

            // Update telemetry
            telemetry();

             // Generate smooth goal trajectory
            goalState = goalProfile.calculate(
                GeneralConstants.simPeriod,
                goalState,
                new TrapezoidProfile.State(goal, 0)  // Target state
            );
            
            // Use profile state as the goal for LQR
            double voltage = lqr.calculate(
                VecBuilder.fill(measurement, velocity),
                VecBuilder.fill(goalState.position, goalState.velocity)
            ).get(0,0);

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
     * Updates the simulated arm physics using the MotorSystem and ArmMechanismSim.
     */
    @Override
    public void simulationPeriodic() {
        motorSystem.simulationPeriodic(mechanismSim, GeneralConstants.simPeriod);
    }
}