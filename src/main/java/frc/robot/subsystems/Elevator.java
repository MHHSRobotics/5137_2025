package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.other.RobotUtils;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.motorSystem.EnhancedEncoder;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.motorSystem.MotorSystem;
import frc.robot.motorSystem.ElevatorMechanismSim;

import java.util.List;

import com.ctre.phoenix6.controls.Follower;

/**
 * The Elevator subsystem controls a two-motor elevator mechanism.
 * It provides position control through a combination of PID and feedforward control.
 * 
 * Features:
 * - Position control using PIDController
 * - Gravity compensation and feedforward control for accurate positioning
 * - Dual-motor synchronization for smooth motion
 * - Simulation support using ElevatorSim
 * - System identification capabilities for tuning
 * - Telemetry output for debugging and monitoring
 * 
 * The elevator's position is measured in meters, where:
 * - 0 meters = fully retracted
 * - Positive values = extension upward
 * - Maximum height defined by ElevatorConstants.maxHeight
 */
public class Elevator extends SubsystemBase {

    /** Motor system that handles both motors and encoder */
    private final MotorSystem motorSystem;
    
    /** PID controller for elevator position control */
    private final PIDController controller;
    
    /** Feedforward controller for gravity compensation and dynamics */
    private final ElevatorFeedforward feedforward;
    
    /** Current goal position for the elevator in meters */
    private double goal;

    /** Simulation model for the elevator's physics */
    private final ElevatorSim elevatorSim;

    /** Adapter to make ElevatorSim compatible with MotorSystem */
    private final ElevatorMechanismSim mechanismSim;

    /** System identification routine for parameter tuning */
    private final SysIdRoutine sysIdRoutine;

    /**
     * Constructor for the Elevator subsystem.
     * Initializes all control systems, motors, encoders, and simulation components.
     */
    public Elevator() {
        // Create motors and encoder
        EnhancedTalonFX leftMotor = new EnhancedTalonFX(
            ElevatorConstants.leftMotorId,
            "rio",
            ElevatorConstants.drumRadius * 2 * Math.PI / ElevatorConstants.gearRatio,
            false,
            true  // Use brake mode for better position holding
        );
        EnhancedTalonFX rightMotor = new EnhancedTalonFX(
            ElevatorConstants.rightMotorId,
            "rio",
            ElevatorConstants.drumRadius * 2 * Math.PI / ElevatorConstants.gearRatio,
            true,
            true  // Use brake mode for better position holding
        );
        EnhancedEncoder elevatorEncoder = new EnhancedEncoder(
            ElevatorConstants.encoderId,
            ElevatorConstants.drumRadius * 2 * Math.PI / ElevatorConstants.encoderRatio,
            ElevatorConstants.encoderOffset
        );

        // Create motor system with both motors
        motorSystem = new MotorSystem(List.of(leftMotor, rightMotor), elevatorEncoder);
        
        // Initialize PID controller
        controller = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
        );
        controller.setTolerance(ElevatorConstants.elevatorTolerance);
        
        // Initialize feedforward controller
        feedforward = new ElevatorFeedforward(
            ElevatorConstants.kS,
            ElevatorConstants.kG,
            ElevatorConstants.kV,
            ElevatorConstants.kA
        );
        
        // Set initial goal position
        goal = SwerveSystemConstants.getDefaultState().elevatorPosition;

        // Initialize simulation components
        elevatorSim = new ElevatorSim(
            ElevatorConstants.motorSim,
            ElevatorConstants.gearRatio,
            ElevatorConstants.carriageMass,
            ElevatorConstants.drumRadius,
            ElevatorConstants.minHeight,
            ElevatorConstants.maxHeight,
            true,
            SwerveSystemConstants.getDefaultState().elevatorPosition
        );
        mechanismSim = new ElevatorMechanismSim(elevatorSim);

        // Initialize system identification routine
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                null         // Use default timeout (10 s)
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("elevator")
                        .voltage(getVolts())
                        .linearPosition(Meters.of(getMeasurement()))
                        .linearVelocity(MetersPerSecond.of(getVelocity()));
                },
                this
            )
        );
    }

    /**
     * Get the current goal position of the elevator.
     * 
     * @return The current goal position in meters.
     */
    public double getGoal() {
        return goal;
    }

    /**
     * Set the goal position for the elevator.
     * 
     * @param newGoal The desired goal position in meters. Will be clamped between
     *                ElevatorConstants.minHeight and ElevatorConstants.maxHeight.
     */
    public void setGoal(double newGoal) {
        goal = RobotUtils.clamp(newGoal, ElevatorConstants.minHeight, ElevatorConstants.maxHeight);
    }

    /**
     * Get the current elevator position.
     * 
     * @return The current position in meters, where 0 is fully retracted
     *         and positive values indicate upward extension.
     */
    public double getMeasurement() {
        return motorSystem.getMeasurement();
    }

    /**
     * Get the current elevator velocity.
     * 
     * @return The current velocity in meters per second.
     */
    public double getVelocity() {
        return motorSystem.getVelocity();
    }

    /**
     * Get the current elevator acceleration.
     * 
     * @return The current acceleration in meters per second squared.
     */
    public double getAcceleration() {
        return motorSystem.getAcceleration();
    }

    /**
     * Check if the elevator is at its target position.
     * 
     * @return true if the elevator is within the tolerance of its goal position,
     *         false otherwise.
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Get the current voltage applied to the elevator motors.
     * 
     * @return The average voltage currently being applied to the motors.
     */
    public Voltage getVolts() {
        return motorSystem.getVolts();
    }

    /**
     * Set the voltage to apply to the elevator motors.
     * 
     * @param v The voltage to apply to both motors.
     */
    public void setVoltage(Voltage v) {
        motorSystem.setVoltage(v);
    }

    /**
     * Get the system identification routine for tuning.
     * 
     * @return The SysId routine configured for this elevator.
     */
    public SysIdRoutine getRoutine() {
        return sysIdRoutine;
    }

    /**
     * Display telemetry data on SmartDashboard.
     * Outputs current position, goal, velocity, and error information
     * for debugging and monitoring.
     */
    private void telemetry() {
        SmartDashboard.putNumber("elevator/height", getMeasurement());
        SmartDashboard.putNumber("elevator/goal", getGoal());
        SmartDashboard.putNumber("elevator/velocity", getVelocity());
        SmartDashboard.putNumber("elevator/error", controller.getError());
        SmartDashboard.putData("elevator/controller", controller);
        motorSystem.log("elevator");
    }

    /**
     * Periodic method called every loop iteration.
     * Updates the elevator's position using PID and feedforward control.
     */
    @Override
    public void periodic() {
        try {
            motorSystem.periodic();

            // Update telemetry
            telemetry();

            // Calculate feedforward and PID control outputs
            double feed = feedforward.calculate(0); // Constant gravity compensation
            double voltage = controller.calculate(getMeasurement(), goal) + feed;
            
            // Apply the calculated voltage to the motors
            setVoltage(Volts.of(voltage));
        } catch (Exception e) {
            DataLogManager.log("Periodic error: " + RobotUtils.getError(e));
        }
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     * Updates the simulated elevator physics using the MotorSystem and ElevatorMechanismSim.
     */
    @Override
    public void simulationPeriodic() {
        motorSystem.simulationPeriodic(mechanismSim, GeneralConstants.simPeriod);
    }
}