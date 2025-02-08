package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ArmSystemConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.other.RobotUtils;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * The Arm subsystem controls the robotic arm using a TalonFX motor controller.
 * It includes PID control, feedforward control, and simulation capabilities.
 */
public class Arm extends SubsystemBase {
    
    // Motor controller for the arm
    private TalonFX armMotor = new TalonFX(ArmConstants.motorId, "rio");
    
    // PID controller for arm position control
    private PIDController controller = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    
    // Feedforward controller for arm dynamics
    private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);
    
    // Goal position for the arm in radians
    private double goal = ArmSystemConstants.defaultState.armPosition;
    
    // Simulation model for the arm
    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        ArmConstants.motorSim, 
        ArmConstants.gearRatio, 
        ArmConstants.momentOfInertia, 
        ArmConstants.armLength, 
        ArmConstants.minAngle, 
        ArmConstants.maxAngle, 
        true, 
        ArmSystemConstants.defaultState.armPosition
    );

    // System Identification routine for characterizing the arm
    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("arm")
                        .voltage(Volts.of(armMotor.get() * RobotController.getBatteryVoltage()))
                        .angularPosition(Radians.of(getMeasurement()))
                        .angularVelocity(RadiansPerSecond.of(getVelocity()));
                },
                this
            )
        );
    
    // Simulation state for the motor
    private TalonFXSimState armMotorSim = armMotor.getSimState();
        
    private StringLogEntry log;
    
    /**
     * Constructor for the Arm subsystem.
     */
    public Arm(StringLogEntry log) {
        // Configure the motor to coast when neutral
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        armMotor.getConfigurator().apply(currentConfigs);

        // Set the tolerance for the PID controller
        controller.setTolerance(ArmConstants.armTolerance);
        
        // Display the PID controller on SmartDashboard for tuning
        SmartDashboard.putData("arm/controller", controller);

        this.log=log;
    }

    /**
     * Get the current arm position in radians.
     * 
     * @return The current arm position in radians.
     */
    public double getMeasurement() {
        return ArmConstants.transform.transformPos(armMotor.getPosition().getValueAsDouble());
    }
    
    /**
     * Set the goal position for the arm, clamping it within the allowed range.
     * 
     * @param newGoal The desired goal position in radians.
     */
    public void setGoal(double newGoal) {
        goal = RobotUtils.clamp(newGoal,ArmConstants.minAngle,ArmConstants.maxAngle);
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
     * @param v The voltage to apply.
     */
    public void setVoltage(Voltage v) {
        armMotor.setVoltage(v.magnitude());
    }

    /**
     * Get the current arm velocity in radians per second.
     * 
     * @return The current arm velocity in radians per second.
     */
    public double getVelocity() {
        return ArmConstants.transform.transformVel(armMotor.getPosition().getValueAsDouble());
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public SysIdRoutine getRoutine(){
        return sysIdRoutine;
    }

    /**
     * Display telemetry data on SmartDashboard.
     */
    public void telemetry() {
        SmartDashboard.putNumber("arm/angle",getMeasurement());
        SmartDashboard.putNumber("arm/goal",getGoal());
        SmartDashboard.putNumber("arm/velocity",getVelocity());
        SmartDashboard.putNumber("arm/error",controller.getError());
        SmartDashboard.putNumber("arm/motor/rawAngle",armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/temp",armMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/fault",armMotor.getFaultField().asSupplier().get());
        SmartDashboard.putNumber("arm/motor/current",armMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/voltage",armMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("arm/motor/supplyVoltage",armMotor.getSupplyVoltage().getValueAsDouble());
    }

    /**
     * Periodic method called every loop iteration.
     */
    @Override
    public void periodic() {
        try{
            // Update telemetry
            telemetry();
            
            // Calculate feedforward and PID control outputs
            double extra = feedforward.calculate(getMeasurement(), getVelocity());
            double voltage = controller.calculate(getMeasurement(), goal) + extra;
            
            // Apply the calculated voltage to the motor
            setVoltage(Volts.of(voltage));
        }catch(Exception e){
            log.append("Periodic error: " + RobotUtils.getError(e));
        }

        System.out.println("Goal: " + Math.toDegrees(getGoal()) + " | Measurement: " + Math.toDegrees(getMeasurement()));
    }

    /**
     * Simulation periodic method called every loop iteration in simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Update the motor simulation state with the current battery voltage
        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        // Get the current motor input voltage and update the arm simulation
        double armInput = armMotorSim.getMotorVoltage();
        armSim.setInputVoltage(armInput);
        armSim.update(GeneralConstants.simPeriod);
        
        // Update the motor simulation state with the new arm position and velocity
        armMotorSim.setRawRotorPosition(ArmConstants.transform.transformPosInv(armSim.getAngleRads()));
        armMotorSim.setRotorVelocity(ArmConstants.transform.transformVelInv(armSim.getVelocityRadPerSec()));
        
        // Update the RoboRIO simulation state with the new battery voltage
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }
}