package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private final DutyCycleOut request = new DutyCycleOut(0.0);
    private PIDController controller;
    private ElevatorFeedforward feedforward;
    private double goal = 0;
    private boolean manualControl;
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance pos = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  public final SysIdRoutine sysIdRoutine = 
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              this::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("elevator")
                    .voltage(
                        appliedVoltage.mut_replace(
                            leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(pos.mut_replace(getMeasurement(), Meters))
                    .linearVelocity(
                        velocity.mut_replace(getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
    public Elevator(){
        controller = new PIDController(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD);
        feedforward = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);
        leftMotor = new TalonFX(ElevatorConstants.leftMotorId, "rhino");
        rightMotor = new TalonFX(ElevatorConstants.rightMotorId, "rhino");

        var currentConfigs = new MotorOutputConfigs();

        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(currentConfigs);

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(currentConfigs);

        controller.setTolerance(ElevatorConstants.elevatorTol);

        manualControl = false;

        Shuffleboard.getTab("PID Controller").add(controller);
    }

    public double getGoal(){
        return goal;
    }

    public void setGoal(double goal){
        this.goal = goal;
    }

    public void setSpeed(double speed){
        leftMotor.setControl(request.withOutput(speed));
        rightMotor.setControl(request.withOutput(speed));
    }

    public void setManualControl(boolean manualControl){
        this.manualControl = manualControl;
    }

    public double getMeasurement(){
        return (leftMotor.getPosition().getValueAsDouble()+ElevatorConstants.elevatorOffset)*ElevatorConstants.metersPerRotation;
    }

    public double getVelocity(){
        return leftMotor.getVelocity().getValueAsDouble()*ElevatorConstants.metersPerRotation;
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public void setVoltage(Voltage v){
        leftMotor.setVoltage(v.magnitude());
        rightMotor.setVoltage(v.magnitude());
    }

    private void telemetry(){
        SmartDashboard.putNumber("Elevator Position", getMeasurement());
        SmartDashboard.putNumber("Elevator Velocity", getVelocity());
        SmartDashboard.putNumber("Elevator Speed",leftMotor.get());
    }

    @Override
    public void periodic(){
        telemetry();
        if(!manualControl){
            double extra = feedforward.calculate(0.0);
            double voltage = controller.calculate(getMeasurement(), getGoal())+extra;
            setVoltage(Volts.of(voltage));
        }
    }
}
