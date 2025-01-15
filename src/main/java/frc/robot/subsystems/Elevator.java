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
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase{

    private TalonFX leftMotor = new TalonFX(ElevatorConstants.leftMotorId, "rhino");
    private TalonFX rightMotor = new TalonFX(ElevatorConstants.rightMotorId, "rhino");

    private final DutyCycleOut request = new DutyCycleOut(0.0);

    private PIDController controller;

    private ElevatorFeedforward feedforward;

    private ElevatorSim elevatorSim=new ElevatorSim(DCMotor.getKrakenX60(2), ElevatorConstants.elevatorGearing, ElevatorConstants.carriageMass, ElevatorConstants.drumRadius, ElevatorConstants.minHeight, ElevatorConstants.maxHeight, true,ElevatorConstants.startingHeight);
    private TalonFXSimState leftMotorSim=new TalonFXSimState(leftMotor);
    private TalonFXSimState rightMotorSim=new TalonFXSimState(rightMotor);
    
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

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d elevatorMech2d =
        mech2dRoot.append(
            new MechanismLigament2d("Elevator", 0, 90));

    public Elevator(){
        controller = new PIDController(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD);
        feedforward = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);

        var currentConfigs = new MotorOutputConfigs();

        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(currentConfigs);

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(currentConfigs);

        controller.setTolerance(ElevatorConstants.elevatorTol);

        manualControl = false;

        setVoltage(Voltage.ofBaseUnits(12, Volts));

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
        SmartDashboard.putNumber("Elevator Position", elevatorSim.getPositionMeters());
        SmartDashboard.putNumber("Elevator Velocity", getVelocity());
        SmartDashboard.putNumber("Elevator Speed",leftMotor.get());
        elevatorMech2d.setLength(elevatorSim.getPositionMeters()*30);
        SmartDashboard.putData("Elevator", mech2d);
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

    @Override
    public void simulationPeriodic(){
        leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double elevatorInput=(leftMotorSim.getMotorVoltage()-rightMotorSim.getMotorVoltage())/2;
        elevatorSim.setInputVoltage(elevatorInput);
        elevatorSim.update(0.02);
        double pos=elevatorSim.getPositionMeters();
        leftMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond()/ElevatorConstants.metersPerRotation);
        rightMotorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond()/ElevatorConstants.metersPerRotation);
        leftMotorSim.setRawRotorPosition(pos/ElevatorConstants.metersPerRotation-ElevatorConstants.elevatorOffset);
        rightMotorSim.setRawRotorPosition(pos/ElevatorConstants.metersPerRotation-ElevatorConstants.elevatorOffset);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}
