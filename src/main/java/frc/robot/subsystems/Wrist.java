package frc.robot.subsystems;

import frc.robot.constants.WristConstants;

//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units.Volts;

public class Wrist extends SubsystemBase{
    private TalonFX wristMotor = new TalonFX(24, "rhino");
    private TalonFXSimState fakeWristMotor = wristMotor.getSimState();
    
    private PIDController controller = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    private double goal = 0;
    private ArmFeedforward feedFor = new ArmFeedforward(WristConstants.ks, WristConstants.kg, WristConstants.kv);
    
    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("WristRoot", 10, 0);
    private final MechanismLigament2d armMech2d = mech2dRoot.append(new MechanismLigament2d("Wrist", 20, 90));
    private SingleJointedArmSim wristSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), WristConstants.gearRatio, WristConstants.jkg, WristConstants.wristLength, WristConstants.min, WristConstants.max, true, WristConstants.min);

    public Wrist(){
        controller.setTolerance(WristConstants.wristTolerance);
        Shuffleboard.getTab("Wrist Controller").add(controller);
    }
    
    private double getMeasurement(TalonFX motor){
        return (motor.getPosition().getValueAsDouble()-WristConstants.wristOffset)*WristConstants.defaultMotorSpeed;
    }

    public void setSpeed(double speed){
        wristMotor.set(speed);
    }
    
    public void stop(){
        wristMotor.set(0.0);
        wristMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void wristPos1(){
        wristMotor.setPosition(1);
    }

    public void wristPos2(){
        wristMotor.setPosition(0.25);
    }

    public double wristPosition(){
        return wristMotor.getPosition().getValueAsDouble();
    }


    public double getPose() {
        return ((wristMotor.getPosition().getValueAsDouble()+WristConstants.wristOffset)/WristConstants.gearRatio);
    }

    public boolean atSetpoint(){
        return controller.atSetpoint();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Wrist pose", wristSim.getAngleRads());
        SmartDashboard.putNumber("Wrist Velocity", wristSim.getVelocityRadPerSec());
        SmartDashboard.putNumber("Wrist Speed", wristMotor.get());
        armMech2d.setAngle(getPose()*360);
        SmartDashboard.putData("Wrist", mech2d);
    }

    @Override
    public void periodic(){
        telemetry();
        double extra = feedFor.calculate(goal,0);
        double voltage = controller.calculate(getPose(),goal)+extra;
        setVoltage(Volts.of(voltage));
    }
    
    @Override
    public void simulationPeriodic(){
        fakeWristMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
        double wristInput = fakeWristMotor.getMotorVoltage();
        wristSim.setInputVoltage(wristInput);
        wristSim.update(0.02);
        double angle = wristSim.getAngleRads()/(Math.PI*2);

        fakeWristMotor.setRotorVelocity(wristSim.getVelocityRadPerSec()/(Math.PI*2)*WristConstants.gearRatio);
        fakeWristMotor.setRawRotorPosition((angle*WristConstants.gearRatio)-WristConstants.wristOffset);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }
}





