package frc.robot.subsystems;

//import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkAnalogSensorSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Intake extends SubsystemBase {
    private SparkMax intakeMotor = new SparkMax(25, MotorType.kBrushless);
    //private SparkAnalogSensorSim skibidi = new SparkAnalogSensorSim(intakeMotor);
    //private SparkMaxSim intakeSimMotor = new SparkMaxSim(intakeMotor, DCMotor.getFalcon500(1));
    //private SparkAbsoluteEncoderSim intakeEncoderSim = new SparkAbsoluteEncoderSim(intakeMotor);
    private DigitalInput input = new DigitalInput(0);
    //private FlywheelSim intakeWheelSim = new FlywheelSim(1, IntakeConstants.gearing, );
    
    //private SingleJointedArmSim intakeSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), IntakeConstants.gearRatio, IntakeConstants.jkg, IntakeConstants.intakeLength, IntakeConstants.min, IntakeConstants.max, true, IntakeConstants.min);
    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("IntakeRoot", 10, 0);
    private final MechanismLigament2d intakeMech2d = mech2dRoot.append(new MechanismLigament2d("Intake", 20, 90));

    public Intake(){
    }

    public void setSpeed(double speed){
        
        intakeMotor.set(speed);
        //System.out.println("hello");
    
    }

    public void stop(){
        intakeMotor.stopMotor();
    }
    public boolean sensorState(){
        return input.get();
    }
    //public double getPose() {
        //return ((intakeEncoderSim.getPosition()+IntakeConstants.intakeOffset)/IntakeConstants.gearRatio);
    //}
    //public void telemetry() {
        //SmartDashboard.putNumber("Intake pose", intakeSim.getAngleRads());
        //SmartDashboard.putNumber("Intake Velocity", intakeSim.getVelocityRadPerSec());
        //SmartDashboard.putNumber("Intake Speed", intakeMotor.get());
        //intakeMech2d.setAngle(getPose()*360);
        //System.out.println(getPose()*360);
        //martDashboard.putData("Intake", mech2d);
    //}

    //@Override
    public void periodic(){
        //NOT SURE IF NEEED TO IMPLEMENT
        //telemetry();
    }
    //@Override
    //public void simulationPeriodic(){
        //intakeSimMotor.setMotorCurrent(RobotController.getBatteryVoltage());
        //skibidi.setVoltage();
        //double intakeInput = intakeSimMotor.getMotorCurrent();
        //ntakeSim.update(0.02);
        //double angle = intakeSim.getAngleRads()/(Math.PI*2);

        //intakeSimMotor.setVelocity(intakeSim.getVelocityRadPerSec()/(Math.PI*2)*IntakeConstants.gearRatio);
        //intakeSimMotor.setPosition((angle*IntakeConstants.gearRatio)-IntakeConstants.intakeOffset);
        //oboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));
    //
}