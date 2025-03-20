package frc.robot.motorSystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.other.PIDFConstants;

public class BetterTalonFX extends TalonFX {
    private MotionMagicVoltage request;
    private TalonFXSimState sim;

    public BetterTalonFX(int port, String bus) {
        super(port, bus);
        request = new MotionMagicVoltage(0.0).withUpdateFreqHz(1000);
        sim=new TalonFXSimState(this);
    }

    public BetterTalonFX withMotorOutputConfigs(boolean inverted, boolean braked) {
        this.getConfigurator().apply(
            new MotorOutputConfigs()
            .withNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast)
            .withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        );
        sim.Orientation=inverted?ChassisReference.Clockwise_Positive:ChassisReference.CounterClockwise_Positive;
        return this;
    }

    public BetterTalonFX withCurrentConfigs(double statorLimit, double supplyLimit) {
        this.getConfigurator().apply(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(statorLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyLimit)
            .withSupplyCurrentLimitEnable(true)
        );
        return this;
    }

    public BetterTalonFX withRatioConfigs(double sensorToMechanismRatio) {
        this.getConfigurator().apply(
            new FeedbackConfigs()
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(sensorToMechanismRatio)
        );
        return this;
    }

    public BetterTalonFX withPIDFConfigs(PIDFConstants constants) {
        this.getConfigurator().apply(constants.slotConfigs());
        return this;
    }

    public BetterTalonFX withMotionMagicConfigs(double maxVelocity, double maxAcceleration) {
        this.getConfigurator().apply(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicAcceleration(maxAcceleration)
        );
        return this;
    }

    public double getPos() {
        return this.getPosition().getValueAsDouble();
    }

    public double getVel() {
        return this.getVelocity().getValueAsDouble();
    }

    public double getAcc() {
        return this.getAcceleration().getValueAsDouble();
    }

    public void setTargetPosition(double position) {
        this.setControl(request.withPosition(position));
    }
    
    public double getTargetPosition() {
        return request.Position;
    }

    public double getError(){
        return getPos()-getTargetPosition();
    }

    public void setSimVel(double vel){
        sim.setRotorVelocity(vel);
    }

    public void setSimAcc(double acc){
        sim.setRotorAcceleration(acc);
    }

    public void refreshSimVoltage(){
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    public double getSimVoltage(){
        return sim.getMotorVoltage();
    }

    public void log(String path){
        SmartDashboard.putNumber(path+"/output",get());
        SmartDashboard.putNumber(path+"/position", getPos());
        SmartDashboard.putNumber(path+"/targetPosition", getTargetPosition());
        SmartDashboard.putNumber(path+"/error", getError());
        SmartDashboard.putNumber(path+"/velocity", getVel());
        SmartDashboard.putNumber(path+"/acceleration", getAcc());
        SmartDashboard.putNumber(path+"/temp", getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(path+"/fault", getFaultField().asSupplier().get());
        SmartDashboard.putNumber(path+"/supplyCurrent", getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(path+"/statorCurrent", getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(path+"/voltage", getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(path+"/supplyVoltage", getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber(path+"/running", getMotionMagicIsRunning().getValueAsDouble());
    }
}
