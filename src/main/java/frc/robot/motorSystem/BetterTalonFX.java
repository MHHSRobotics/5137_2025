package frc.robot.motorSystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.other.PIDFConstants;

public class BetterTalonFX extends TalonFX {
    private MotionMagicTorqueCurrentFOC request;

    public BetterTalonFX(int port, String bus) {
        super(port, bus);
        request = new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(1000);
    }

    public void setMotorOutputConfigs(boolean inverted, boolean braked) {
        this.getConfigurator().apply(
            new MotorOutputConfigs()
            .withNeutralMode(braked ? NeutralModeValue.Brake : NeutralModeValue.Coast)
            .withInverted(inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        );
    }

    public void setCurrentConfigs(double statorLimit, double supplyLimit) {
        this.getConfigurator().apply(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(statorLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyLimit)
            .withSupplyCurrentLimitEnable(true)
        );
    }

    public void setRatioConfigs(double motorToSensorRatio, double sensorToMechanismRatio) {
        this.getConfigurator().apply(
            new FeedbackConfigs()
            .withRotorToSensorRatio(motorToSensorRatio)
            .withSensorToMechanismRatio(sensorToMechanismRatio)
        );
    }

    public void setPIDFConfigs(PIDFConstants constants, boolean isArm) {
        this.getConfigurator().apply(
            new SlotConfigs()
            .withKP(constants.kP)
            .withKI(constants.kI)
            .withKD(constants.kD)
            .withKS(constants.kS)
            .withKG(constants.kG)
            .withKV(constants.kV)
            .withKA(constants.kA)
            .withGravityType(isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static)
        );
    }

    public void setMotionMagicConfigs(double maxVelocity, double maxAcceleration) {
        this.getConfigurator().apply(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicAcceleration(maxAcceleration)
        );
    }

    public double getPositionAsDouble() {
        return this.getPosition().getValueAsDouble();
    }

    public double getVelocityAsDouble() {
        return this.getVelocity().getValueAsDouble();
    }

    public double getAccelerationAsDouble() {
        return this.getAcceleration().getValueAsDouble();
    }

    public void setTargetPosition(double position) {
        this.setControl(request.withPosition(position));
    }
    
    public double getTargetPosition() {
        return request.Position;
    }

    public void log(String path){
        SmartDashboard.putNumber(path+"/output",get());
        SmartDashboard.putNumber(path+"/position", getPositionAsDouble());
        SmartDashboard.putNumber(path+"/velocity", getVelocityAsDouble());
        SmartDashboard.putNumber(path+"/acceleration", getAccelerationAsDouble());
        SmartDashboard.putNumber(path+"/temp", getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(path+"/fault", getFaultField().asSupplier().get());
        SmartDashboard.putNumber(path+"/supplyCurrent", getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(path+"/statorCurrent", getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(path+"/voltage", getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(path+"/supplyVoltage", getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber(path+"/running", this.getMotionMagicIsRunning().getValueAsDouble());
    }
}
