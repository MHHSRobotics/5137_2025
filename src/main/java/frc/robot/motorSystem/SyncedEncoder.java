package frc.robot.motorSystem;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SyncedEncoder {
    private BetterTalonFX motor;
    private DutyCycleEncoder encoder;
    private double ratio;
    private double offset;

    public SyncedEncoder(BetterTalonFX motor, int id, double encoderRatio, double encoderOffset) {
        this.motor = motor;
        this.encoder = new DutyCycleEncoder(id);
        this.ratio = encoderRatio;
        this.offset = encoderOffset;
    }

    public BetterTalonFX getMotor() {
        return motor;
    }

    public DutyCycleEncoder getEncoder() {
        return encoder;
    }

    public double getRawEncoderPosition() {
        return encoder.get()*ratio + offset;
    }

    public double getSyncedEncoderPosition() {
        return encoder.get()*(Math.floor(motor.getPositionAsDouble()/ratio))*ratio + offset;
    }

    public void update() {
       motor.setPosition(getSyncedEncoderPosition());
    }

    public void log(String path){
        SmartDashboard.putBoolean(path+"/connected", encoder.isConnected());
        SmartDashboard.putNumber(path+"/rawPosition", getRawEncoderPosition());
        SmartDashboard.putNumber(path+"/syncedPosition", getSyncedEncoderPosition());
    }
}