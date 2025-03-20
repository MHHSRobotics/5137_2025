package frc.robot.motorSystem;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SyncedEncoder extends DutyCycleEncoder{
    private double ratio;
    private double offset;
    private DutyCycleEncoderSim sim;

    public SyncedEncoder(int channel, double ratio, double offset) {
        super(channel);
        this.ratio=ratio;
        this.offset=offset;
        sim=new DutyCycleEncoderSim(this);
    }

    @Override
    public double get() {
        return super.get()*ratio+offset;
    }

    public double getSyncedPosition(double motorPosition) {
        double pos=get();
        return pos+ratio*Math.round((motorPosition-pos)/ratio);
    }

    public void set(double val){
        double encoderVal=((val-offset)/ratio)%1;
        sim.set(encoderVal);
    }

    public void log(String path){
        SmartDashboard.putBoolean(path+"/connected", isConnected());
        SmartDashboard.putNumber(path+"/rawPosition", super.get());
        SmartDashboard.putNumber(path+"/position", get());
    }
}