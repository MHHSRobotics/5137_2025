package frc.robot.motorSystem;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnhancedEncoder extends DutyCycleEncoder{
    private double last=-1;
    private double shift=0;
    private double mul;
    private double offset;
    private DutyCycleEncoderSim sim;

    // mul should be negative to invert the encoder
    public EnhancedEncoder(int channel,double mul,double offset){
        super(channel);
        this.mul=mul;
        this.offset=offset;
        sim=new DutyCycleEncoderSim(this);
    }

    @Override
    public double get(){
        return (super.get()+shift)*mul+offset;
    }

    public void periodic(){
        double pos=super.get();
        if(last>-1 && Math.abs(last-pos)>0.5){
            if(last<pos){
                shift-=1;
            }else{
                shift+=1;
            }
        }
        last=pos;
    }

    public void downShift() {
        shift -= 1;
    }

    public void upShift() {
        shift += 1;
    }

    public void set(double val){
        double encoderVal=(val-offset)/mul;
        shift=Math.floor(encoderVal);
        sim.set(encoderVal-shift);
    }

    public void log(String path){
        SmartDashboard.putBoolean(path+"/connected", isConnected());
        SmartDashboard.putNumber(path+"/rawPosition", super.get());
        SmartDashboard.putNumber(path+"/position", get());
    }
}
