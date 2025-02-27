package frc.robot.motorSystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnhancedTalonFX extends TalonFX{
    private double mul;
    private TalonFXSimState sim;

    public EnhancedTalonFX(int port,String canBus){
        this(port,canBus,1,false,false);
    }

    public EnhancedTalonFX(int port,String canBus,double mul,boolean inverted,boolean brake){
        super(port,canBus);
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.NeutralMode = brake?NeutralModeValue.Brake:NeutralModeValue.Coast;
        currentConfigs.Inverted = inverted?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive;
        this.getConfigurator().apply(currentConfigs);
        this.mul=mul;
        sim=new TalonFXSimState(this,inverted?ChassisReference.Clockwise_Positive:ChassisReference.CounterClockwise_Positive);
    }

    public double getVel(){
        return super.getVelocity().getValueAsDouble()*mul;
    }

    public double getAcc(){
        return super.getAcceleration().getValueAsDouble()*mul;
    }

    public void setSimVel(double vel){
        sim.setRotorVelocity(vel/mul);
    }

    public void setSimAcc(double acc){
        sim.setRotorAcceleration(acc/mul);
    }

    public void refreshSupplyVoltage(){
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    public double getSimVoltage(){
        return sim.getMotorVoltage();
    }

    public void log(String path){
        SmartDashboard.putNumber(path+"/output",get());
        SmartDashboard.putNumber(path+"/rawVelocity", getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(path+"/velocity", getVel());
        SmartDashboard.putNumber(path+"/rawAcceleration", getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber(path+"/acceleration", getAcc());
        SmartDashboard.putNumber(path+"/temp", getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber(path+"/fault", getFaultField().asSupplier().get());
        SmartDashboard.putNumber(path+"/current", getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(path+"/voltage", getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(path+"/supplyVoltage", getSupplyVoltage().getValueAsDouble());
    }
}
