package frc.robot.motorSystem;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// Implementation for SingleJointedArmSim
public class ArmMechanismSim extends SingleJointedArmSim implements MechanismSim {
    public ArmMechanismSim(DCMotor motorSim,double gearRatio,double moi,double length,double minAngle,double maxAngle,boolean simGravity,double startAngle){
        super(motorSim,gearRatio,moi,length,minAngle,maxAngle,simGravity,startAngle);
    }
    
    @Override
    public double getPosition() {
        return getAngleRads();
    }

    @Override
    public double getVelocity() {
        return getVelocityRadPerSec();
    }
}
