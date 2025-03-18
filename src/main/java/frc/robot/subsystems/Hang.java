package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HangConstants;
import frc.robot.motorSystem.EnhancedEncoder;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.motorSystem.MotorSystem;
import frc.robot.other.RobotUtils;

import java.util.List;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The Hang subsystem controls the pneumatics for the robot's hanging mechanism.
 * This includes the clamp and climb solenoids, as well as the compressor that
 * provides the necessary air pressure.
 */
public class Hang extends SubsystemBase {
    
    private MotorSystem motorSystem;

    private boolean hitMin;

    /**
     * Constructs a new Hang subsystem.
     */
    public Hang() {
       // Create motor and encoder
        EnhancedTalonFX hangMotor = new EnhancedTalonFX(
            HangConstants.motorId, 
            "rio", 
            1, 
            false, 
            true  // Use brake mode for better position holding
        );
        EnhancedEncoder hangEncoder = new EnhancedEncoder(
            HangConstants.encoderId, 
            2*Math.PI,
            HangConstants.encoderOffset
        );

        // Create motor system
        motorSystem = new MotorSystem(List.of(hangMotor), hangEncoder);
    }
    
    public void setSpeed(double speed){
        if(hitMin && getMeasurement()>HangConstants.maxAngle){
            motorSystem.set(0);
        }else{
            motorSystem.set(speed);
        }
    }

    public void stop(){
        motorSystem.set(0);
    }

    private void telemetry(){
        motorSystem.log("hang");
    }

    public double getMeasurement(){
        return motorSystem.getMeasurement();
    }

    @Override
    public void periodic() {
        try{
            telemetry();
            if(getMeasurement()<HangConstants.minAngle){
                hitMin=true;
            }
        }catch(RuntimeException e){
            DataLogManager.log("Periodic error: "+RobotUtils.processError(e));
        }
    }
        
}