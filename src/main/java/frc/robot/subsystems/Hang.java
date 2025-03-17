package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HangConstants;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.other.RobotUtils;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The Hang subsystem controls the pneumatics for the robot's hanging mechanism.
 * This includes the clamp and climb solenoids, as well as the compressor that
 * provides the necessary air pressure.
 */
public class Hang extends SubsystemBase {
    
    private EnhancedTalonFX hangMotor;

    /**
     * Constructs a new Hang subsystem.
     */
    public Hang() {
        hangMotor = new EnhancedTalonFX(
            HangConstants.motorId,
            "rio",
            1,
            true,
            true
        );
    }
    
    public void setSpeed(double speed){
        hangMotor.set(speed);
    }

    public void stop(){
        hangMotor.stopMotor();
    }

    private void telemetry(){
        hangMotor.log("hang/motor");
    }

    
    @Override
    public void periodic() {
        try{
            telemetry();
        }catch(RuntimeException e){
            DataLogManager.log("Periodic error: "+RobotUtils.processError(e));
        }
    }
        
}