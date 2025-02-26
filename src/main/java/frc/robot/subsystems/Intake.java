package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.other.RobotUtils;

/**
 * The Intake subsystem controls the intake mechanism of the robot.
 * It includes a motor to control the intake mechanism and a limit switch
 * to detect when the intake is in a specific position.
 */
public class Intake extends SubsystemBase {
    private EnhancedTalonFX intakeMotor;

    /**
     * Constructs a new Hang subsystem.
     */
    public Intake() {
        intakeMotor = new EnhancedTalonFX(
            IntakeConstants.motorId,
            "rio",
            1,
            true,
            true
        );
    }
    
    public void setSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void stop(){
        intakeMotor.stopMotor();
    }

    private void telemetry(){
        intakeMotor.log("intake/motor");
    }

    
    @Override
    public void periodic() {
        try{
            telemetry();
        }catch(Exception e){
            DataLogManager.log("Periodic error: "+RobotUtils.getError(e));
        }
    }
        
}