package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.other.RobotUtils;

/**
 * The Intake subsystem controls the intake mechanism of the robot.
 * It includes a motor to control the intake mechanism and a limit switch
 * to detect when the intake is in a specific position.
 */
public class Intake extends SubsystemBase {
    // Motor controller for the intake mechanism
    private SparkMax intakeMotor;

    // Limit switch to detect the position of the intake
    private DigitalInput limitSwitch;

    private StringLogEntry log;

    /**
     * Constructs an Intake subsystem.
     */
    public Intake(StringLogEntry log) {
        // Initialize the motor controller with the ID and type from constants
        intakeMotor = new SparkMax(IntakeConstants.motorId, MotorType.kBrushless);

        // Initialize the limit switch with the channel from constants
        limitSwitch = new DigitalInput(IntakeConstants.switchChannel);

        this.log=log;
    }

    /**
     * Sets the speed of the intake motor.
     *
     * @param speed The speed to set the motor to, in the range [-1.0, 1.0].
     */
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * Checks if the limit switch is triggered.
     *
     * @return True if the limit switch is triggered, false otherwise.
     */
    public boolean isSwitched() {
        return limitSwitch.get();
    }

    /**
     * Logs relevant data from the Intake subsystem.
     * This method is intended to be called periodically to record data.
     */
    public void telemetry(){
        SmartDashboard.putNumber("intake/speed",intakeMotor.get());
        SmartDashboard.putNumber("intake/appliedOutput",intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("intake/busVoltage",intakeMotor.getBusVoltage());
        SmartDashboard.putNumber("intake/motorTemp",intakeMotor.getMotorTemperature());
        SmartDashboard.putNumber("intake/outputCurrent",intakeMotor.getOutputCurrent());
        SmartDashboard.putBoolean("intake/isSwitched",isSwitched());
    }

    @Override
    public void periodic(){
        try{
            telemetry();
        }catch(Exception e){
            log.append("Periodic error: "+RobotUtils.getError(e));
        }
    }
}