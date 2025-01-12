package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm_Constants;

public class Arm extends SubsystemBase{
    
    private TalonFX armMotor;
    public MotorOutputConfigs motorOutput;
    private PIDController armPID;

    public Arm() {
        armMotor = new TalonFX(Arm_Constants.motorId, "rhino");
        motorOutput = new MotorOutputConfigs();
        armPID = new PIDController(Arm_Constants.kP, Arm_Constants.kI, Arm_Constants.kP);

    }

    public void setArmUp() {
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotor.getConfigurator().apply(motorOutput);
        
    }
    public void setArmDown() {
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotor.getConfigurator().apply(motorOutput);
        
    }
    public void setSpeed() {
        armMotor.set(Arm_Constants.armSpeed);
    }
    public void moveToLevel(double targetAngle) {
        armPID.setSetpoint(targetAngle);
        armPID.enableContinuousInput(Arm_Constants.min, Arm_Constants.max);
    }
}
