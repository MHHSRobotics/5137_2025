package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private final DutyCycleOut request=new DutyCycleOut(0.0);
    private PIDController controller;
    private ElevatorFeedforward feedforward;
    private double goal=0;
    private boolean manualControl;
    public Elevator(){
        controller=new PIDController(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD);
        feedforward=new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv, ElevatorConstants.ka);
        leftMotor = new TalonFX(ElevatorConstants.leftMotorId, "rhino");
        rightMotor = new TalonFX(ElevatorConstants.rightMotorId, "rhino");

        var currentConfigs=new MotorOutputConfigs();

        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(currentConfigs);

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(currentConfigs);

        controller.setTolerance(ElevatorConstants.elevatorTol);

        manualControl=false;
    }

    public double getGoal(){
        return goal;
    }

    public void setGoal(double goal){
        this.goal=goal;
    }

    public void setSpeed(double speed){
        leftMotor.setControl(request.withOutput(speed));
        rightMotor.setControl(request.withOutput(speed));
    }

    public void setManualControl(boolean manualControl){
        this.manualControl=manualControl;
    }

    public double getMeasurement(){
        return (leftMotor.getPosition().getValueAsDouble()+ElevatorConstants.elevatorOffset)*ElevatorConstants.elevatorRatio;
    }

    @Override
    public void periodic(){
        if(manualControl){
            double extra=feedforward.calculate(0.0);
            double voltage=controller.calculate(getMeasurement(), getGoal())+extra;
            leftMotor.setVoltage(voltage);
            rightMotor.setVoltage(voltage);
        }
    }
}
