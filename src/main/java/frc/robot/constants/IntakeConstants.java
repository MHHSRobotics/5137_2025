package frc.robot.constants;

import frc.robot.Robot;

public class IntakeConstants {
    public static final int motorId = Robot.isSimulation()?24:4;
    public static final double intakeSpeed = 0.9;
    public static final double outtakeTime = 1;
    public static final double intakeTime = 1;
}
