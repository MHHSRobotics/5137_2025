package frc.robot.constants;

import frc.robot.Robot;

public class HangConstants {
    public static final int encoderId = 3;
    public static final int encoderOffset = 0;
    public static final int motorId=Robot.isSimulation()?25:5;
    public static final double hangSpeed=0.75;
    public static final double maxAngle=100; // TODO change
}
