package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private final RobotContainer robotContainer;
	
	public Robot() {
		robotContainer = new RobotContainer();
	}

	public Command prev = new Command() {};

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		Runtime r=Runtime.getRuntime();
		SmartDashboard.putNumber("mem/total",r.totalMemory());
		SmartDashboard.putNumber("mem/free",r.freeMemory());
		SmartDashboard.putNumber("mem/max",r.maxMemory());
		CommandScheduler.getInstance().onCommandInitialize((command) -> {
			System.out.println("Command initialized: " + command.getName());
		});
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		if (MathUtil.isNear(120.0, Timer.getMatchTime(), 0.1)) {
			robotContainer.vibrateControllers();
		}
	}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
