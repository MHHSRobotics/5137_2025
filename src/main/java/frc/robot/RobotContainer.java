package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.elastic.*;
import frc.robot.other.AutoFactory;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

@SuppressWarnings("unused")
public class RobotContainer {
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;

	private Vision vision;
	private Swerve swerve;
	private Elevator elevator;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
  	private Hang hang;

	private SwerveCommands swerveCommands;
	private ElevatorCommands elevatorCommands;
	private ArmCommands armCommands;
	private WristCommands wristCommands;
	private IntakeCommands intakeCommands;
  	private HangCommand hangCommand;

	private Supplier<Command> scoreL4;
	private Supplier<Command> scoreL3;
	private Supplier<Command> scoreL2;
	private Supplier<Command> scoreL1;
	private Supplier<Command> removeAlgae;
	private Supplier<Command> groundIntake;
	private Supplier<Command> sourceIntake;

	private Reef reef;

	private AutoFactory autoFactory;

	@SuppressWarnings("unchecked")
	public RobotContainer() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);

		vision = new Vision();
		swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve.json"), vision);
		elevator = new Elevator();
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake();
    	hang = new Hang();

		swerveCommands = new SwerveCommands(swerve);
		elevatorCommands = new ElevatorCommands(elevator);
		armCommands = new ArmCommands(arm);
		wristCommands = new WristCommands(wrist);
		intakeCommands = new IntakeCommands(intake);
    	hangCommand = new HangCommand(hang);

		reef = new Reef();
		SmartDashboard.putData("Reef", reef);

		configureCommands();
		configureBindings();

		autoFactory = new AutoFactory(scoreL4, scoreL3, scoreL2, scoreL1, removeAlgae, groundIntake, sourceIntake);
	}

	private void configureCommands() { //TODO: Configure these to retract elevator/arm, add wrist/intake commands, and add pickup methods
		scoreL4 = () ->
		new SequentialCommandGroup (
			new ParallelCommandGroup(
				elevatorCommands.moveToL4(),
				armCommands.moveToL4()
			)
		);

		scoreL3 = () -> 
		new SequentialCommandGroup (
			new ParallelCommandGroup(
				elevatorCommands.moveToL3(),
				armCommands.moveToL3()
			)
		);

		scoreL2 = () -> 
		new SequentialCommandGroup (
			new ParallelCommandGroup(
				elevatorCommands.moveToL2(),
				armCommands.moveToL2()
			)
		);

		scoreL1 = () -> 
		new SequentialCommandGroup (
			new ParallelCommandGroup(
				elevatorCommands.moveToL1(),
				armCommands.moveToL1()
			)
		);

		removeAlgae = () ->
		new SequentialCommandGroup (
			new ParallelCommandGroup(
				elevatorCommands.moveToL3(), //TODO: Make algae positions
				armCommands.moveToL3()
			)
		);

		groundIntake = () ->
		new SequentialCommandGroup (
			new ParallelCommandGroup(
				elevatorCommands.moveToL1(),
				armCommands.moveToL1()
			)
		);

		sourceIntake = () ->
		new SequentialCommandGroup (
			new ParallelCommandGroup(
				elevatorCommands.moveToL3(),
				armCommands.moveToL3()
			)
		);
	}

	private void configureBindings() {
		// Driver Bindings

		swerve.setDefaultCommand(
			swerveCommands.drive(
				() -> -driver.getLeftY(),
				() -> -driver.getLeftX(),
				() -> -driver.getRightX(),
				() -> driver.R1().negate().getAsBoolean())
		);

		driver.cross().whileTrue(swerveCommands.lock());

		driver.triangle().onTrue(swerveCommands.driveToStation());
		driver.square().onTrue(swerveCommands.driveToCage());
		driver.circle().onTrue(swerveCommands.driveToProcessor());

		driver.povLeft().onTrue(swerveCommands.driveToReefLeft());
		driver.povUp().onTrue(swerveCommands.driveToReefCenter());
		driver.povRight().onTrue(swerveCommands.driveToReefRight());

		driver.options().onTrue(swerveCommands.resetGyro());

		/*
		driver.povUp().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineTranslation)));
		driver.povLeft().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineSteer)));
		driver.povRight().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineRotation)));
		driver.options().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.options().and(driver.povDown()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.create().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		driver.create().and(driver.povDown()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		*/

		driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		// Operator Bindings

		elevator.setDefaultCommand(elevatorCommands.setGoal(()->1-operator.getLeftY()));

		arm.setDefaultCommand(armCommands.setSpeed(() -> operator.getRightX()));

		operator.triangle()
			.onTrue(scoreL4.get());

		operator.circle()
			.onTrue(scoreL3.get());

		operator.square()
			.onTrue(scoreL2.get());

		operator.cross()
			.onTrue(scoreL1.get());

		operator.R1()
			.onTrue(wristCommands.wristForward())
			.onFalse(wristCommands.wristReverse());

		operator.L1()
			.onTrue(intakeCommands.intakeReverse());
			//.onFalse(intakeCommands.stop());

		operator.L2()
			.onTrue(intakeCommands.intakeForward());
			//.onFalse(intakeCommands.stop());
    
		operator.touchpad()
		.onTrue(hangCommand);
	}

	public Command getAutonomousCommand() {
		return autoFactory.getAuto();
	}
}