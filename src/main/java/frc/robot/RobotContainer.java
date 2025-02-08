package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.elastic.*;
import frc.robot.other.AutoFactory;
import frc.robot.other.CageChoice;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

@SuppressWarnings("unused")
public class RobotContainer {
	// Controllers for driver and operator
	private CommandPS5Controller driver;

	// Subsystems
	private Vision vision;
	private Swerve swerve;
	private Elevator elevator;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private Hang hang;
	private ArmSystem armSystem;
	private LED led;

	// Commands for each subsystem
	private SwerveCommands swerveCommands;
	private ElevatorCommands elevatorCommands;
	private ArmCommands armCommands;
	private WristCommands wristCommands;
	private IntakeCommands intakeCommands;
	private HangCommand hangCommand;
	private ArmSystemCommands armSystemCommands;
	private MultiCommands multiCommands;

	// Additional components
	private Reef reef;
	private ReefScoring reefScoring;

	private CageChoice cageChoice;

	// Factory for autonomous commands
	private AutoFactory autoFactory;

	private StringLogEntry log;

	/**
	 * Constructor for RobotContainer.
	 * Initializes all subsystems, commands, and binds controls.
	 */
	public RobotContainer() {
		// Start data log
		DataLogManager.start();
		DataLog dataLog=DataLogManager.getLog();
		DriverStation.startDataLog(dataLog);
		log = new StringLogEntry(dataLog, "container");
		StringLogEntry swerveLog = new StringLogEntry(dataLog, "swerve");

		try{
			// Initialize controller
			driver = new CommandPS5Controller(0);

			// Initialize Reef and ReefScoring components
			reef = new Reef();
			reefScoring = new ReefScoring(reef);
			SmartDashboard.putData("reef", reef);
			SmartDashboard.putData("reefScoring", reefScoring);

			// // Initialize subsystems with data log
			vision = new Vision(reef,new StringLogEntry(dataLog, "vision"));
			swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve.json"), vision,swerveLog);
			elevator = new Elevator(new StringLogEntry(dataLog, "elevator"));
			arm = new Arm(new StringLogEntry(dataLog, "arm"));
			wrist = new Wrist(new StringLogEntry(dataLog, "wrist"));
			intake = new Intake(new StringLogEntry(dataLog, "intake"));
			hang = new Hang(new StringLogEntry(dataLog, "hang"));
			armSystem = new ArmSystem(arm, elevator, wrist);
			led = new LED();

			// Initialize commands for each subsystem
			swerveCommands = new SwerveCommands(swerve);
			elevatorCommands = new ElevatorCommands(elevator);
			armCommands = new ArmCommands(arm);
			wristCommands = new WristCommands(wrist);
			intakeCommands = new IntakeCommands(intake);
			hangCommand = new HangCommand(hang);
			armSystemCommands = new ArmSystemCommands(armSystem);
			multiCommands = new MultiCommands(armSystemCommands, swerveCommands, intakeCommands, hangCommand, reef);

			// Initialize cage choice
			cageChoice = new CageChoice();
			
			// Configure button bindings
			configureBindings();

			// Initialize autonomous command factory
			autoFactory = new AutoFactory(multiCommands);
		}catch(Exception e){
			log.append("Error while initializing: "+RobotUtils.getError(e));
		}
	}

	/**
	 * Configures button bindings for driver and operator controllers.
	 */
	private void configureBindings() {
		// Driver Bindings
		// Set default command for swerve to drive with joystick inputs
		swerve.setDefaultCommand(
			swerveCommands.drive(
				() -> -driver.getLeftY(),
				() -> -driver.getLeftX(),
				() -> -driver.getRightX(),
				() -> driver.R1().negate().getAsBoolean())
		);

		// Bind cross button to lock swerve
		driver.L3().whileTrue(swerveCommands.lock());

		// Drive to reef positions
		driver.L1().onTrue(swerveCommands.driveToReefLeft());
		driver.R1().onTrue(swerveCommands.driveToReefRight());

		// Ground Intake
		driver.L2().and(driver.R2().negate()).onTrue(multiCommands.groundIntake());

		// Source, Cage, and Processor
		driver.triangle().and(driver.R2().negate()).onTrue(multiCommands.sourceIntake());
		driver.square().and(driver.R2().negate()).onTrue(multiCommands.hang(()->cageChoice.getCage()));
		driver.circle().and(driver.R2().negate()).onTrue(multiCommands.scoreProcessor());

		// Algae Removal
		//driver.L2().and(driver.R2()).onTrue(multiCommands.getAlgae(0));

		// Scoring Buttons (Right trigger must be pressed down)
		driver.triangle().and(driver.R2()).onTrue(multiCommands.scoreCoral(4));
		driver.square().and(driver.R2()).onTrue(multiCommands.scoreCoral(3));
		driver.circle().and(driver.R2()).onTrue(multiCommands.scoreCoral(2));
		driver.cross().and(driver.R2()).onTrue(multiCommands.scoreCoral(1));

		// Default Arm System
		driver.triangle().negate()
		.and(driver.square().and(driver.R2()).negate())
		.and(driver.circle().negate())
		.and(driver.cross().and(driver.R2()).negate())
		.and(driver.L2().negate())
		.onTrue(armSystemCommands.moveTo("default"));

		// Bind options button to reset gyro
		driver.options().onTrue(swerveCommands.resetGyro());

		/*
		// Example of SysId bindings (commented out)
		driver.povUp().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineTranslation)));
		driver.povLeft().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineSteer)));
		driver.povRight().onTrue(new InstantCommand(() -> swerve.setRoutine(swerve.m_sysIdRoutineRotation)));
		driver.options().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.options().and(driver.povDown()).whileTrue(swerveCommands.sysIdDynamic(Direction.kForward));
		driver.create().and(driver.povDown().negate()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		driver.create().and(driver.povDown()).whileTrue(swerveCommands.sysIdQuasistatic(Direction.kReverse));
		*/

		// Bind touchpad to cancel all commands
		driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		// For testing: Set default commands for elevator and arm with joystick inputs
		driver.povLeft().whileTrue(armCommands.changeGoal(() -> -0.02));
		driver.povRight().whileTrue(armCommands.changeGoal(() -> 0.02));
		driver.povUp().whileTrue(elevatorCommands.changeGoal(() -> 0.02));
		driver.povDown().whileTrue(elevatorCommands.changeGoal(() -> -0.02));
	}

	/**
	 * Returns the autonomous command to be executed.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		return autoFactory.getAuto();
	}
}