package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.gamepieces.Gamepieces;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.*;
import frc.robot.visualization.RobotPublisher;
import frc.robot.commands.*;
import frc.robot.constants.*;

@SuppressWarnings("unused")
public class RobotContainer {
	// Controllers
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;

	// Subsystems and their commands
	private Vision vision;
	private Swerve swerve;
	private SwerveCommands swerveCommands;

	private Elevator elevator;
	private ElevatorCommands elevatorCommands;

	private Arm arm;
	private ArmCommands armCommands;

	private Wrist wrist;
	private WristCommands wristCommands;

	private Intake intake;
	private IntakeCommands intakeCommands;

	private Hang hang;
	private HangCommands hangCommands;

	private MultiCommands multiCommands;

	// Additional components
	private Gamepieces gamepieces;

	private SendableChooser<String> autoChoice;
	private Command auto;
	private boolean autoAlignDisabled;

	private RobotPublisher robotPublisher;
	private RobotPublisherCommands robotPublisherCommands;

	/**
	 * Constructor for RobotContainer.
	 * Initializes all subsystems, commands, and binds controls.
	 */
	public RobotContainer() {
		// Start data logging
		DataLogManager.start();
		
		DriverStation.startDataLog(DataLogManager.getLog());

		try {
			initControllers();
			
			// Configure emergency stop - this should always be available
			driver.touchpad().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
			
			if (Robot.isSimulation()) {
				initGamepieces();
			}
			
			// Initialize subsystems
			initVision();
			initSwerve();
			initElevator();
			initArm();
			initWrist();
			initIntake();
			initHang();
			initRobotPublisher();
			
			// Initialize combined systems and commands
			initMultiCommands();

			autoChoice = new SendableChooser<String>();
			AutoBuilder.getAllAutoNames().forEach((name) -> autoChoice.addOption(name, name));
			SmartDashboard.putData("Auto Choice", autoChoice);

			autoChoice.onChange((choice) -> {
				swerve.resetGyro();
				auto = AutoBuilder.buildAuto(choice);
			});

			autoAlignDisabled = false;
		} catch (RuntimeException e) {
			DataLogManager.log("Error while initializing: " + RobotUtils.processError(e));
		}
	}

	private void initControllers() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);
	}

	private void initGamepieces() {
		gamepieces=new Gamepieces();
	}

	private void initVision() {
		vision = new Vision();
	}

	private void initSwerve() {
		swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve.json"), vision);
		swerveCommands = new SwerveCommands(swerve);

		// Configure swerve bindings
		swerve.setDefaultCommand(swerveCommands.drive(
			() -> -Math.pow(MathUtil.applyDeadband(driver.getLeftY(), 0.05), 1), 
			() -> -Math.pow(MathUtil.applyDeadband(driver.getLeftX(), 0.05), 1), 
			() -> -Math.pow(MathUtil.applyDeadband(driver.getRightX(), 0.05), 1),
			() -> true)
		);

		driver.L3().whileTrue(swerveCommands.lock());
		driver.options().onTrue(swerveCommands.resetGyro());
	}

	private void initElevator() {
		elevator = new Elevator();
		elevatorCommands = new ElevatorCommands(elevator);

		// Configure elevator bindings
		elevator.setDefaultCommand(elevatorCommands.changeGoal(() -> -MathUtil.applyDeadband(operator.getLeftY(),0.1) / 50));

		driver.povLeft().whileTrue(
			new SequentialCommandGroup(
				new WaitCommand(3),
				elevatorCommands.downShift()));
		driver.povRight().whileTrue(
			new SequentialCommandGroup(
				new WaitCommand(3),
				elevatorCommands.upShift()));
	}

	private void initArm() {
		arm = new Arm();
		armCommands = new ArmCommands(arm);

		// Configure arm bindings
		arm.setDefaultCommand(armCommands.changeGoal(() -> -MathUtil.applyDeadband(operator.getRightX(), 0.1) / 50));
	}

	private void initWrist() {
		wrist = new Wrist(arm);
		wristCommands = new WristCommands(wrist);

		// Configure wrist bindings
		operator.L1().whileTrue(wristCommands.changeGoal(()->0.02));
		operator.R1().whileTrue(wristCommands.changeGoal(()->-0.02));
	}

	private void initIntake() {
		intake = new Intake();
		intakeCommands = new IntakeCommands(intake);

		// Configure intake bindings
		operator.L2().or(driver.L2().and(driver.R2().negate())).or(driver.R1().and(driver.R2()))
			.onTrue(intakeCommands.setSpeed(()->IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());

		operator.R2().or(driver.L2().and(driver.R2())).or(driver.R1().and(driver.R2().negate()))
			.onTrue(intakeCommands.setSpeed(()->-IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());
		
		NamedCommands.registerCommand("Intake", intakeCommands.setSpeed(()->IntakeConstants.intakeSpeed));
		NamedCommands.registerCommand("Outtake", intakeCommands.setSpeed(()->-IntakeConstants.intakeSpeed));
	}

	private void initHang() {
		hang = new Hang();
		hangCommands = new HangCommands(hang);
		
		driver.povUp().whileTrue(hangCommands.setSpeed(() -> -driver.getRightY()*HangConstants.hangSpeed));
	}

	private void initMultiCommands() {
		multiCommands = new MultiCommands(arm,elevator,wrist,swerve, swerveCommands, intakeCommands, hangCommands,robotPublisherCommands);

		driver.triangle().and(driver.R2().negate()).onTrue(multiCommands.getCoralFromSource());
		driver.circle().and(driver.R2().negate()).onTrue(multiCommands.getAlgae());

		driver.triangle().and(driver.R2()).onTrue(multiCommands.placeCoral(3, () -> autoAlignDisabled));
		driver.square().and(driver.R2()).onTrue(multiCommands.placeCoral(2, () -> autoAlignDisabled));
		driver.circle().and(driver.R2()).onTrue(multiCommands.placeCoral(1, () -> autoAlignDisabled));
		driver.cross().and(driver.R2()).onTrue(multiCommands.placeCoral(0, () -> autoAlignDisabled));

		driver.create().onTrue(new InstantCommand(() -> {autoAlignDisabled = !autoAlignDisabled;}));

		driver.triangle().negate()
		.and(driver.square().negate())
		.and(driver.circle().negate())
		.and(driver.cross().negate())
		.onTrue(multiCommands.moveToDefault());

		driver.povDown()
		.onTrue(multiCommands.moveToDefault());

		NamedCommands.registerCommand("Default", multiCommands.moveToDefault());
		NamedCommands.registerCommand("SourceCoral", multiCommands.getCoralFromSource());
		NamedCommands.registerCommand("L4", multiCommands.placeCoral(3, () -> true));

		driver.L1()
		.toggleOnTrue(intakeCommands.pulseIntake())
		.onTrue(wristCommands.setGoal(() -> Units.degreesToRadians(-45)));

		driver.square().and(driver.R2().negate()).onTrue(multiCommands.moveToProcessor());
		driver.cross().and(driver.R2().negate()).onTrue(multiCommands.moveToBarge());
	}

	private void initRobotPublisher() {
		robotPublisher = new RobotPublisher(arm, elevator, wrist, swerve, gamepieces);
		robotPublisherCommands = new RobotPublisherCommands(robotPublisher);
	}

	/**
	 * Returns the autonomous command to be executed.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		if (auto != null) {
			return auto;
		} else {
			return AutoBuilder.buildAuto("Leave");
		}
	}
}