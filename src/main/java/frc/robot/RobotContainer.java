package frc.robot;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.elastic.*;
import frc.robot.gamepieces.Gamepieces;
import frc.robot.other.AutoFactory;
import frc.robot.other.CageChoice;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.visualization.RobotPublisher;
import frc.robot.commands.RobotPublisherCommands;

@SuppressWarnings("unused")
public class RobotContainer {
	// Controllers
	private CommandPS5Controller driver;
	private CommandPS5Controller operator;
	private CommandPS5Controller sysIdTest;

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

	private LED led;

	private MultiCommands multiCommands;

	// Additional components
	private Reef reef;
	private ReefScoring reefScoring;
	private CageChoice cageChoice;
	private AutoFactory autoFactory;
	private Gamepieces gamepieces;

	private RobotPublisher robotPublisher;
	private RobotPublisherCommands robotPublisherCommands;
	
	private SendableChooser<String> autoChoice;

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
			
			initReef();
			
			
			// Initialize subsystems
			initVision();
			initSwerve();
			initElevator();
			initArm();
			initWrist();
			initIntake();
			initHang();
			initLED();
			
			// Initialize combined systems and commands
			initMultiCommands();
			initAdditionalComponents();

			if (Robot.isSimulation()) {
				initGamepieces();
				initRobotPublisher();
			}

			autoChoice = new SendableChooser<String>();
			autoChoice.setDefaultOption("Single Center", "Single Center");
			AutoBuilder.getAllAutoNames().forEach((name) -> autoChoice.addOption(name, name));
			SmartDashboard.putData("Auto Choice", autoChoice);
		} catch (Exception e) {
			DataLogManager.log("Error while initializing: " + RobotUtils.getError(e));
		}
	}

	private void initControllers() {
		driver = new CommandPS5Controller(ControlConstants.driverControllerPort);
		operator = new CommandPS5Controller(ControlConstants.operatorControllerPort);
		//sysIdTest = new CommandPS5Controller(ControlConstants.sysIdControllerPort);
	}

	private void initReef() {
		reef = new Reef();
		reefScoring = new ReefScoring(reef);
		SmartDashboard.putData("reef", reef);
		SmartDashboard.putData("reefScoring", reefScoring);
	}

	private void initGamepieces() {
		gamepieces = new Gamepieces();
	}

	private void initVision() {
		vision = new Vision(reef);
	}

	private void initSwerve() {
		swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve.json"), vision);
		swerveCommands = new SwerveCommands(swerve);

		// Configure swerve bindings
		// swerve.setDefaultCommand(swerveCommands.drive(
		// 	() -> -Math.pow(MathUtil.applyDeadband(driver.getLeftY(), ControlConstants.driveDeadband), ControlConstants.joystickExponent), 
		// 	() -> -Math.pow(MathUtil.applyDeadband(driver.getLeftX(), ControlConstants.driveDeadband), ControlConstants.joystickExponent), 
		// 	() -> -Math.pow(MathUtil.applyDeadband(driver.getRightX(), ControlConstants.driveDeadband), ControlConstants.joystickExponent),
		// 	() -> true)
		// );

		driver.L3().whileTrue(swerveCommands.lock());
		driver.options().onTrue(swerveCommands.resetGyro());
	}

	private void initElevator() {
		elevator = new Elevator();
		elevatorCommands = new ElevatorCommands(elevator);

		// Configure elevator bindings
		elevator.setDefaultCommand(elevatorCommands.changeGoal(() -> -MathUtil.applyDeadband(operator.getLeftY(), ControlConstants.operatorDeadband) * ControlConstants.elevatorManualRate));
	}

	private void initArm() {
		arm = new Arm();
		armCommands = new ArmCommands(arm);

		// Configure arm bindings
		arm.setDefaultCommand(armCommands.changeGoal(() -> -MathUtil.applyDeadband(operator.getRightX(), ControlConstants.operatorDeadband) * ControlConstants.armManualRate));
	}

	private void initWrist() {
		wrist = new Wrist(arm);
		wristCommands = new WristCommands(wrist);

		// Configure wrist bindings
		operator.L1().whileTrue(wristCommands.changeGoal(() -> ControlConstants.wristManualRate));
		operator.R1().whileTrue(wristCommands.changeGoal(() -> -ControlConstants.wristManualRate));
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

	private void initLED() {
		led = new LED();
	}
	
	private void initRobotPublisher() {
		robotPublisher = new RobotPublisher(arm, elevator, wrist, swerve, gamepieces);
		robotPublisherCommands = new RobotPublisherCommands(robotPublisher);
	}

	private void initMultiCommands() {
		multiCommands = new MultiCommands(arm, elevator, wrist, swerve, swerveCommands, intakeCommands, hangCommands);
		multiCommands.setRobotPublisher(robotPublisher);

		// Register named commands for auto
		NamedCommands.registerCommand("L4", multiCommands.moveToLevel(3));
		NamedCommands.registerCommand("Default", multiCommands.moveToDefault());
		NamedCommands.registerCommand("SourceCoral", multiCommands.getCoralFromSource());

		// Driver controls
		driver.triangle().and(driver.R2().negate()).onTrue(multiCommands.getCoralFromSource());
		driver.circle().and(driver.R2().negate()).onTrue(multiCommands.getAlgae());

		driver.triangle().and(driver.R2()).onTrue(multiCommands.placeCoral(3));
		driver.square().and(driver.R2()).onTrue(multiCommands.placeCoral(2));
		driver.circle().and(driver.R2()).onTrue(multiCommands.placeCoral(1));
		driver.cross().and(driver.R2()).onTrue(multiCommands.placeCoral(0));

		driver.triangle().negate()
		.and(driver.square().negate())
		.and(driver.circle().negate())
		.and(driver.cross().negate())
		.onTrue(multiCommands.moveToDefault());

		driver.L1()
		.toggleOnTrue(intakeCommands.pulseIntake())
		.onTrue(wristCommands.setGoal(() -> ControlConstants.wristIntakePosition));
		
		// Operator controls
		operator.R1().onTrue(multiCommands.moveToDefault());
		operator.L1().onTrue(multiCommands.moveToSource());
		operator.square().onTrue(multiCommands.moveToProcessor());
		operator.triangle().onTrue(multiCommands.moveToAlgae());
		
		// Gamepiece handling simulation
		// operator.R2().onTrue(multiCommands.simCoralIntake());
		// operator.L2().onTrue(multiCommands.simAlgaeIntake());
		
		// operator.cross().onTrue(multiCommands.simCoralOuttake());
		// operator.circle().onTrue(multiCommands.simAlgaeOuttake());
	}

	private void initAdditionalComponents() {
		cageChoice = new CageChoice();
		autoFactory = new AutoFactory(multiCommands);
	}

	private void configureSysIdBindings(SysIdCommands subsystemCommands) {
		sysIdTest.cross()
			.onTrue(subsystemCommands.sysIdDynamic(Direction.kForward))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		sysIdTest.circle()
			.onTrue(subsystemCommands.sysIdDynamic(Direction.kReverse))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		sysIdTest.square()
			.onTrue(subsystemCommands.sysIdQuasistatic(Direction.kForward))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

		sysIdTest.triangle()
			.onTrue(subsystemCommands.sysIdQuasistatic(Direction.kReverse))
			.onFalse(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
	}

	public void resetGyro() {
		if (swerve != null) {
			swerve.resetGyro();
		}
	}

	/**
	 * Returns the autonomous command to be executed.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		/*if (autoFactory != null) {
			return autoFactory.getAuto();
		}*/
		resetGyro();
		return new WaitCommand(ControlConstants.autoInitialWaitTime).andThen(multiCommands.placeCoral(3)).andThen(multiCommands.moveToDefault());
	}
}