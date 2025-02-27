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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.elastic.*;
import frc.robot.gamepieces.Gamepieces;
import frc.robot.other.AutoFactory;
import frc.robot.other.CageChoice;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.constants.*;

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
	private SwerveSystem swerveSystem;
	private SwerveSystemCommands swerveSystemCommands;

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
			//initLED();
			
			// Initialize combined systems and commands
			initSwerveSystem();
			initMultiCommands();
			//initAdditionalComponents();

			// Configure SysId bindings for elevator
			//configureSysIdBindings(elevatorCommands);

			//arm.resetPos();
			//wrist.resetPos();
			//elevator.resetPos();
		} catch (Exception e) {
			DataLogManager.log("Error while initializing: " + RobotUtils.getError(e));
		}
	}

	private void initControllers() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);
		//sysIdTest = new CommandPS5Controller(2);
	}

	private void initReef() {
		reef = new Reef();
		reefScoring = new ReefScoring(reef);
		SmartDashboard.putData("reef", reef);
		SmartDashboard.putData("reefScoring", reefScoring);
	}

	private void initGamepieces() {
		gamepieces=new Gamepieces();
	}

	private void initVision() {
		vision = new Vision(reef);
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
		operator.L2().or(driver.L2().and(driver.R2().negate()))
			.onTrue(intakeCommands.setSpeed(()->IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());

		operator.R2().or(driver.R1()).or(driver.L2().and(driver.R2()))
			.onTrue(intakeCommands.setSpeed(()->-IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());
		
		NamedCommands.registerCommand("Intake", intakeCommands.setSpeed(()->IntakeConstants.intakeSpeed));
		NamedCommands.registerCommand("Outtake", intakeCommands.setSpeed(()->-IntakeConstants.intakeSpeed));
	}

	private void initHang() {
		hang = new Hang();
		hangCommands = new HangCommands(hang);
		
		operator.povUp().or(driver.povUp()).whileTrue(hangCommands.setSpeed(() -> HangConstants.hangSpeed));
		operator.povDown().or(driver.povDown()).whileTrue(hangCommands.setSpeed(() -> -HangConstants.hangSpeed));
		operator.povLeft().or(driver.povLeft()).whileTrue(hangCommands.setSpeed(() -> -0.2));
		operator.povRight().or(driver.povRight()).onTrue(armCommands.setGoal(() -> Units.degreesToRadians(ArmConstants.hangPosition)));
	}

	private void initLED() {
		led = new LED();
	}

	private void initSwerveSystem() {
		swerveSystem = new SwerveSystem(arm, elevator, wrist, swerve, gamepieces);
		swerveSystemCommands = new SwerveSystemCommands(swerveSystem);

		driver.square().and(driver.R2().negate()).onTrue(swerveSystemCommands.moveToProcessor());
		driver.cross().and(driver.R2().negate()).onTrue(swerveSystemCommands.moveToBarge());

		driver.triangle().and(driver.R2()).onTrue(swerveSystemCommands.moveToLevel(3));
		driver.square().and(driver.R2()).onTrue(swerveSystemCommands.moveToLevel(2));
		driver.circle().and(driver.R2()).onTrue(swerveSystemCommands.moveToLevel(1));
		driver.cross().and(driver.R2()).onTrue(swerveSystemCommands.moveToLevel(0));

		NamedCommands.registerCommand("L4", swerveSystemCommands.moveToLevel(3));

		//driver.axisLessThan(0,-0.1).onTrue(swerveSystemCommands.moveToLevel(3));

		//driver.L1().onTrue(swerveSystemCommands.moveToState(()->SwerveSystemConstants.getGroundIntake()));

		// operator.povUp().onTrue(swerveSystemCommands.moveToGround(()->new Pose2d()));
		// operator.povDown().onTrue(swerveSystemCommands.moveToState(()->SwerveSystemConstants.getSourceStates()[0]));
		// operator.cross().onTrue(swerveSystemCommands.moveToBranch(()->0,()->0));
		// operator.triangle().onTrue(swerveSystemCommands.moveToBranch(()->1,()->0));
		// operator.square().onTrue(swerveSystemCommands.moveToBranch(()->2,()->0));
		// operator.circle().onTrue(swerveSystemCommands.moveToBranch(()->3,()->0));
	}

	private void initMultiCommands() {
		multiCommands = new MultiCommands(swerveSystemCommands, swerveCommands, intakeCommands, hangCommands);

		driver.triangle().and(driver.R2().negate()).onTrue(multiCommands.getCoralFromSource());
		driver.circle().and(driver.R2().negate())
		.onTrue(multiCommands.getAlgae())
		.onFalse(new SequentialCommandGroup(
			new ParallelCommandGroup(
				intakeCommands.pulseIntake(),
				wristCommands.setGoal(() -> Units.degreesToRadians(-90))
			)
		));

		driver.triangle().negate()
		.and(driver.square().negate())
		.and(driver.circle().negate())
		.and(driver.cross().negate())
		.onTrue(new SequentialCommandGroup(
			new WaitCommand(0.1),
			multiCommands.moveToDefault()));

		NamedCommands.registerCommand("Default", multiCommands.moveToDefault());

		driver.L1()
		.toggleOnTrue(intakeCommands.pulseIntake())
		.onTrue(wristCommands.setGoal(() -> Units.degreesToRadians(-90)));
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

	/**
	 * Returns the autonomous command to be executed.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		/*if (autoFactory != null) {
			return autoFactory.getAuto();
		}*/
		swerve.resetGyro();
		return AutoBuilder.buildAuto("Test Auto");
	}
}