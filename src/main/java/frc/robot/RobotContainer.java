package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
	private CommandPS5Controller sysIdController;

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
	private SendableChooser<Integer> autoChoiceOne;
	private SendableChooser<Integer> autoChoiceTwo;
	private Command auto;

	private RobotPublisher robotPublisher;
	private RobotPublisherCommands robotPublisherCommands;

	/**
	 * Constructor for RobotContainer.
	 * Initializes all subsystems, commands, and binds controls.
	 */
	public RobotContainer() {
		// Start data logging
		DataLogManager.start();
		SignalLogger.start();
		
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
			//initSysId();

			autoChoice = new SendableChooser<String>();
			AutoBuilder.getAllAutoNames().forEach((name) -> autoChoice.addOption(name, name));
			autoChoice.setDefaultOption("Dynamic", "Dynamic");

			autoChoiceOne = new SendableChooser<Integer>();
			autoChoiceTwo = new SendableChooser<Integer>();
			for (int i = 0; i < 12; i++) {
				autoChoiceOne.addOption("Reef " + (char)('A' + i), i);
				autoChoiceTwo.addOption("Reef " + (char)('A' + i), i);
			}

			autoChoice.onChange((choice) -> {
				swerve.resetGyro();
				if (choice != "Dynamic") {
					auto = AutoBuilder.buildAuto(choice);
				}
			});

			SmartDashboard.putData("Auto Choice", autoChoice);
			SmartDashboard.putData("Auto Choice One", autoChoiceOne);
			SmartDashboard.putData("Auto Choice Two", autoChoiceTwo);

		} catch (RuntimeException e) {
			DataLogManager.log("Error while initializing: " + RobotUtils.processError(e));
		}
	}

	private void initControllers() {
		driver = new CommandPS5Controller(0);
		operator = new CommandPS5Controller(1);
		//sysIdController = new CommandPS5Controller(2);
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

		// () -> swerve.getPose().getY() <= FieldPositions.fieldWidth/2 ? RobotUtils.onRedAlliance() ? FieldPositions.leftStation : FieldPositions.rightStation : RobotUtils.onRedAlliance() ? FieldPositions.rightStation : FieldPositions.leftStation))

		// Configure swerve bindings
		swerve.setDefaultCommand(swerveCommands.drive(
			() -> -Math.pow(MathUtil.applyDeadband(driver.getLeftY(), 0.05), 1), 
			() -> -Math.pow(MathUtil.applyDeadband(driver.getLeftX(), 0.05), 1), 
			() -> -Math.pow(MathUtil.applyDeadband(driver.getRightX(), 0.05), 1),
			(driver.triangle().or(driver.square()).or(driver.circle()).or(driver.cross())).and(driver.R2().negate()))
		);

		//driver.L3().whileTrue(swerveCommands.lock());
		driver.options().debounce(1).onTrue(swerveCommands.resetGyro());
		driver.create().debounce(1).onTrue(new InstantCommand(() -> swerve.toggleAutoAlign()));
	}

	private void initElevator() {
		elevator = new Elevator();
		elevatorCommands = new ElevatorCommands(elevator);

		// Configure elevator bindings
		elevator.setDefaultCommand(elevatorCommands.changeGoal(() -> -MathUtil.applyDeadband(operator.getLeftY(),0.1) / 50));

		driver.povLeft().debounce(1).onTrue(elevatorCommands.downShift());
		driver.povRight().debounce(1).onTrue(elevatorCommands.upShift());
	}

	private void initArm() {
		arm = new Arm();
		armCommands = new ArmCommands(arm);

		// Configure arm bindings
		arm.setDefaultCommand(armCommands.changeGoal(() -> -MathUtil.applyDeadband(operator.getRightX(), 0.1) / 50));
	}

	private void initWrist() {
		wrist = new Wrist();
		wristCommands = new WristCommands(wrist);

		// Configure wrist bindings
		operator.L1().whileTrue(wristCommands.changeGoal(()->0.02));
		operator.R1().whileTrue(wristCommands.changeGoal(()->-0.02));
	}

	private void initIntake() {
		intake = new Intake();
		intakeCommands = new IntakeCommands(intake);

		// Configure intake bindings
		operator.L2().or(driver.L2())
			.onTrue(intakeCommands.setSpeed(()->IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());

		operator.R2().or(driver.R1())
			.onTrue(intakeCommands.setSpeed(()->-IntakeConstants.intakeSpeed))
			.onFalse(intakeCommands.stop());
		
		operator.cross().onTrue(intakeCommands.setSpeed(() -> 0.2, () -> 0.5));
		
		NamedCommands.registerCommand("Intake", intakeCommands.intake(() -> 0.1));
		NamedCommands.registerCommand("Outtake", intakeCommands.outtake());
	}

	private void initHang() {
		hang = new Hang();
		hangCommands = new HangCommands(hang);
		
		driver.povUp().whileTrue(hangCommands.setSpeed(() -> -driver.getRightY()*HangConstants.hangSpeed));
		operator.povUp().whileTrue(hangCommands.setSpeed(()->HangConstants.hangSpeed));
		operator.povDown().whileTrue(hangCommands.setSpeed(()->-HangConstants.hangSpeed));

		/*driver.create().and(driver.R3()).onTrue(
			new ParallelDeadlineGroup(
				new WaitCommand(3.0),
				new InstantCommand(() -> hangCommands.setSpeed(() -> 1.0))));

		driver.create().and(driver.L3()).onTrue(
			new SequentialCommandGroup(
			new ParallelDeadlineGroup(
				new WaitCommand(0.5),
				new InstantCommand(() -> hangCommands.setSpeed(() -> -1.0))),
			new ParallelDeadlineGroup(
				new WaitCommand(0.5),
				new InstantCommand(() -> hangCommands.setSpeed(() -> 1.0)))));*/
	}

	private void initMultiCommands() {
		multiCommands = new MultiCommands(arm,elevator,wrist,swerve, swerveCommands, intakeCommands, hangCommands,robotPublisherCommands);

		driver.triangle().debounce(0.05).and(driver.R2().negate()).onTrue(multiCommands.getCoralFromSource());
		driver.square().debounce(0.05).and(driver.R2().negate()).onTrue(multiCommands.moveToProcessor());
		driver.circle().debounce(0.05).and(driver.R2().negate()).onTrue(multiCommands.getAlgaeFromReef());
		driver.cross().debounce(0.05).and(driver.R2().negate()).onTrue(multiCommands.scoreBarge());

		//driver.L1().and(driver.R2().negate()).debounce(0.05).onTrue(multiCommands.getCoralFromGround(false));
		driver.L1().and(driver.R2().negate()).debounce(0.05).onTrue(multiCommands.getAlgaeFromGround());

		driver.triangle().debounce(0.05).and(driver.R2()).and(driver.L1().negate()).onTrue(multiCommands.placeCoral(3, false));
		driver.square().debounce(0.05).and(driver.R2()).and(driver.L1().negate()).onTrue(multiCommands.placeCoral(2, false));
		driver.circle().debounce(0.05).and(driver.R2()).and(driver.L1().negate()).onTrue(multiCommands.placeCoral(1, false));
		driver.cross().debounce(0.05).and(driver.R2()).and(driver.L1().negate()).onTrue(multiCommands.placeCoral(0, false));

		driver.triangle().debounce(0.05).and(driver.R2()).and(driver.L1()).onTrue(multiCommands.placeCoral(3, true));
		driver.square().debounce(0.05).and(driver.R2()).and(driver.L1()).onTrue(multiCommands.placeCoral(2, true));
		driver.circle().debounce(0.05).and(driver.R2()).and(driver.L1()).onTrue(multiCommands.placeCoral(1, true));
		driver.cross().debounce(0.05).and(driver.R2()).and(driver.L1()).onTrue(multiCommands.placeCoral(0, true));


		driver.triangle().negate()
		.and(driver.square().negate())
		.and(driver.circle().negate())
		.and(driver.cross().negate())
		.and((driver.L1().and(driver.R1().negate())).negate())
		.and((driver.R1().and(driver.L1().negate())).negate())
		.onTrue(multiCommands.moveToDefault());

		driver.povDown()
		.onTrue(multiCommands.moveToDefault());

		NamedCommands.registerCommand("Default", multiCommands.moveToDefault());
		NamedCommands.registerCommand("SourceIntake", multiCommands.getCoralFromSource());
		NamedCommands.registerCommand("GroundIntake", multiCommands.getCoralFromGround(true));
		NamedCommands.registerCommand("L4", multiCommands.placeCoralAuto(3, false));
		NamedCommands.registerCommand("L4_Horz", multiCommands.placeCoralAuto(3, true));
		NamedCommands.registerCommand("Prescore", multiCommands.moveToPreScoringState());
	}

	private void initRobotPublisher() {
		robotPublisher = new RobotPublisher(arm, elevator, wrist, swerve, gamepieces);
		robotPublisherCommands = new RobotPublisherCommands(robotPublisher);
	}

	private void initSysId(){
		sysIdController.square().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		sysIdController.cross().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		sysIdController.triangle().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
		sysIdController.circle().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		sysIdController.L1().onTrue(new InstantCommand(()->SignalLogger.stop()));
	}

	public void vibrateControllers() {
		new Thread(() -> {
			driver.setRumble(RumbleType.kBothRumble, 1);
			operator.setRumble(RumbleType.kBothRumble, 1);
			try {
				Thread.sleep(1000);
				stopVibration();
			} catch (InterruptedException e) {
				stopVibration();
			}
		});
	}

	public void stopVibration() {
		driver.setRumble(RumbleType.kBothRumble, 0);
		operator.setRumble(RumbleType.kBothRumble, 0);
	}

	/**
	 * Returns the autonomous command to be executed.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		if (auto != null && autoChoice.getSelected() != "Dynamic") {
			return auto;
		} else if (autoChoiceOne.getSelected() != null) {
			if (autoChoiceTwo.getSelected() != null) {
				return new SequentialCommandGroup(
					intakeCommands.intake(() -> 0.05),
					multiCommands.placeCoral(()->3, false, ()->autoChoiceOne.getSelected()),
					multiCommands.getCoralFromSource(),
					new WaitCommand(0.5),
					intakeCommands.stop(),
					multiCommands.placeCoral(()->3, false, ()->autoChoiceTwo.getSelected()),
					multiCommands.moveToDefault()
				);
			} else {
				return new SequentialCommandGroup(
					intakeCommands.intake(() -> 0.05),
					multiCommands.placeCoral(()->3, false, ()->autoChoiceOne.getSelected()),
					multiCommands.moveToDefault()
				);
			}
		} else {
			return AutoBuilder.buildAuto("Leave");
		}
	}
}