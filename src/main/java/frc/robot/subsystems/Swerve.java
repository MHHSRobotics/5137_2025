package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.constants.SwerveConstants;
import frc.robot.motorSystem.EnhancedTalonFX;
import frc.robot.other.DetectedObject;
import frc.robot.other.RobotUtils;
import frc.robot.other.SwerveFactory;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * The Swerve subsystem controls the swerve drivetrain of the robot.
 * It handles driving, odometry, vision integration, and system identification (SysId) routines.
 */
public class Swerve extends SubsystemBase {

    private SwerveDrivetrain<EnhancedTalonFX, EnhancedTalonFX, CANcoder> swerve; // Swerve drivetrain instance
    private Vision vision; // Vision subsystem for pose estimation and object detection

    private double maxSpeed; // Maximum translational speed of the robot
    private double maxAngularSpeed; // Maximum rotational speed of the robot

    private Field2d field; // Field visualization for SmartDashboard
    private Field2d targetField;

    // Swerve control requests
    private SwerveRequest.FieldCentric fieldOrientedDrive; // Field-oriented driving request
    private SwerveRequest.RobotCentric robotOrientedDrive; // Robot-oriented driving request
    private SwerveRequest.ApplyRobotSpeeds setChassisSpeeds; // Request to set chassis speeds directly
    private SwerveRequest.SwerveDriveBrake lock; // Request to lock the swerve modules in place

    // Target pose
    private Pose2d targetPose=new Pose2d();

    private StructArrayPublisher<Pose2d> estPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/estimatedPoses",Pose2d.struct).publish();

    private Command currentAuto;
    /**
     * Constructor for the Swerve subsystem.
     *
     * @param file    The configuration file for the swerve drivetrain.
     * @param vision  The vision subsystem for pose estimation.
     */
    public Swerve(File file, Vision vision) {
        SwerveFactory factory = new SwerveFactory(file);
        swerve = factory.create(); // Create the swerve drivetrain using the factory
        this.vision = vision;

        maxSpeed = factory.getMaxSpeed(); // Get max speed from factory
        maxAngularSpeed = factory.getMaxAngularSpeed(); // Get max angular speed from factory

        // Configure field-oriented and robot-oriented driving requests with deadbands
        fieldOrientedDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband)
            .withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        robotOrientedDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * SwerveConstants.translationalDeadband)
            .withRotationalDeadband(maxAngularSpeed * SwerveConstants.rotationalDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        setChassisSpeeds = new SwerveRequest.ApplyRobotSpeeds(); // Request to set chassis speeds
        lock = new SwerveRequest.SwerveDriveBrake(); // Request to lock the swerve modules

        // Configure PathPlanner AutoBuilder for autonomous driving
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
                this::getPose, // Method to get the current pose
                this::resetPose, // Method to reset the pose
                this::getCurrentSpeeds, // Method to get the current chassis speeds
                (speeds, feedforwards) -> drive(speeds), // Method to drive the robot
                new PPHolonomicDriveController(
                        new PIDConstants(SwerveConstants.translationKP, SwerveConstants.translationKI, SwerveConstants.translationKD),
                        new PIDConstants(SwerveConstants.rotationKP, SwerveConstants.rotationKI, SwerveConstants.rotationKD)
                ),
                config,
                () -> RobotUtils.onRedAlliance(), // Method to check if the robot is on the red alliance
                this
        );

        // Start simulation thread if in simulation
        if (Robot.isSimulation()) {
            startSimThread();
        }

        // Initialize telemetry and field visualization
        swerve.registerTelemetry(this::telemetry);

        field = new Field2d();
        targetField = new Field2d();
        SmartDashboard.putData("field", field);
        SmartDashboard.putData("targetField", targetField);

        // Warmup pathfinding
        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();
    }

    /**
     * Sets the control request for the swerve drivetrain.
     *
     * @param request The swerve control request to apply.
     */
    public void setControl(SwerveRequest request) {
        swerve.setControl(request);
    }

    /**
     * Resets the robot's pose to the specified pose.
     *
     * @param pose The new pose to set.
     */
    public void resetPose(Pose2d pose) {
        swerve.resetPose(pose);
    }

    /**
     * Gets the current pose of the robot.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        return swerve.getState().Pose;
    }

    /**
     * Drives the robot with the specified chassis speeds.
     *
     * @param speeds The chassis speeds to apply.
     */
    public void drive(ChassisSpeeds speeds) {
        setControl(setChassisSpeeds.withSpeeds(speeds));
    }

    /**
     * Gets the current chassis speeds of the robot.
     *
     * @return The current chassis speeds.
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return swerve.getState().Speeds;
    }

    private void cancelAuto(){
        if(currentAuto!=null){
            currentAuto.cancel();
            currentAuto=null;
        }
    }

    private void startAuto(Command auto){
        cancelAuto();
        auto=new ParallelRaceGroup(auto,new WaitCommand(SwerveConstants.moveTimeout));
        auto.schedule();
        currentAuto=auto;
    }
    /**
     * Drives the robot with a percentage of maximum speed.
     *
     * @param dx          The percentage of maximum speed in the x direction.
     * @param dy          The percentage of maximum speed in the y direction.
     * @param dtheta      The percentage of maximum angular speed.
     * @param fieldRelative Whether the drive is field-relative or robot-relative.
     */
    public void setPercentDrive(double dx, double dy, double dtheta, boolean fieldRelative) {
        double absSpeedX = dx*maxSpeed;
        double absSpeedY = dy*maxSpeed;
        double absRot = dtheta*maxAngularSpeed;
        if (fieldRelative) {
            setControl(fieldOrientedDrive
                .withVelocityX(absSpeedX)
                .withVelocityY(absSpeedY)
                .withRotationalRate(absRot)
            );
        } else {
            setControl(robotOrientedDrive
                .withVelocityX(absSpeedX)
                .withVelocityY(absSpeedY)
                .withRotationalRate(absRot)
            );
        }
    }

    /**
     * Resets the gyro and reseeds the field-centric drive.
     */
    public void resetGyro() {
        swerve.setOperatorPerspectiveForward(RobotUtils.getPerspectiveForward());
        swerve.seedFieldCentric();
    }

    /**
     * Locks the swerve modules in place.
     */
    public void lock() {
        this.setControl(lock);
    }

    public List<DetectedObject> getGroundCoral(){
        if(vision!=null){
            return vision.getGroundCoral(SwerveConstants.coralExpirationTime);
        }else{
            return new ArrayList<DetectedObject>();
        }
    }

    public Pose2d getTargetPose(){
        return targetPose;
    }

    public void setTargetPose(Pose2d target){
        targetPose = target;
        startAuto(AutoBuilder.pathfindToPose(target, SwerveConstants.constraints, 0.0));
        //followPath("Reef H");
    }

    public void followPath(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            var poses = path.getPathPoses();
            targetPose = RobotUtils.invertToAlliance(path.getPathPoses().get(poses.size() - 1));
            startAuto(AutoBuilder.pathfindThenFollowPath(path, SwerveConstants.constraints));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public boolean atTarget(){
        if(targetPose==null){
            return true;
        }
        Pose2d currentPose=getPose();
        double dist=currentPose.getTranslation().getDistance(targetPose.getTranslation());
        if(dist>SwerveConstants.transTol){
            return false;
        }
        double rotDist=currentPose.getRotation().minus(targetPose.getRotation()).getRadians();
        if(rotDist>SwerveConstants.rotTol){
            return false;
        }
        return true;
    }

    /**
     * Periodic method called every robot loop cycle.
     * Updates vision measurements, processes new objects, and updates field visualization.
     */
    
    @Override
    public void periodic() {
        try{
            if(vision!=null){
                List<EstimatedRobotPose> newPoses = vision.getNewPoses();
                Pose2d[] estPoses=new Pose2d[newPoses.size()];
                for(int i=0;i<newPoses.size();i++){
                    estPoses[i]=newPoses.get(i).estimatedPose.toPose2d();
                }
                estPosePublisher.set(estPoses);
                for (EstimatedRobotPose newPose : newPoses) {
                    swerve.addVisionMeasurement(newPose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(newPose.timestampSeconds));
                }
                vision.processNewObjects(this.getPose());
                field.setRobotPose(this.getPose());
                targetField.setRobotPose(this.getTargetPose());
                if(Robot.isSimulation()){
                    vision.updateSim(this.getPose());
                }
            }
        }catch(Exception e){
            DataLogManager.log("Periodic error: "+RobotUtils.getError(e));
        }
    }
        

    private void telemetry(SwerveDriveState state){
        SmartDashboard.putNumber("driveState/odometryPeriod", state.OdometryPeriod);
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putNumberArray("driveState/module"+i+"/state", new double[]{state.ModuleStates[i].speedMetersPerSecond,state.ModuleStates[i].angle.getRadians()});
            SmartDashboard.putNumberArray("driveState/module"+i+"/target", new double[]{state.ModuleTargets[i].speedMetersPerSecond,state.ModuleStates[i].angle.getRadians()});
            swerve.getModule(i).getDriveMotor().log("driveState/module"+i+"/driveMotor");
            swerve.getModule(i).getSteerMotor().log("driveState/module"+i+"/steerMotor");
        }
    }

    // SysId Routines for system characterization

    /**
     * SysId routine for characterizing translation.
     * This is used to find PID gains for the drive motors.
     */
    public final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null        // Use default timeout (10 s)
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(new SwerveRequest.SysIdSwerveTranslation().withVolts(output)),
            null,
            this
        )
    );

    /**
     * SysId routine for characterizing steer.
     * This is used to find PID gains for the steer motors.
     */
    public final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null        // Use default timeout (10 s)
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(new SwerveRequest.SysIdSwerveSteerGains().withVolts(volts)),
            null,
            this
        )
    );

    /**
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     */
    public final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second), // Ramp rate in radians per second²
            Volts.of(Math.PI), // Dynamic voltage in radians per second
            null // Use default timeout (10 s)
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts)));
            },
            null,
            this
        )
    );

    private SysIdRoutine routine = sysIdRoutineTranslation; // Current SysId routine to test

    /**
     * Sets the current SysId routine.
     *
     * @param routine The SysId routine to set.
     */
    public void setRoutine(SysIdRoutine routine) {
        this.routine = routine;
    }

    /**
     * Gets the current SysId routine.
     *
     * @return The current SysId routine.
     */
    public SysIdRoutine getRoutine() {
        return routine;
    }

    // Simulation
    private Notifier simNotifier; // Notifier for simulation thread
    private double lastSimTime; // Last simulation time

    /**
     * Starts the simulation thread.
     */
    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        // Run simulation at a faster rate for better PID behavior
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            // Update simulation state with measured time delta and battery voltage
            swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SwerveConstants.simLoopPeriod);
    }
}