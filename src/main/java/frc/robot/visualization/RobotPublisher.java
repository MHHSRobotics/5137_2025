package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.GamepieceConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.gamepieces.Gamepiece;
import frc.robot.gamepieces.Gamepieces;
import frc.robot.other.RobotUtils;
import frc.robot.states.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/**
 * Handles publishing robot state data to NetworkTables for visualization.
 * This class is responsible for converting robot state information into
 * 3D poses that can be visualized in tools like AdvantageScope.
 * It also handles gamepiece simulation for visualization.
 */
public class RobotPublisher extends SubsystemBase {
    
    // Current wrist pose for tracking
    private Pose3d currentWristPose;
    
    // Subsystems to monitor
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Swerve swerve;
    
    // Gamepiece handling
    private final Gamepieces gamepieces;
    private Gamepiece currentPiece;
    
    /**
     * Creates a new RobotPublisher.
     * 
     * @param arm The arm subsystem
     * @param elevator The elevator subsystem
     * @param wrist The wrist subsystem
     * @param swerve The swerve subsystem
     * @param gamepieces The gamepieces manager
     */
    public RobotPublisher(Arm arm, Elevator elevator, Wrist wrist, Swerve swerve, Gamepieces gamepieces) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.swerve = swerve;
        this.gamepieces = gamepieces;
        this.currentWristPose = null;
        this.currentPiece = null;
    }
    
    /**
     * Gets the current state of the robot.
     * 
     * @return The current robot state
     */
    public RobotState getState() {
        return new RobotState(
            arm == null ? null : arm.getMeasurement(),
            elevator == null ? null : elevator.getMeasurement(),
            wrist == null ? null : wrist.getMeasurement(),
            swerve == null ? null : swerve.getPose()
        );
    }
    
    /**
     * Gets the target state of the robot.
     * 
     * @return The target robot state
     */
    public RobotState getTargetState() {
        return new RobotState(
            arm == null ? null : arm.getGoal(),
            elevator == null ? null : elevator.getGoal(),
            wrist == null ? null : wrist.getGoal(),
            swerve == null ? null : swerve.getTargetPose()
        );
    }
    
    /**
     * Publishes the robot state to NetworkTables for visualization.
     * 
     * @param state The robot state to publish
     * @param path The path in NetworkTables to publish to
     * @param realBot Whether this is the real robot state (true) or a target/simulated state (false)
     * @return The calculated wrist pose, if available
     */
    public Pose3d publishData(RobotState state, String path, boolean realBot) {
        StructPublisher<Pose3d> botPosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/pose", Pose3d.struct);
        StructPublisher<Pose3d> firstStagePosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/elevatorFirst", Pose3d.struct);
        StructPublisher<Pose3d> secondStagePosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/elevatorSecond", Pose3d.struct);
        StructPublisher<Pose3d> armPosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/arm", Pose3d.struct);
        StructPublisher<Pose3d> wristPosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/wrist", Pose3d.struct);
        
        Pose3d wristPose = null;
        
        if (state.robotPosition != null) {
            // Use the raw blue alliance position for visualization
            Pose2d botPosition = state.robotPosition.bluePos();
            Pose3d botPose = new Pose3d(botPosition);
            botPosePublisher.set(botPose);
            
            if(state.elevatorPosition != null) {
                // Calculate elevator positions
                double elevatorHeight = state.elevatorPosition;
                Pose3d firstStagePose = botPose.plus(new Transform3d(0, 0, elevatorHeight/2, new Rotation3d()));
                firstStagePosePublisher.set(firstStagePose.relativeTo(botPose));
                
                // Calculate elevator end position
                Pose3d elevatorPose = botPose.plus(new Transform3d(0, 0, elevatorHeight, new Rotation3d()));
                secondStagePosePublisher.set(elevatorPose.relativeTo(botPose));
    
                if(state.armPosition != null) {
                    // Calculate arm pivot position (offset from elevator end)
                    double armAngle = state.armPosition;
                    Pose3d armPose = elevatorPose.plus(new Transform3d(SwerveConstants.armTransOffset, new Rotation3d(0, -armAngle, 0)));
    
                    // Publish arm pose
                    armPosePublisher.set(armPose.relativeTo(botPose));
    
                    if(state.wristPosition != null) {
                        // Calculate and publish wrist pose
                        double wristAngle = state.wristPosition;
                        wristPose = armPose.plus(new Transform3d(0, 0, ArmConstants.armLength, new Rotation3d(0, -wristAngle, 0)));
                        wristPosePublisher.set(wristPose.relativeTo(botPose));
                        
                        if(realBot) {
                            currentWristPose = wristPose;
                        }
                    }
                }
            }
        }
        
        return wristPose;
    }
    
    /**
     * Simulates picking up a coral gamepiece.
     */
    public void simCoralIntake() {
        if(gamepieces != null) {
            currentPiece = gamepieces.getClosestCoral(currentWristPose);
        }
    }
    
    /**
     * Simulates picking up an algae gamepiece.
     */
    public void simAlgaeIntake() {
        if(gamepieces != null) {
            currentPiece = gamepieces.getClosestAlgae(currentWristPose);
        }
    }
    
    /**
     * Simulates releasing a coral gamepiece.
     */
    public void simCoralOuttake() {
        if(currentPiece != null) {
            currentPiece.setGoalOnBot(currentWristPose.plus(new Transform3d(-GamepieceConstants.coralDrop, 0, 0, new Rotation3d())));
            currentPiece = null;
        }
    }
    
    /**
     * Simulates releasing an algae gamepiece.
     */
    public void simAlgaeOuttake() {
        if(currentPiece != null) {
            currentPiece.setGoalOnBot(currentWristPose.plus(new Transform3d(-GamepieceConstants.algaeDrop, 0, 0, new Rotation3d())));
            currentPiece = null;
        }
    }
    
    @Override
    public void periodic() {
        // Publish current and target states
        publishData(getState(), "sim/real", true);
        publishData(getTargetState(), "sim/target", false);
        
        // Update gamepiece position if one is being held
        if(currentPiece != null) {
            currentPiece.setGoalOnBot(currentWristPose);
        }
    }
} 