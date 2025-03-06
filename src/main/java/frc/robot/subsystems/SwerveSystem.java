package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.GamepieceConstants;
import frc.robot.constants.SwerveSystemConstants;
import frc.robot.gamepieces.Gamepiece;
import frc.robot.gamepieces.Gamepieces;
import frc.robot.other.RobotUtils;

/**
 * The `ArmMechanism` class is responsible for visualizing the arm, elevator, and wrist mechanisms
 * using a 2D mechanism representation on the SmartDashboard. This class extends `SubsystemBase`
 * and updates the visualization periodically based on the current state of the arm, elevator, and wrist.
 */
public class SwerveSystem extends SubsystemBase {
    /**
     * Represents a complete state of the arm system, including positions for
     * the arm, elevator, and wrist.
     */
    public static class SwerveSystemState {
        public final Double armPosition;
        public final Double elevatorPosition;
        public final Double wristPosition;
        public final Pose2d botPosition;

        public SwerveSystemState(Double armPosition, Double elevatorPosition, Double wristPosition, Pose2d botPosition) {
            this.armPosition = armPosition;
            this.elevatorPosition = elevatorPosition;
            this.wristPosition = wristPosition;
            this.botPosition = botPosition;
        }

        public SwerveSystemState withPose(Pose2d pose){
            return new SwerveSystemState(armPosition, elevatorPosition, wristPosition, pose);
        }
    }

    // Subsystems
    private Arm arm;        // The arm subsystem
    private Elevator elevator; // The elevator subsystem
    private Wrist wrist;    // The wrist subsystem
    private Swerve swerve; // The swerve subsystem

    private Pose3d currentWristPose;
    private Gamepieces gamepieces;
    private Gamepiece currentPiece;

    /**
     * Constructor for the `ArmMechanism` class.
     *
     * @param arm      The arm subsystem.
     * @param elevator The elevator subsystem.
     * @param wrist    The wrist subsystem.
     */
    public SwerveSystem(Arm arm, Elevator elevator, Wrist wrist, Swerve swerve, Gamepieces gamepieces) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        this.swerve = swerve;
        this.gamepieces = gamepieces;
    }

    private void publishData(SwerveSystemState state,String path,boolean realBot){
        StructPublisher<Pose3d> botPosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/pose", Pose3d.struct);
        StructPublisher<Pose3d> firstStagePosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/elevatorFirst", Pose3d.struct);
        StructPublisher<Pose3d> secondStagePosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/elevatorSecond", Pose3d.struct);
        StructPublisher<Pose3d> armPosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/arm", Pose3d.struct);
        StructPublisher<Pose3d> wristPosePublisher = RobotUtils.getPublisher("SmartDashboard/"+path+"/wrist", Pose3d.struct);
        if(state.botPosition!=null){
            Pose3d botPose=new Pose3d(state.botPosition);
            botPosePublisher.set(botPose);
            if(state.elevatorPosition!=null){
                // Calculate elevator positions
                double elevatorHeight = state.elevatorPosition;
                Pose3d firstStagePose=botPose.plus(new Transform3d(0, 0, elevatorHeight/2, new Rotation3d()));
                firstStagePosePublisher.set(firstStagePose.relativeTo(botPose));
                
                // Calculate elevator end position
                Pose3d elevatorPose = botPose.plus(new Transform3d(0, 0, elevatorHeight, new Rotation3d()));
                secondStagePosePublisher.set(elevatorPose.relativeTo(botPose));
    
                if(state.armPosition!=null){
                    // Calculate arm pivot position (offset from elevator end)
                    double armAngle = state.armPosition;
                    Pose3d armPose = elevatorPose.plus(new Transform3d(SwerveSystemConstants.armTransOffset,new Rotation3d(0,-armAngle,0)));
    
                    // Publish arm pose
                    armPosePublisher.set(armPose.relativeTo(botPose));
    
                    if(state.wristPosition!=null){
                        // Calculate and publish wrist pose
                        double wristAngle = state.wristPosition;
                        Pose3d wristPose = armPose.plus(new Transform3d(0, 0, ArmConstants.armLength, new Rotation3d(0,-wristAngle,0)));
                        wristPosePublisher.set(wristPose.relativeTo(botPose));
                        if(realBot){
                            currentWristPose = wristPose;
                        }
                    }
                }
            }
        }
    }

    public SwerveSystemState getState(){
        return new SwerveSystemState(
            arm==null?null:arm.getMeasurement(), 
            elevator==null?null:elevator.getMeasurement(), 
            wrist==null?null:wrist.getMeasurement(), 
            swerve==null?null:swerve.getPose()
        );
    }

    public SwerveSystemState getTargetState(){
        return new SwerveSystemState(
            arm==null?null:arm.getGoal(), 
            elevator==null?null:elevator.getGoal(), 
            wrist==null?null:wrist.getGoal(), 
            swerve==null?null:swerve.getTargetPose()
        );
    }

    public Pose3d getCurrentWristPose(){
        return currentWristPose;
    }

    public void coralIntake(){
        if(gamepieces!=null){
            currentPiece=gamepieces.getClosestCoral(getCurrentWristPose());
        }
    }

    public void algaeIntake(){
        if(gamepieces!=null){
            currentPiece=gamepieces.getClosestAlgae(getCurrentWristPose());
        }
    }

    public void outtakeCoral(){
        if(currentPiece!=null){
            currentPiece.setGoalOnBot(currentWristPose.plus(new Transform3d(-GamepieceConstants.coralDrop, 0, 0, new Rotation3d())));
            currentPiece=null;
        }
    }

    public void outtakeAlgae(){
        if(currentPiece!=null){
            currentPiece.setGoalOnBot(currentWristPose.plus(new Transform3d(-GamepieceConstants.algaeDrop, 0, 0, new Rotation3d())));
            currentPiece=null;
        }
    }
    /**
     * This method is called periodically (every 20ms by default) and updates the visualization
     * of the arm, elevator, and wrist mechanisms based on their current positions.
     */
    @Override
    public void periodic() {
        publishData(getState(),"sim/real",true);
        publishData(getTargetState(),"sim/target",false);
        if(currentPiece!=null){
            currentPiece.setGoalOnBot(currentWristPose);
        }
    }

    /**
     * Calculate the distance between two poses, including rotation
     * Uses a weighted sum of translational and rotational distance
     */
    private double calculateDistance(Pose2d pose1, Pose2d pose2) {
        return RobotUtils.getWeightedPoseDistance(pose1, pose2, SwerveSystemConstants.rotationWeight);
    }

    /**
     * Sets the arm system to a specific state.
     * 
     * @param state The state to set the arm system to.
     * @param targetPose The target robot pose to use if state's position is null
     */
    public void setTargetState(SwerveSystemState state) {
        if(arm!=null && state.armPosition != null){
            arm.setGoal(state.armPosition);
        }
        if(elevator!=null && state.elevatorPosition != null){
            elevator.setGoal(state.elevatorPosition);
        }
        if(wrist!=null && state.wristPosition != null){
            wrist.setGoal(state.wristPosition);
        }
        if(swerve!=null && state.botPosition != null){
            // Uncomment at your own risk
            //swerve.setTargetPose(state.botPosition);
        }
    }

    public SwerveSystemState getClosestState(SwerveSystemState[] states){
        if (states == null || states.length == 0) {
            throw new IllegalArgumentException("States array cannot be empty");
        }

        // Get current robot pose

        Pose2d currentPose = swerve==null?new Pose2d():swerve.getPose();
        
        // Find the closest state based on robot position
        SwerveSystemState closestState = states[0];
        double minDistance = Double.MAX_VALUE;
        
        for (SwerveSystemState state : states) {
            // Use targetPose if state's position is null
            double distance = calculateDistance(currentPose, state.botPosition);
            if (distance < minDistance) {
                minDistance = distance;
                closestState = state;
            }
        }

        return closestState;
    }

    /**
     * Checks if all mechanisms (arm, elevator, wrist) are at their setpoints.
     * 
     * @return true if all mechanisms are at their setpoints, false otherwise.
     */
    public boolean atSetpoint() {
        return (arm==null || arm.atSetpoint()) &&
            (elevator==null || elevator.atSetpoint()) &&
            (wrist==null || wrist.atSetpoint()) &&
            (swerve==null || swerve.atTarget());
    }
}