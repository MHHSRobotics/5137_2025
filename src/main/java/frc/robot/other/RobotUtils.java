package frc.robot.other;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.constants.FieldGeometry;
import frc.robot.states.RobotState;

/**
 * Utility class for robot-related operations, such as alliance-specific transformations,
 * pose manipulations, and perspective adjustments.
 */
public class RobotUtils {

    /**
     * Checks if the robot is on the red alliance.
     *
     * @return True if the robot is on the red alliance, false otherwise.
     */
    public static boolean onRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /**
     * Inverts a given Pose2d across the field's center. This is useful for transforming
     * poses from one alliance's perspective to the other.
     *
     * @param pose The pose to invert.
     * @return The inverted pose.
     */
    public static Pose2d invertPose(Pose2d pose) {
        return new Pose2d(
            FieldGeometry.fieldLength - pose.getX(),
            FieldGeometry.fieldWidth - pose.getY(),
            pose.getRotation().rotateBy(Rotation2d.k180deg)
        );
    }

    /**
     * Inverts a given Pose2d to the current alliance's perspective. If the robot is on the red alliance,
     * the pose is inverted; otherwise, it is returned as-is.
     *
     * @param pose The pose to invert.
     * @return The pose transformed to the current alliance's perspective.
     */
    public static Pose2d invertToAlliance(Pose2d pose) {
        if (onRedAlliance()) {
            return invertPose(pose);
        } else {
            return pose;
        }
    }

    /**
     * Inverts a given Translation3d across the field's center. This is useful for transforming
     * 3D translations from one alliance's perspective to the other.
     *
     * @param trans The 3D translation to invert.
     * @return The inverted 3D translation.
     */
    public static Pose3d invertPose(Pose3d pose) {
        return new Pose3d(
            FieldGeometry.fieldLength - pose.getX(),
            FieldGeometry.fieldWidth - pose.getY(),
            pose.getZ(),
            pose.getRotation().rotateBy(new Rotation3d(Rotation2d.k180deg))
        );
    }

    /**
     * Inverts a given Translation3d to the current alliance's perspective. If the robot is on the red alliance,
     * the translation is inverted; otherwise, it is returned as-is.
     *
     * @param trans The 3D translation to invert.
     * @return The 3D translation transformed to the current alliance's perspective.
     */
    public static Pose3d invertToAlliance(Pose3d pose) {
        if (onRedAlliance()) {
            return invertPose(pose);
        } else {
            return pose;
        }
    }

    /**
     * Determines the forward direction based on the robot's perspective. In simulation, the forward
     * direction is 90 degrees counterclockwise. On the red alliance, the forward direction is 180 degrees.
     * On the blue alliance, the forward direction is 0 degrees.
     *
     * @return The forward direction as a Rotation2d.
     */
    public static Rotation2d getPerspectiveForward() {
        if (Robot.isSimulation()) {
            return Rotation2d.kCCW_90deg;
        } else if (onRedAlliance()) {
            return Rotation2d.k180deg;
        } else {
            return Rotation2d.kZero;
        }
    }

    /**
     * Finds the closest Pose2d from a list of poses to a given reference pose. The reference pose
     * is first transformed to the current alliance's perspective before calculating distances.
     *
     * @param pose The reference pose.
     * @param others An array of poses to compare against.
     * @return The closest pose from the list.
     */
    public static Pose2d getClosestPoseToPose(Pose2d pose, Pose2d[] others) {
        Pose2d closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Pose2d other : others) {
            Transform2d transform = pose.minus(other);
            double distance = Math.hypot(transform.getX(), transform.getY());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = other;
            }
        }
        return closest;
    }

    // Converts an exception to an error string
    public static String getError(Exception e){
        StringBuilder sb=new StringBuilder();
        sb.append(e.getMessage());
        sb.append("\nBegin stack trace\n");
        for (var ste:e.getStackTrace()) {
            sb.append(ste);
            sb.append("\n");
        }
        sb.append("End stack trace");
        return sb.toString();
    }

    public static double clamp(double in,double min,double max){
        if(in<min){
            return min;
        }
        if(in>max){
            return max;
        }
        return in;
    }

    /**
     * Calculates the weighted distance between two poses, considering both translation and rotation.
     * 
     * @param pose1 The first pose
     * @param pose2 The second pose
     * @param rotationWeight The weight to apply to rotational differences (in meters per radian)
     * @return The weighted distance between the poses
     */
    public static double getWeightedPoseDistance(Pose2d pose1, Pose2d pose2, double rotationWeight) {
        if (pose1 == null || pose2 == null) {
            return Double.MAX_VALUE;
        }
        
        // Calculate translational distance
        double translationDistance = pose1.getTranslation().getDistance(pose2.getTranslation());
        
        // Calculate rotational distance (in radians)
        double rotationDistance = Math.abs(pose1.getRotation().minus(pose2.getRotation()).getRadians());
        
        // Apply weight to rotation and combine with translation
        return translationDistance + rotationWeight * rotationDistance;
    }

    /**
     * Find the closest RobotState from an array of states based on robot position.
     * 
     * @param currentPose The current pose to compare against
     * @param states Array of possible states
     * @return The closest state based on robot position, or null if states is empty
     */
    public static RobotState getClosestState(Pose2d currentPose, RobotState[] states) {
        if (states == null || states.length == 0) {
            return null;
        }
        
        // Find the closest state based on robot position
        RobotState closestState = states[0];
        double minDistance = Double.MAX_VALUE;
        
        for (RobotState state : states) {
            // Calculate distance to this state
            if (state.robotPosition != null) {
                Pose2d botPosition = state.robotPosition.alliancePos();
                double distance = getWeightedPoseDistance(currentPose, botPosition, 1.0);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestState = state;
                }
            }
        }

        return closestState;
    }

    /**
     * Find the closest RobotState from an array of states based on robot position,
     * using the specified rotation weight.
     * 
     * @param currentPose The current pose to compare against
     * @param states Array of possible states
     * @param rotationWeight The weight to apply to rotational differences (in meters per radian)
     * @return The closest state based on robot position, or null if states is empty
     */
    public static RobotState getClosestState(Pose2d currentPose, RobotState[] states, double rotationWeight) {
        if (states == null || states.length == 0) {
            return null;
        }
        
        // Find the closest state based on robot position
        RobotState closestState = states[0];
        double minDistance = Double.MAX_VALUE;
        
        for (RobotState state : states) {
            // Calculate distance to this state
            if (state.robotPosition != null) {
                Pose2d botPosition = state.robotPosition.alliancePos();
                double distance = getWeightedPoseDistance(currentPose, botPosition, rotationWeight);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestState = state;
                }
            }
        }

        return closestState;
    }

    @SuppressWarnings("all")
    private static HashMap<String,StructPublisher> map=new HashMap<>();

    @SuppressWarnings("all")
    public static<T> StructPublisher<T> getPublisher(String name,Struct<T> struct){
        if(map.containsKey(name)){
            return (StructPublisher<T>) map.get(name);
        }else{
            StructPublisher<T> pub=NetworkTableInstance.getDefault().getStructTopic(name, struct).publish();
            map.put(name,pub);
            return pub;
        }
    }

    public static double mod(double a,double b){
        double result = a % b;
        if (result < 0) {
            result += b;
        }
        return result;
    }

    public static int mod(int a,int b){
        int result = a % b;
        if (result < 0) {
            result += b;
        }
        return result;
    }
}