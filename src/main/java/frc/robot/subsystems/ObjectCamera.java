package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.other.DetectedObject;
import frc.robot.other.RobotUtils;

/**
 * The ObjectCamera subsystem is responsible for interfacing with a PhotonVision camera
 * to detect and track objects in the robot's environment. It processes the camera's output
 * to determine the 3D positions of detected objects relative to the robot.
 */
public class ObjectCamera extends SubsystemBase {
    private PhotonCamera cam; // The PhotonVision camera used for object detection
    private Transform3d robotToCamera; // The transformation from the robot's center to the camera's position
    private List<PhotonTrackedTarget> newTargets; // A list of newly detected targets
    private StringLogEntry log;

    /**
     * Constructs an ObjectCamera subsystem.
     *
     * @param name The name of the PhotonVision camera.
     * @param robotToCamera The transformation from the robot's center to the camera's position.
     */
    public ObjectCamera(String name, Transform3d robotToCamera,StringLogEntry log) {
        this.robotToCamera = robotToCamera;
        this.cam = new PhotonCamera(name);
        this.newTargets = new ArrayList<>();
        this.log=log;
    }

    /**
     * Retrieves a list of detected objects, converting their positions from camera-relative
     * to robot-relative coordinates.
     *
     * @param robotPose The current pose of the robot in 2D space.
     * @return A list of DetectedObject instances, each representing an object detected by the camera.
     */
    public List<DetectedObject> getNewObjects(Pose2d robotPose) {
        var detectedObjects = new ArrayList<DetectedObject>();

        // Convert the robot's 2D pose to a 3D pose
        Pose3d robotPose3d = new Pose3d(robotPose);

        // Process each newly detected target
        for (PhotonTrackedTarget target : newTargets) {
            // Calculate the target's pose relative to the robot
            Pose3d targetPose = robotPose3d.transformBy(robotToCamera).transformBy(target.getBestCameraToTarget());

            // Get the class ID of the detected object (if using object detection)
            int classId = target.objDetectId;

            log.append("Detected object with transform "+targetPose+" id "+classId);
            // Add the detected object to the list
            detectedObjects.add(new DetectedObject(targetPose, classId, Utils.getCurrentTimeSeconds()));
        }

        return detectedObjects;
    }

    /**
     * The periodic method is called periodically by the WPILib scheduler. It updates the list
     * of newly detected targets by reading unprocessed results from the camera.
     */
    
    @Override
    public void periodic() {
        try{
            // Retrieve all unprocessed results from the camera
            var results = cam.getAllUnreadResults();

            // Add all detected targets to the newTargets list
            for (PhotonPipelineResult res : results) {
                newTargets.addAll(res.getTargets());
            }
        }catch(Exception e){
            log.append("Periodic error: "+RobotUtils.getError(e));
        }
    }
        
}