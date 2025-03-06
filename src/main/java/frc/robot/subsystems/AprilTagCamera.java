package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Robot;
import frc.robot.other.RobotUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@code AprilTagCamera} class represents a subsystem that uses a PhotonCamera
 * to detect AprilTags on the field and estimate the robot's pose (position and orientation).
 * It leverages the PhotonVision library for AprilTag detection and pose estimation.
 */
public class AprilTagCamera extends SubsystemBase {
    // PhotonCamera object to interact with the camera
    private PhotonCamera cam;
    // PhotonPoseEstimator to estimate the robot's pose using AprilTags
    private PhotonPoseEstimator estimator;
    // Transform3d representing the transformation from the robot's center to the camera
    private Transform3d robotToCamera;
    // List to store estimated robot poses
    private List<EstimatedRobotPose> estimatedPoses;

    private String name;

    /**
     * Constructs an {@code AprilTagCamera} subsystem.
     *
     * @param name           The name of the camera (used to identify the camera in PhotonVision).
     * @param robotToCamera  The transformation from the robot's center to the camera's position.
     * @param fieldLayout    The layout of AprilTags on the field, used for pose estimation.
     */
    public AprilTagCamera(String name, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout) {
        this.robotToCamera = robotToCamera;
        // Initialize the PhotonCamera with the given name
        cam = new PhotonCamera(name);
        this.name=name;
        // Initialize the PhotonPoseEstimator with the field layout, pose strategy, and robot-to-camera transform
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        // Initialize the list to store estimated poses
        estimatedPoses = new ArrayList<>();
    }

    /**
     * Configures and starts the simulation for the camera. This method sets up the camera's
     * properties, such as resolution, field of view, FPS, and latency, and adds the simulated
     * camera to the vision system simulation.
     *
     * @param sim The {@link VisionSystemSim} object used to simulate the vision system.
     */
    public void startSim(VisionSystemSim sim) {
        // Create simulation camera properties
        SimCameraProperties cameraProp = new SimCameraProperties();
        // Set the camera calibration (resolution and field of view)
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Set the frames per second (FPS) for the camera
        cameraProp.setFPS(50);
        // Set the average latency in milliseconds
        cameraProp.setAvgLatencyMs(35);
        // Set the standard deviation of the latency in milliseconds
        cameraProp.setLatencyStdDevMs(5);
        // Create a simulated PhotonCamera using the camera and its properties
        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, cameraProp);
        // Add the simulated camera to the vision system simulation
        sim.addCamera(cameraSim, robotToCamera);
        // If the robot is in simulation mode, enable drawing the wireframe
        if (Robot.isSimulation()) {
            cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * Retrieves the list of newly estimated robot poses and clears the internal list
     * for future estimates. This method is typically called to process the latest pose
     * estimates.
     *
     * @return A list of {@link EstimatedRobotPose} objects representing the newly estimated poses.
     */
    public List<EstimatedRobotPose> getNewPoses() {
        // Create a copy of the current estimated poses
        var res = new ArrayList<>(estimatedPoses);
        // Clear the list of estimated poses
        estimatedPoses.clear();
        // Return the copied list of poses
        return res;
    }

    /**
     * The {@code periodic} method is called regularly by the subsystem. It checks for
     * new camera results, updates the estimated robot pose using the PhotonPoseEstimator,
     * and stores the estimated poses in a list for later retrieval.
     */

    @Override
    public void periodic() {
        try{
            // Get all unread results from the camera
            var results = cam.getAllUnreadResults();
            
            // Iterate through each result
            for (PhotonPipelineResult res : results) {
                // Update the estimated pose using the current result
                var estimatedPose = estimator.update(res);
                // If the estimated pose is present, add it to the list
                if (estimatedPose.isPresent()) {
                    var target = res.getBestTarget();
                    if (target.getPoseAmbiguity() < 0.2) {
                        estimatedPoses.add(estimatedPose.get());
                    }
                }
            }
        }catch(Exception e){
            DataLogManager.log("Periodic error from camera "+name+": "+RobotUtils.getError(e));
        }
        
    }

}