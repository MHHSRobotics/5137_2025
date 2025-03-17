package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

/**
 * The Vision subsystem is responsible for managing the robot's vision processing,
 * including AprilTag detection, object detection, and simulation updates.
 */
public class Vision extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout; // Layout of AprilTags on the field

    private AprilTagCamera[] aprilTagCameras; // Array of cameras used for AprilTag detection

    private VisionSystemSim visionSim; // Simulation object for vision system

    /**
     * Constructor for the Vision subsystem.
     *
     * @param reef The Reef object used for tracking coral placements.
     */
    public Vision() {
        // Load the AprilTag field layout from the default field
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Initialize AprilTag cameras with their respective positions and the field layout
        AprilTagCamera blatCamera = new AprilTagCamera("BackLeft_AT", VisionConstants.robotToBLATCamera, fieldLayout);
        AprilTagCamera bratCamera = new AprilTagCamera("BackRight_AT", VisionConstants.robotToBRATCamera, fieldLayout);
        AprilTagCamera flatCamera = new AprilTagCamera("FrontLeft_AT", VisionConstants.robotToFLATCamera, fieldLayout);
        aprilTagCameras = new AprilTagCamera[]{blatCamera, flatCamera};

        // Initialize the vision system simulation and add AprilTags to it
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        // Start simulation for each AprilTag camera
        for (AprilTagCamera cam : aprilTagCameras) {
            cam.startSim(visionSim);
        }
    }

    /**
     * Retrieves new estimated robot poses from all AprilTag cameras.
     *
     * @return A list of EstimatedRobotPose objects representing the new poses.
     */
    public List<EstimatedRobotPose> getNewPoses() {
        var res = new ArrayList<EstimatedRobotPose>();
        for (AprilTagCamera cam : aprilTagCameras) {
            res.addAll(cam.getNewPoses());
        }
        return res;
    }

    /**
     * Updates the vision system simulation with the current robot pose.
     *
     * @param currentPose The current pose of the robot.
     */
    public void updateSim(Pose2d currentPose) {
        visionSim.update(currentPose);
    }
}