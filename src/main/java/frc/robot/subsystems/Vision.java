package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldGeometry;
import frc.robot.constants.VisionConstants;
import frc.robot.elastic.Reef;
import frc.robot.other.DetectedObject;

/**
 * The Vision subsystem is responsible for managing the robot's vision processing,
 * including AprilTag detection, object detection, and simulation updates.
 */
public class Vision extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout; // Layout of AprilTags on the field

    private AprilTagCamera[] aprilTagCameras; // Array of cameras used for AprilTag detection
    private ObjectCamera[] objectCameras; // Array of cameras used for object detection

    private ArrayList<DetectedObject> objects;

    private VisionSystemSim visionSim; // Simulation object for vision system

    private Reef reef; // Reference to the Reef object for coral placement tracking

    /**
     * Constructor for the Vision subsystem.
     *
     * @param reef The Reef object used for tracking coral placements.
     */
    public Vision(Reef reef) {
        // Load the AprilTag field layout from the default field
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Initialize AprilTag cameras with their respective positions and the field layout
        //AprilTagCamera blatCamera = new AprilTagCamera("BackLeft_AT", VisionConstants.robotToBLATCamera, fieldLayout);
        AprilTagCamera bratCamera = new AprilTagCamera("BackRight_AT", VisionConstants.robotToBRATCamera, fieldLayout);
        AprilTagCamera flatCamera = new AprilTagCamera("FrontLeft_AT", VisionConstants.robotToFLATCamera, fieldLayout);
        aprilTagCameras = new AprilTagCamera[]{/*blatCamera, */bratCamera, flatCamera};

        // Initialize object cameras with their respective positions
        //ObjectCamera frontObjectCamera = new ObjectCamera("frontObjectCamera", VisionConstants.robotToFrontObjectCamera, new StringLogEntry(dataLog, "frontObjectCamera"));
        objectCameras = new ObjectCamera[]{};

        // Initialize the vision system simulation and add AprilTags to it
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        // Start simulation for each AprilTag camera
        for (AprilTagCamera cam : aprilTagCameras) {
            cam.startSim(visionSim);
        }

        this.reef = reef;
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
     * Retrieves new detected objects from all object cameras.
     *
     * @param robotPose The current pose of the robot.
     * @return A list of DetectedObject objects representing the new objects.
     */
    private List<DetectedObject> getNewObjects(Pose2d robotPose) {
        var res = new ArrayList<DetectedObject>();
        for (ObjectCamera cam : objectCameras) {
            res.addAll(cam.getNewObjects(robotPose));
        }
        return res;
    }

    /**
     * Processes new detected objects and updates the Reef if a coral is detected.
     *
     * @param robotPose The current pose of the robot.
     */
    public void processNewObjects(Pose2d robotPose) {
        List<DetectedObject> objects = getNewObjects(robotPose);
        for (DetectedObject object : objects) {
            // Only process objects classified as "Coral"
            if (!object.getClassName().equals("Coral")) {
                continue;
            }
            Translation3d coralPos = object.getPose().getTranslation();
            Pair<Integer, Integer> coralLoc = checkObjectOnReef(coralPos);
            if (coralLoc != null) {
                // Update the Reef to indicate that a coral has been placed
                reef.setCoralPlaced(coralLoc.getFirst(), coralLoc.getSecond(), true);
            }else{
                // Coral is on ground
                objects.add(object);
            }
        }
    }

    /**
     * Checks if a detected object is within the margin of error of any coral position on the Reef.
     *
     * @param target3d The 3D position of the detected object.
     * @return A Pair of integers representing the branch and level of the closest coral position,
     *         or null if no position is within the margin of error.
     */
    public Pair<Integer, Integer> checkObjectOnReef(Translation3d target3d) {
        double closestDist = VisionConstants.objectMarginOfError;
        int closestBranch = -1;
        int closestLevel = -1;
        for (int branch = 0; branch < FieldGeometry.coralPositions.length; branch++) {
            for (int level = 0; level < FieldGeometry.coralPositions[branch].length; level++) {
                Translation3d pos = FieldGeometry.coralPositions[branch][level].alliancePos().getTranslation();
                double dist = target3d.minus(pos).getNorm();
                if (dist < closestDist) {
                    closestDist = dist;
                    closestBranch = branch;
                    closestLevel = level;
                }
            }
        }
        if (closestBranch == -1) {
            return null;
        } else {
            DataLogManager.log("Detected coral on level "+(closestLevel+2)+" branch "+closestBranch);
            return Pair.of(closestLevel, closestBranch);
        }
    }

    /**
     * Updates the vision system simulation with the current robot pose.
     *
     * @param currentPose The current pose of the robot.
     */
    public void updateSim(Pose2d currentPose) {
        visionSim.update(currentPose);
    }

    public List<DetectedObject> getGroundCoral(double expirationTime){
        ArrayList<DetectedObject> relevantObjects=new ArrayList<>();
        double currentTime = Utils.getCurrentTimeSeconds();
        for (DetectedObject object : objects) {
            if(currentTime-object.getDetectionTime()<expirationTime){
                relevantObjects.add(object);
            }
        }
        return relevantObjects;
    }
}