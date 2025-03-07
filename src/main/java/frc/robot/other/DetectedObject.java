package frc.robot.other;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.VisionConstants;

/**
 * Represents a detected object in 3D space, including its position and classification.
 */
public class DetectedObject {
    // The 3D position of the detected object.
    private Pose3d pose;

    // The class ID of the detected object, which corresponds to a specific type or category.
    private int classId;

    private double detectionTime;

    /**
     * Constructs a new DetectedObject with the specified position and class ID.
     *
     * @param pose The 3D position of the detected object.
     * @param classId The class ID of the detected object.
     */
    public DetectedObject(Pose3d pose, int classId, double detectionTime) {
        this.pose = pose;
        this.classId = classId;
        this.detectionTime = detectionTime;
    }

    /**
     * Returns the 3D position of the detected object.
     *
     * @return The 3D position of the detected object.
     */
    public Pose3d getPose() {
        return pose;
    }

    /**
     * Returns the class ID of the detected object.
     *
     * @return The class ID of the detected object.
     */
    public int getClassId() {
        return classId;
    }

    /**
     * Returns the detection time of the object.
     *
     * @return The detection time of the object.
     */
    public double getDetectionTime() {
        return detectionTime;
    }

    /**
     * Returns the class name of the detected object based on its class ID.
     * The class name is retrieved from the VisionConstants.classNames array.
     *
     * @return The class name of the detected object.
     */
    public String getClassName() {
        return VisionConstants.classNames[classId];
    }
}