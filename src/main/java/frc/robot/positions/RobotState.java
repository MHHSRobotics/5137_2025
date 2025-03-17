package frc.robot.positions;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.positions.RobotPositions.RobotPosition;

/**
 * Represents a complete state of the robot, including positions for
 * the arm, elevator, wrist, and the robot's position on the field.
 */
public class RobotState {
    public static final RobotState NULL = new RobotState(null,null,null, (PathPlannerPath) null);
    public final Double armPosition;
    public final Double elevatorPosition;
    public final Double wristPosition;
    public final RobotPosition robotPosition;
    public final PathPlannerPath robotPath;

    /**
     * Constructs a new RobotState with the specified positions.
     *
     * @param armPosition      The position of the arm in radians.
     * @param elevatorPosition The position of the elevator in meters.
     * @param wristPosition    The position of the wrist in radians.
     * @param robotPath        The path the robot will take.
     */
    public RobotState(Double armPosition, Double elevatorPosition, Double wristPosition, PathPlannerPath robotPath) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.robotPosition = null;
        this.robotPath = robotPath;
    }

    /**
     * Constructs a new RobotState with the specified positions.
     *
     * @param armPosition      The position of the arm in radians.
     * @param elevatorPosition The position of the elevator in meters.
     * @param wristPosition    The position of the wrist in radians.
     * @param robotPosition    The position of the robot on the field.
     */
    public RobotState(Double armPosition, Double elevatorPosition, Double wristPosition, RobotPosition robotPosition) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.robotPosition = robotPosition;
        this.robotPath = null;
    }

    /**
     * Constructs a new RobotState with the specified positions, using a Pose2d for the robot position.
     * This constructor is provided for backward compatibility.
     *
     * @param armPosition      The position of the arm in radians.
     * @param elevatorPosition The position of the elevator in meters.
     * @param wristPosition    The position of the wrist in radians.
     * @param botPosition      The position of the robot on the field as a Pose2d.
     */
    public RobotState(Double armPosition, Double elevatorPosition, Double wristPosition, Pose2d botPosition) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.robotPosition = botPosition != null ? new RobotPosition(botPosition) : null;
        this.robotPath = null;
    }

    public RobotState stageOne() {
        return new RobotState(null, elevatorPosition, null, (RobotPosition) null);
    }

    public RobotState stageTwo() {
        return new RobotState(armPosition, null, wristPosition, (RobotPosition) null);
    }

    /**
     * Creates a new RobotState with the same arm, elevator, and wrist positions,
     * but with a different robot path.
     *
     * @param path The new robot path.
     * @return A new RobotState with the updated robot position.
     */
    public RobotState withPath(PathPlannerPath path) {
        return new RobotState(armPosition, elevatorPosition, wristPosition, path);
    }

    /**
     * Creates a new RobotState with the same arm, elevator, and wrist positions,
     * but with a different robot position.
     *
     * @param position The new robot position.
     * @return A new RobotState with the updated robot position.
     */
    public RobotState withPosition(RobotPosition position) {
        return new RobotState(armPosition, elevatorPosition, wristPosition, position);
    }

    /**
     * Creates a new RobotState with the same arm, elevator, and wrist positions,
     * but with a different robot position.
     *
     * @param pose The new robot position as a Pose2d.
     * @return A new RobotState with the updated robot position.
     */
    public RobotState withPose(Pose2d pose) {
        return withPosition(pose != null ? new RobotPosition(pose) : null);
    }
} 