package frc.robot.positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.positions.RobotPositions.RobotPosition;

/**
 * Represents a complete state of the robot, including positions for
 * the arm, elevator, wrist, and the robot's position on the field.
 */
public class RobotState {
    public static final RobotState NULL = new RobotState(null,null,null, (RobotPosition) null);
    public final Double armPosition;
    public final Double elevatorPosition;
    public final Double wristPosition;
    public final RobotPosition robotPosition;
    public final boolean autoAlign;

    /**
     * Constructs a new RobotState with the specified positions.
     *
     * @param armPosition      The position of the arm in radians.
     * @param elevatorPosition The position of the elevator in meters.
     * @param wristPosition    The position of the wrist in radians.
     * @param robotPath        The path the robot will take.
     */
    public RobotState(Double armPosition, Double elevatorPosition, Double wristPosition, Rotation2d rotation) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.robotPosition = new RobotPosition(new Pose2d(new Translation2d(), rotation));
        autoAlign = false;
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
        this.autoAlign = true;
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
        this.autoAlign = true;
    }

    /**
     * Constructs a new RobotState with the specified positions.
     *
     * @param armPosition      The position of the arm in radians.
     * @param elevatorPosition The position of the elevator in meters.
     * @param wristPosition    The position of the wrist in radians.
     * @param robotPosition    The position of the robot on the field.
     */
    public RobotState(Double armPosition, Double elevatorPosition, Double wristPosition, RobotPosition robotPosition, boolean autoAlign) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.robotPosition = robotPosition;
        this.autoAlign = autoAlign;
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
    public RobotState(Double armPosition, Double elevatorPosition, Double wristPosition, Pose2d botPosition, boolean autoAlign) {
        this.armPosition = armPosition;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.robotPosition = botPosition != null ? new RobotPosition(botPosition) : null;
        this.autoAlign = autoAlign;
    }

    public RobotState noElevator() {
        return new RobotState(armPosition, null, wristPosition, robotPosition, autoAlign);
    }

    public RobotState onlyElevator() {
        return new RobotState(null, elevatorPosition, null, robotPosition, autoAlign);
    }

    public RobotState withElevator(double height) {
        return new RobotState(armPosition, height, wristPosition, (RobotPosition) null, autoAlign);
    }

    public RobotState noArm() {
        return new RobotState(null, elevatorPosition, wristPosition, robotPosition, autoAlign);
    }

    public RobotState onlyArm() {
        return new RobotState(armPosition, null, null, robotPosition, autoAlign);
    }

    public RobotState withArm(double angle) {
        return new RobotState(angle, elevatorPosition, wristPosition, robotPosition, autoAlign);
    }

    public RobotState noWrist() {
        return new RobotState(armPosition, elevatorPosition, null, robotPosition, autoAlign);
    }

    public RobotState onlyWrist() {
        return new RobotState(null, null, wristPosition, robotPosition, autoAlign);
    }

    public RobotState withWrist(double angle) {
        return new RobotState(armPosition, elevatorPosition, angle, robotPosition, autoAlign);
    }

    public RobotState noPosition() {
        return new RobotState(armPosition, elevatorPosition, wristPosition, (RobotPosition) null, autoAlign);
    }

    public RobotState onlyPosition() {
        return new RobotState(null, null, null, robotPosition, autoAlign);
    }

    /**
     * Creates a new RobotState with the same arm, elevator, and wrist positions,
     * but with a different robot path.
     *
     * @param path The new robot path.
     * @return A new RobotState with the updated robot position.
     */
    public RobotState withRotation(Rotation2d rotation) {
        return new RobotState(armPosition, elevatorPosition, wristPosition, rotation);
    }

    /**
     * Creates a new RobotState with the same arm, elevator, and wrist positions,
     * but with a different robot position.
     *
     * @param position The new robot position.
     * @return A new RobotState with the updated robot position.
     */
    public RobotState withPosition(RobotPosition position) {
        return new RobotState(armPosition, elevatorPosition, wristPosition, position, autoAlign);
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