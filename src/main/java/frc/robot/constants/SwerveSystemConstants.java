package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.elastic.Reef;

/**
 * Constants for coordinated positions of the arm, elevator, and wrist mechanisms.
 * This class centralizes all the goal positions for the entire arm system.
 */
public final class SwerveSystemConstants {
    public static final double timeout = 1; // seconds
    
    // Weight for how much to consider rotation vs translation when finding closest state
    // Higher values mean rotation differences matter more
    public static final double rotationWeight = 0.0; // meters per radian
    // 0.32 arm source
    // 0.48 arm L3
    // elevator L2 0.15

    private static final double wristUp = Units.degreesToRadians(90);
    private static final double wristDown = Units.degreesToRadians(-90);      // 0 - 90 = -90 (horizontal)

    // Basic states without specific robot positions
    private static final SwerveSystem.SwerveSystemState baseGroundIntake = new SwerveSystem.SwerveSystemState(
        -2.0,  // From ArmConstants.groundIntakeGoal
        0.41,                       // From ElevatorConstants.groundIntakeGoal
        -1.39,                  // From WristConstants.pos1 (down)
        null,                        // Robot position determined at runtime
        null
    );   

    private static final SwerveSystem.SwerveSystemState baseDefaultState = new SwerveSystem.SwerveSystemState(
        Units.degreesToRadians(0),  // From ArmConstants.defaultGoal (vertical)
        0.05,                         // From ElevatorConstants.defaultGoal
        wristUp,                   // From WristConstants.pos1 (down)
        null,                         // Robot position determined at runtime
        null
    );

    private static final SwerveSystem.SwerveSystemState bargeState = new SwerveSystem.SwerveSystemState(
        Units.degreesToRadians(0),  // From ArmConstants.defaultGoal (vertical)
        1.11,                         // From ElevatorConstants.defaultGoal
        Units.degreesToRadians(-45),                   // From WristConstants.pos1 (down)
        null,                         // Robot position determined at runtime
        null
    );

    // Offset for the arm pivot in AdvantageScope simulation
    public static final Translation3d armTransOffset = new Translation3d(0.11,-0.18,0.26);

    public static final double intakeDistance = 1;

    public static SwerveSystem.SwerveSystemState getGroundIntake() {
        return baseGroundIntake;
    }

    public static SwerveSystem.SwerveSystemState getDefaultState() {
        return baseDefaultState;
    }

    public static SwerveSystem.SwerveSystemState getBargeState() {
        return bargeState;
    }

    public static SwerveSystem.SwerveSystemState getProcessor() {
        return new SwerveSystem.SwerveSystemState(
            Units.degreesToRadians(-95),  // 75 - 90 = -15 degrees
            0.65,
            Units.degreesToRadians(-45),
            RobotPositions.processor.alliancePos(),
            null
        );
    }

    public static SwerveSystem.SwerveSystemState[] getSourceStates() {
        SwerveSystem.SwerveSystemState[] states = new SwerveSystem.SwerveSystemState[RobotPositions.stations.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveSystem.SwerveSystemState(
                -0.43,  // 45 - 90 = -45 degrees
                0.23,                        // From ElevatorConstants.sourceGoal
                Units.degreesToRadians(-113.6),              // From WristConstants.pos2 (straight)
                RobotPositions.stations[i].alliancePos(),
                //"Source "+(char)('A'+i)
                null
            );
        }
        return states;
    }

    public static SwerveSystem.SwerveSystemState[] getAlgaeStates() {
        SwerveSystem.SwerveSystemState[] states = new SwerveSystem.SwerveSystemState[FieldGeometry.reefSides];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveSystem.SwerveSystemState(
                Units.degreesToRadians(-55),  // 30 - 90 = -60, 120 - 90 = 30
                Reef.isAlgaeLow(i) ? 0.43 : 0.9,
                Units.degreesToRadians(-115),                             // From WristConstants.minAngle
                RobotPositions.centerReef[i].alliancePos(),
                null
            );
        }
        return states;
    }

    public static SwerveSystem.SwerveSystemState[][] getScoringStates() {
        SwerveSystem.SwerveSystemState[][] states = new SwerveSystem.SwerveSystemState[4][FieldGeometry.reefSides*2];
        double[] armAngles = {
            0.66,   // L1 (135 - 90)
            0.34,   // L2 (135 - 90)
            0.34,   // L3 (135 - 90)
            0.4    // L4 (120 - 90)
        };
        double[] elevatorHeights = {
            0.01,  // L1
            0.06,  // L2
            0.46,  // L3
            1.11   // L4
        };
        double[] wristAngles = {
            0.1,   // L1 (135 - 90)
            0.1,   // L2 (135 - 90)
            0.1,   // L3 (135 - 90)
            0.15    // L4 (120 - 90)
        };

        for (int level = 0; level < states.length; level++) {
            for (int pos = 0; pos < states[level].length; pos++) {
                states[level][pos] = new SwerveSystem.SwerveSystemState(
                    armAngles[level],
                    elevatorHeights[level],
                    wristAngles[level],
                    RobotPositions.branchReef[pos].alliancePos(),
                    //"Reef "+(char)('A'+pos)
                    null
                );
            }
        }
        return states;
    }
} 