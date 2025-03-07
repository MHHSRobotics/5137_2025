package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotPositions.RobotPosition;
import frc.robot.states.RobotState;
import static frc.robot.elastic.Reef.isAlgaeLow;

/**
 * Constants for coordinated positions of the arm, elevator, and wrist mechanisms.
 * This class centralizes all the goal positions for the entire robot.
 */
public final class SwerveSystemConstants {
    // General constants
    public static final double timeout = 1; // seconds
    
    // Weight for rotation vs translation when finding closest state
    public static final double rotationWeight = 1.0; // meters per radian

    // Wrist positions
    private static final double wristUp = Units.degreesToRadians(90);
    private static final double wristDown = Units.degreesToRadians(-90);

    // Offset from the center of the robot to the arm pivot
    public static final Translation3d armTransOffset = new Translation3d(0.11, -0.18, 0.26);
    
    // Distance for intake operations
    public static final double intakeDistance = 1;

    // Basic states without specific robot positions
    public static final RobotState groundIntake = new RobotState(
        -2.0,
        0.41,
        -1.39,
        (RobotPosition)null  // Robot position determined at runtime
    );   

    public static final RobotState defaultState = new RobotState(
        Units.degreesToRadians(0),
        0.05,
        wristUp,
        (RobotPosition)null  // Robot position determined at runtime
    );
    
    // Processor state
    public static final RobotState processor = new RobotState(
        Units.degreesToRadians(-95),
        0.65,
        Units.degreesToRadians(-45),
        RobotPositions.processor
    );
    
    // Source states
    public static final RobotState[] sourceStates;
    
    // Algae states
    public static final RobotState[] algaeStates;
    
    // Scoring states
    public static final RobotState[][] scoringStates;
    
    // Static initializers
    static {
        sourceStates = new RobotState[RobotPositions.stations.length];
        for (int i = 0; i < sourceStates.length; i++) {
            sourceStates[i] = new RobotState(
                -0.43,
                0.17,
                Units.degreesToRadians(-113.6),
                RobotPositions.stations[i]
            );
        }
    }
    
    static {
        algaeStates = new RobotState[FieldGeometry.reefSides];
        for (int i = 0; i < algaeStates.length; i++) {
            algaeStates[i] = new RobotState(
                Units.degreesToRadians(-55),
                isAlgaeLow(i) ? 0.43 : 0.9,
                Units.degreesToRadians(-115),
                RobotPositions.centerReef[i]
            );
        }
    }
    
    static {
        scoringStates = new RobotState[4][FieldGeometry.reefSides*2];
        double[] armAngles = {
            0.66,   // L1
            0.34,   // L2
            0.34,   // L3
            0.4     // L4
        };
        double[] elevatorHeights = {
            0.01,  // L1
            0.06,  // L2
            0.46,  // L3
            1.11   // L4
        };
        double[] wristAngles = {
            0.1,   // L1
            0.1,   // L2
            0.1,   // L3
            0.1    // L4
        };

        for (int level = 0; level < scoringStates.length; level++) {
            for (int pos = 0; pos < scoringStates[level].length; pos++) {
                scoringStates[level][pos] = new RobotState(
                    armAngles[level],
                    elevatorHeights[level],
                    wristAngles[level],
                    RobotPositions.branchReef[pos]
                );
            }
        }
    }
} 