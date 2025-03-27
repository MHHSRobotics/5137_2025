package frc.robot.positions;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.other.RobotUtils;

/**
 * Centralizes all robot positions and states for the entire robot.
 * This includes field positions, robot states, and mechanism configurations.
 */
public final class RobotPositions {
    /**
     * Represents a position on the field, with methods to convert between alliance perspectives.
     */
    public static class RobotPosition {
        private final Pose2d bluePosition;
        
        public RobotPosition(Pose2d bluePosition) {
            this.bluePosition = bluePosition;
        }
        
        public Pose2d bluePos() { return bluePosition; }
        public Pose2d redPos() { return RobotUtils.invertPose(bluePosition); }
        public Pose2d alliancePos() { 
            return RobotUtils.invertToAlliance(bluePosition);
        }
    }
    
    // ===== Reef geometry for robot positions =====
    private static final double d1 = 1.3;
    private static final double d2 = 0.165;
    private static final double dShift = -0.21;

    // ===== Robot Positions =====
    
    // Static robot positions
    public static final RobotPosition processor = new RobotPosition(
        new Pose2d(new Translation2d(6.25, 1), new Rotation2d(3*Math.PI/2))
    );

    public static final RobotPosition[] stations = generateStations();
    public static final RobotPosition[] pickups = generatePickups();
    public static final RobotPosition[] centerReef = {
        new RobotPosition(new Pose2d(new Translation2d(2.87, 4.03), new Rotation2d(Units.degreesToRadians(0)))),
        new RobotPosition(new Pose2d(new Translation2d(3.69, 2.63), new Rotation2d(Units.degreesToRadians(60)))),
        new RobotPosition(new Pose2d(new Translation2d(5.28, 2.63), new Rotation2d(Units.degreesToRadians(120)))),
        new RobotPosition(new Pose2d(new Translation2d(6.1, 4.03), new Rotation2d(Units.degreesToRadians(180)))),
        new RobotPosition(new Pose2d(new Translation2d(5.28, 5.43), new Rotation2d(Units.degreesToRadians(240)))),
        new RobotPosition(new Pose2d(new Translation2d(3.69, 5.43), new Rotation2d(Units.degreesToRadians(300))))
    };
    public static final RobotPosition[] branchReef = generateBranchReef();

    // ===== Robot States =====
    
    // Basic states without specific robot positions
    public static final RobotState defaultState = new RobotState(
        Units.degreesToRadians(0),
        0.,
        Units.degreesToRadians(90),
        (RobotPosition) null
    );

    public static final RobotState groundCoralTeleop = new RobotState(
        -2.0,
        0.35,
        -2.3,
        (RobotPosition) null
    );

    public static final RobotState groundCoralAuto = new RobotState(
        Units.degreesToRadians(-100),
        0.22,
        Units.degreesToRadians(-80),
        (RobotPosition) null
    );

    public static final RobotState groundAlgaeState = new RobotState(
        Units.degreesToRadians(-100),
        0.3,
        Units.degreesToRadians(-115),
        (RobotPosition) null
    );
    
    public static final RobotState processorState = new RobotState(
        Units.degreesToRadians(-95),
        0.5,
        Units.degreesToRadians(-115),
        (RobotPosition) null
        //new RobotPosition(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(-90))))
    );

    public static final RobotState[] bargeStates = {
        new RobotState(
            Units.degreesToRadians(-25),
            1.2,
            Units.degreesToRadians(-45),
            (RobotPosition) null),
            //new RobotPosition(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(180))))),
        new RobotState(
            Units.degreesToRadians(25),
            null,
            null,
            (RobotPosition) null)
    };

    public static final RobotState preScoringState = new RobotState(
        Units.degreesToRadians(0),
        1.1,
        Units.degreesToRadians(0),
        (RobotPosition)null
    );
    
    // Source states
    public static final RobotState[] sourceStates = generateSourceStates();
    
    // Algae states
    public static final RobotState[] algaeStates = generateAlgaeStates();
    
    // Scoring states
    private static final Pair<RobotState[][],RobotState[][]> scoringStates = generateScoringStates();
    public static final RobotState[][] scoringStatesVertical = scoringStates.getFirst();
    public static final RobotState[][] scoringStatesHorizontal = scoringStates.getSecond();
    
    

    // ===== Position Generators =====

    private static RobotPosition[] generateStations() {
        return new RobotPosition[] {
            new RobotPosition(new Pose2d(new Translation2d(1.8, 1), new Rotation2d(Units.degreesToRadians(234)))),      // RR
            new RobotPosition(new Pose2d(new Translation2d(0.975, 1.625), new Rotation2d(Units.degreesToRadians(234)))), // RL
            new RobotPosition(new Pose2d(new Translation2d(0.975, 6.555), new Rotation2d(Units.degreesToRadians(126)))), // LR
            new RobotPosition(new Pose2d(new Translation2d(1.8, 7.05), new Rotation2d(Units.degreesToRadians(126))))     // LL
        };
    }

    private static RobotPosition[] generatePickups() {
        return new RobotPosition[] {
            new RobotPosition(new Pose2d(new Translation2d(1.8, 5.825), new Rotation2d(0.0))), // left
            new RobotPosition(new Pose2d(new Translation2d(1.8, 4.025), new Rotation2d(0.0))), // center
            new RobotPosition(new Pose2d(new Translation2d(1.8, 2.225), new Rotation2d(0.0)))  // right
        };
    }

    private static RobotPosition[] generateBranchReef() {
        RobotPosition[] positions = new RobotPosition[FieldPositions.reefSides * 2];
        
        for(int i = 0; i < FieldPositions.reefSides; i++) {
            Rotation2d forward = new Rotation2d(2*i*Math.PI/FieldPositions.reefSides).rotateBy(Rotation2d.k180deg);
            Translation2d outer = FieldPositions.reefCenter.plus(new Translation2d(d1, forward));
            Rotation2d sidewaysAngle = forward.rotateBy(Rotation2d.kCW_90deg);
            Translation2d algae = outer.plus(new Translation2d(dShift, sidewaysAngle));
            
            Translation2d left = algae.plus(new Translation2d(d2, sidewaysAngle));
            positions[2*i] = new RobotPosition(new Pose2d(left, forward));
            
            Translation2d right = algae.plus(new Translation2d(-d2, sidewaysAngle));
            positions[2*i+1] = new RobotPosition(new Pose2d(right, forward));
        }
        
        return positions;
    }
    
    // ===== State Generators =====
    
    private static RobotState[] generateSourceStates() {
        RobotState[] states = new RobotState[2];
        states[0] = new RobotState(
            -0.43,
            0.17,
            Units.degreesToRadians(-113.6),
            (RobotPosition) null);
        states[1] = new RobotState(
            -0.43,
            0.17,
            Units.degreesToRadians(-113.6),
            (RobotPosition) null);
        return states;
    }
    
    private static RobotState[] generateAlgaeStates() {
        RobotState[] states = new RobotState[FieldPositions.reefSides];
        for (int i = 0; i < states.length; i++) {
            states[i] = new RobotState(
                Units.degreesToRadians(-45),
                (i%2==0) ? 0.45 : 0.1,
                Units.degreesToRadians(-90),
                centerReef[i]
            );
        }
        return states;
    }
    
    private static Pair<RobotState[][], RobotState[][]> generateScoringStates() {
        RobotState[][] verticalStates = new RobotState[4][12];
        RobotState[][] horizontalStates = new RobotState[4][12];
        double[] armAngles = {
            0.61, 0.34, 0.34, 0.225, // Vertical Coral Scoring
            0.61, -0.2, -0.2, 0.75 // Horizontal Coral Scoring
        };
        double[] elevatorHeights = {
            0.01, 0.1, 0.47, 1.2, // Vertical Coral Scoring
            0.0, 0.02, 0.39, 0.9 // Horizontal Coral Scoring
        };
        double[] wristAngles = {
            -0.61, 0.18, 0.18, 0.54, // Vertical Coral Scoring
            0.3, -2.14, -2.14, -0.95 // Horizontal Coral Scoring
        };
        RobotPosition[] positions = {
            new RobotPosition(new Pose2d(new Translation2d(3.21, 3.96), new Rotation2d(Units.degreesToRadians(180)))),
            new RobotPosition(new Pose2d(new Translation2d(3.21, 3.64), new Rotation2d(Units.degreesToRadians(180)))),
            new RobotPosition(new Pose2d(new Translation2d(3.895, 2.895), new Rotation2d(Units.degreesToRadians(240)))),
            new RobotPosition(new Pose2d(new Translation2d(4.207, 2.726), new Rotation2d(Units.degreesToRadians(240)))),
            new RobotPosition(new Pose2d(new Translation2d(5.168, 2.953), new Rotation2d(Units.degreesToRadians(300)))),
            new RobotPosition(new Pose2d(new Translation2d(5.472, 3.072), new Rotation2d(Units.degreesToRadians(300)))),
            new RobotPosition(new Pose2d(new Translation2d(5.78, 4.1), new Rotation2d(Units.degreesToRadians(0)))),
            new RobotPosition(new Pose2d(new Translation2d(5.78, 4.41), new Rotation2d(Units.degreesToRadians(0)))),
            new RobotPosition(new Pose2d(new Translation2d(5.083, 5.159), new Rotation2d(Units.degreesToRadians(60)))),
            new RobotPosition(new Pose2d(new Translation2d(4.83, 5.337), new Rotation2d(Units.degreesToRadians(60)))),
            new RobotPosition(new Pose2d(new Translation2d(3.804, 5.108), new Rotation2d(Units.degreesToRadians(120)))),
            new RobotPosition(new Pose2d(new Translation2d(3.518, 4.941), new Rotation2d(Units.degreesToRadians(120))))
        };
        RobotPosition[] altPositions = {
            new RobotPosition(new Pose2d(new Translation2d(3.21, 4.4), new Rotation2d(Units.degreesToRadians(0)))),
            new RobotPosition(new Pose2d(new Translation2d(3.21, 4.08), new Rotation2d(Units.degreesToRadians(0)))),
            new RobotPosition(new Pose2d(new Translation2d(3.507, 3.091), new Rotation2d(Units.degreesToRadians(60)))),
            new RobotPosition(new Pose2d(new Translation2d(3.793, 2.955), new Rotation2d(Units.degreesToRadians(60)))),
            new RobotPosition(new Pose2d(new Translation2d(4.791, 2.716), new Rotation2d(Units.degreesToRadians(120)))),
            new RobotPosition(new Pose2d(new Translation2d(5.074, 2.866), new Rotation2d(Units.degreesToRadians(120)))),
            new RobotPosition(new Pose2d(new Translation2d(5.78, 3.65), new Rotation2d(Units.degreesToRadians(180)))),
            new RobotPosition(new Pose2d(new Translation2d(5.78, 3.96), new Rotation2d(Units.degreesToRadians(180)))),
            new RobotPosition(new Pose2d(new Translation2d(5.462, 4.954), new Rotation2d(Units.degreesToRadians(240)))),
            new RobotPosition(new Pose2d(new Translation2d(5.176, 5.099), new Rotation2d(Units.degreesToRadians(240)))),
            new RobotPosition(new Pose2d(new Translation2d(4.166, 5.317), new Rotation2d(Units.degreesToRadians(300)))),
            new RobotPosition(new Pose2d(new Translation2d(3.899, 5.171), new Rotation2d(Units.degreesToRadians(300))))
        };

        for (int level = 0; level < 4; level++) {
            for (int pos = 0; pos < 12; pos++) {
                verticalStates[level][pos] = new RobotState(
                    armAngles[level],
                    elevatorHeights[level],
                    wristAngles[level],
                    (level == 0) ? (RobotPosition) null : positions[pos]
                );
                horizontalStates[level][pos] = new RobotState(
                    armAngles[level+4],
                    elevatorHeights[level+4],
                    wristAngles[level+4],
                    (level == 0) ? (RobotPosition) null : (level == 3) ? positions[pos] : altPositions[pos]
                );
            }
        }
        return new Pair<RobotState[][],RobotState[][]>(verticalStates, horizontalStates);
    }
} 