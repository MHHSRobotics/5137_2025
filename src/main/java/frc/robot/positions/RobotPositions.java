package frc.robot.positions;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

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
    public static final RobotState groundIntake = new RobotState(
        -2.0,
        0.41,
        -1.39,
        (RobotPosition)null  // Robot position determined at runtime
    );   

    public static final RobotState defaultState = new RobotState(
        Units.degreesToRadians(0),
        0.,
        Units.degreesToRadians(90),
        (RobotPosition)null  // Robot position determined at runtime
    );
    
    // Processor state
    public static final RobotState processorState = new RobotState(
        Units.degreesToRadians(-95),
        0.5,
        Units.degreesToRadians(-115),
        new RobotPosition(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(-90))))
    );

    public static final RobotState bargeState = new RobotState(
        Units.degreesToRadians(0),
        1.2,
        Units.degreesToRadians(-45),
        new RobotPosition(new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(180))))
    );

    public static final RobotState groundAlgaeState = new RobotState(
        Units.degreesToRadians(-100),
        0.3,
        Units.degreesToRadians(-115),
        (RobotPosition) null
    );

    public static final RobotState preScoringState = new RobotState(
        Units.degreesToRadians(0),
        null,
        Units.degreesToRadians(0),
        (RobotPosition)null
    );
    
    // Source states
    public static final RobotState[] sourceStates = generateSourceStates();
    
    // Algae states
    public static final RobotState[] algaeStates = generateAlgaeStates();
    
    // Scoring states
    public static final RobotState[][] scoringStates = generateScoringStates();
    
    

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
        try {
            states[0] = new RobotState(
                -0.43,
                0.17,
                Units.degreesToRadians(-113.6),
                PathPlannerPath.fromPathFile("Source Left")
            );
            states[1] = new RobotState(
                -0.43,
                0.17,
                Units.degreesToRadians(-113.6),
                PathPlannerPath.fromPathFile("Source Right")
            );
        } catch (FileVersionException | IOException | ParseException e) {
            throw new RuntimeException(e);
        }
        return states;
    }

    public static final RobotState algaeHighState = new RobotState(
        Units.degreesToRadians(-45),
        0.45,
        Units.degreesToRadians(-90),
        (RobotPosition) null);
    
    public static final RobotState algaeLowState = new RobotState(
        Units.degreesToRadians(-45),
        0.1,
        Units.degreesToRadians(-90),
        (RobotPosition) null);
    
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
    
    private static RobotState[][] generateScoringStates() {
        RobotState[][] states = new RobotState[4][12];
        double[] armAngles = {
            0.66,   // L1
            0.34,   // L2
            0.34,   // L3
            0.225    // L4
        };
        double[] elevatorHeights = {
            0.01,   // L1
            0.1,  // L2
            0.47,   // L3
            1.2     // L4
        };
        double[] wristAngles = {
            0.15,   // L1
            0.18,   // L2
            0.18,   // L3
            0.54    // L4
        };
        PathPlannerPath[] paths = new PathPlannerPath[12];
        for (int i = 0; i < 12; i++) {
            try {
                paths[i] = PathPlannerPath.fromPathFile("Reef " + (char) ('A' + i));
            } catch (FileVersionException | IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }

        for (int level = 0; level < states.length; level++) {
            for (int pos = 0; pos < states[level].length; pos++) {
                states[level][pos] = new RobotState(
                    armAngles[level],
                    elevatorHeights[level],
                    wristAngles[level],
                    paths[pos]
                );
            }
        }
        return states;
    }
} 