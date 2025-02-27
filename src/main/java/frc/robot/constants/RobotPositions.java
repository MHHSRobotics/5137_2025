package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.other.RobotUtils;

public final class RobotPositions {
    // Reef geometry for robot positions
    private static final double d1 = 1.3;
    private static final double d2 = 0.165;
    private static final double dShift = -0.21;

    // Static robot positions
    public static final RobotPosition processor = new RobotPosition(
        new Pose2d(new Translation2d(6.25, 1), new Rotation2d(3*Math.PI/2))
    );

    public static final RobotPosition[] cages = generateCages();
    public static final RobotPosition[] stations = generateStations();
    public static final RobotPosition[] pickups = generatePickups();
    public static final RobotPosition[] centerReef = generateCenterReef();
    public static final RobotPosition[] branchReef = generateBranchReef();

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

    private static RobotPosition[] generateCages() {
        return new RobotPosition[] {
            new RobotPosition(new Pose2d(new Translation2d(8.5, 7.25), new Rotation2d(3*Math.PI/2))), // left
            new RobotPosition(new Pose2d(new Translation2d(8.5, 6.15), new Rotation2d(3*Math.PI/2))), // center
            new RobotPosition(new Pose2d(new Translation2d(8.5, 5.05), new Rotation2d(3*Math.PI/2)))  // right
        };
    }

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

    private static RobotPosition[] generateCenterReef() {
        RobotPosition[] positions = new RobotPosition[FieldGeometry.reefSides];
        
        for(int i = 0; i < FieldGeometry.reefSides; i++) {
            Rotation2d forward = new Rotation2d(2*i*Math.PI/FieldGeometry.reefSides).rotateBy(Rotation2d.k180deg);
            Translation2d outer = FieldGeometry.reefCenter.plus(new Translation2d(d1, forward));
            Rotation2d sidewaysAngle = forward.rotateBy(Rotation2d.kCW_90deg);
            Translation2d algae = outer.plus(new Translation2d(dShift, sidewaysAngle));
            positions[i] = new RobotPosition(new Pose2d(algae, forward));
        }
        
        return positions;
    }

    private static RobotPosition[] generateBranchReef() {
        RobotPosition[] positions = new RobotPosition[FieldGeometry.reefSides * 2];
        
        for(int i = 0; i < FieldGeometry.reefSides; i++) {
            Rotation2d forward = new Rotation2d(2*i*Math.PI/FieldGeometry.reefSides).rotateBy(Rotation2d.k180deg);
            Translation2d outer = FieldGeometry.reefCenter.plus(new Translation2d(d1, forward));
            Rotation2d sidewaysAngle = forward.rotateBy(Rotation2d.kCW_90deg);
            Translation2d algae = outer.plus(new Translation2d(dShift, sidewaysAngle));
            
            Translation2d left = algae.plus(new Translation2d(d2, sidewaysAngle));
            positions[2*i] = new RobotPosition(new Pose2d(left, forward));
            
            Translation2d right = algae.plus(new Translation2d(-d2, sidewaysAngle));
            positions[2*i+1] = new RobotPosition(new Pose2d(right, forward));
        }
        
        return positions;
    }
} 