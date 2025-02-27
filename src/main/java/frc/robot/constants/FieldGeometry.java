package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.other.RobotUtils;

public final class FieldGeometry {
    public static final double fieldLength = 17.55;
    public static final double fieldWidth = 8.05;
    public static final int reefSides = 6;

    // Starting coral geometry
    private static final double startingCoralInset = 0.65;
    private static final double startingCoralHeight = 1.12;
    private static final double startingCoralPitch = Math.PI/4;
    private static final double startingCoralYaw = 0.94;
    private static final int startingCoralsPerLine = 20; // -10 to 10
    private static final double startingCoralSpacing = 0.1; // 1/10th meter spacing

    public static class FieldPosition {
        private final Pose3d bluePosition;
        
        public FieldPosition(Pose3d bluePosition) {
            this.bluePosition = bluePosition;
        }
        
        public Pose3d bluePos() { return bluePosition; }
        public Pose3d redPos() { return RobotUtils.invertPose(bluePosition); }
        public Pose3d alliancePos() { 
            return RobotUtils.invertToAlliance(bluePosition);
        }
    }

    public static final Translation2d reefCenter = new Translation2d(4.4958, 4.0259);
    
    // Coral geometry
    private static final double branchd1 = 0.75;
    private static final double branchd2 = 0.16;
    private static final double l2d1 = 0.1;
    private static final double l2height = 0.740;
    private static final double l3d1 = 0.1;
    private static final double l3height = 1.143;
    private static final double l4d1 = 0.2;
    private static final double l4height = 1.6;

    // Algae geometry
    private static final double algaed1 = 0.66;
    private static final double algaeLowHeight = 0.88;
    private static final double algaeHighHeight = 1.28;

    // Static field elements
    public static final FieldPosition[][] coralPositions = generateCoralPositions();
    public static final FieldPosition[] algaePositions = generateAlgaePositions();
    public static final FieldPosition[] sourceCoralPositions = generateSourceCoralPositions();

    private static FieldPosition[] generateSourceCoralPositions() {
        FieldPosition[] positions = new FieldPosition[startingCoralsPerLine * 2]; // 2 lines of corals (blue alliance)
        
        // Define the two blue alliance corner starting positions
        Pose3d[] blueCornerStarts = {
            new Pose3d(startingCoralInset, startingCoralInset, startingCoralHeight, 
                      new Rotation3d(0, startingCoralPitch, startingCoralYaw)),
            new Pose3d(startingCoralInset, fieldWidth - startingCoralInset, startingCoralHeight, 
                      new Rotation3d(0, startingCoralPitch, -startingCoralYaw))
        };
        
        // Generate coral positions for each blue alliance corner
        for (int corner = 0; corner < 2; corner++) {
            for (int i = 0; i < startingCoralsPerLine; i++) {
                double offset = (i - startingCoralsPerLine/2) * startingCoralSpacing;
                positions[corner * startingCoralsPerLine + i] = new FieldPosition(
                    blueCornerStarts[corner].transformBy(new Transform3d(0, offset, 0, new Rotation3d()))
                );
            }
        }
        
        return positions;
    }

    private static FieldPosition[][] generateCoralPositions() {
        FieldPosition[][] positions = new FieldPosition[12][3]; // [branch][level]
        
        for (int branch = 0; branch < 12; branch++) {
            int side = branch / 2;
            boolean isSecondBranch = (branch % 2) == 1;
            
            Rotation2d angle = new Rotation2d(2 * side * Math.PI / reefSides);
            Translation2d center = reefCenter.plus(new Translation2d(branchd1, angle));
            
            Rotation2d branchOffset = angle.rotateBy(isSecondBranch ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg);
            Translation2d branchPos = center.plus(new Translation2d(branchd2, branchOffset));
            
            double[] heights = {l2height, l3height, l4height};
            double[] offsets = {l2d1, l3d1, l4d1};
            
            for (int level = 0; level < 3; level++) {
                Translation2d levelPos = branchPos.plus(new Translation2d(offsets[level], angle));
                positions[branch][level] = new FieldPosition(
                    new Pose3d(levelPos.getX(), levelPos.getY(), heights[level], new Rotation3d())
                );
            }
        }
        return positions;
    }

    private static FieldPosition[] generateAlgaePositions() {
        FieldPosition[] positions = new FieldPosition[reefSides];
        
        for (int i = 0; i < reefSides; i++) {
            Rotation2d angle = new Rotation2d(2 * i * Math.PI / reefSides);
            Translation2d pos = reefCenter.plus(new Translation2d(algaed1, angle));
            double height = isAlgaeLow(i) ? algaeLowHeight : algaeHighHeight;
            
            positions[i] = new FieldPosition(
                new Pose3d(pos.getX(), pos.getY(), height, new Rotation3d())
            );
        }
        return positions;
    }

    private static boolean isAlgaeLow(int index) {
        return index % 2 == 0;
    }
}
