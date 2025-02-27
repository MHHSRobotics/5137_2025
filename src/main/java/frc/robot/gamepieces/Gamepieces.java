package frc.robot.gamepieces;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldGeometry;
import frc.robot.constants.GamepieceConstants;

public class Gamepieces extends SubsystemBase{
    private StructArrayPublisher<Pose3d> algaePublisher=NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/sim/algae",Pose3d.struct).publish();
    private StructArrayPublisher<Pose3d> coralPublisher=NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/sim/coral",Pose3d.struct).publish();
    private List<Gamepiece> coral;
    private List<Gamepiece> algae;
    public Gamepieces(){
        // Initialize algae
        algae = new ArrayList<>();
        for (FieldGeometry.FieldPosition pos : FieldGeometry.algaePositions) {
            algae.add(new Gamepiece(GamepieceConstants.algae, pos.bluePos()));
            algae.add(new Gamepiece(GamepieceConstants.algae, pos.redPos()));
        }

        // Initialize source corals
        coral = new ArrayList<>();
        // Add blue alliance corals
        for (FieldGeometry.FieldPosition pos : FieldGeometry.sourceCoralPositions) {
            coral.add(new Gamepiece(GamepieceConstants.coral, pos.bluePos()));
            coral.add(new Gamepiece(GamepieceConstants.coral, pos.redPos()));
        }
    }

    private Gamepiece getClosest(Pose3d pose,Iterable<Gamepiece> others){
        Gamepiece closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Gamepiece g:others) {
            Transform3d transform = g.getLocation().minus(pose);
            double distance = Math.hypot(Math.hypot(transform.getX(), transform.getY()),transform.getZ());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = g;
            }
        }
        return closest;
    }

    public Gamepiece getClosestCoral(Pose3d pose){
        return getClosest(pose,coral);
    }

    public Gamepiece getClosestAlgae(Pose3d pose){
        return getClosest(pose,algae);
    }

    @Override
    public void periodic(){
        Pose3d[] coralPos=new Pose3d[coral.size()];
        for(int i=0;i<coral.size();i++){
            coralPos[i]=coral.get(i).getLocation();
        }
        coralPublisher.set(coralPos);
        Pose3d[] algaePos=new Pose3d[algae.size()];
        for(int i=0;i<algae.size();i++){
            algaePos[i]=algae.get(i).getLocation();
        }
        algaePublisher.set(algaePos);
        for(Gamepiece g:coral){
            g.periodic();
        }
        for(Gamepiece g:algae){
            g.periodic();
        }
    }
}
