package frc.robot.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Gamepiece {
    private Pose3d goal;
    private Pose3d location;
    private Translation3d transform;

    public Gamepiece(Translation3d transform,Pose3d start){
        this.transform=transform;
        this.location=start;
        this.goal=start;
    }

    public void setGoal(Pose3d location){
        this.goal=location;
    }

    public void setGoalOnBot(Pose3d location){
        this.goal=location.transformBy(new Transform3d(transform,new Rotation3d()));
    }

    public Pose3d getLocation(){
        return location;
    }

    public Pose3d getGoal(){
        return goal;
    }

    public void periodic(){
        location=location.transformBy(goal.minus(location).div(5));
    }
}
