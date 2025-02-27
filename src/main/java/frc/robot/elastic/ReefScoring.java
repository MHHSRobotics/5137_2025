package frc.robot.elastic;

import org.json.simple.JSONObject;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.constants.FieldGeometry;

public class ReefScoring implements Sendable{
    private Reef reef;
    public ReefScoring(Reef reef){
        this.reef=reef;
    }
    
    public int getCount(int level){
        int count=0;
        for(int j=0; j<FieldGeometry.reefSides*2; j++){
            if(reef.isCoralPlaced(level,j) && !reef.isCoralBlocked(level,j)){
                count++;
            }
        }
        return count;
    }

    @SuppressWarnings("unchecked")
    public String jsonify(){
        JSONObject obj = new JSONObject();
        for (int i=0; i<=2; i++){
            obj.put("L"+(i+2), getCount(i));
        }
        return obj.toJSONString();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        for(int i=0;i<=2;i++){
            int icopy=i;
            builder.addIntegerProperty("L"+(i+2), ()->getCount(icopy), null);
        }
        builder.addStringProperty("ScoringJson", ()->jsonify(), null);
        builder.setSmartDashboardType("ReefScoring");
    }
}
