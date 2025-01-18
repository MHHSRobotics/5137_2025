package frc.robot.elastic;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Reef implements Sendable{
    private boolean[][] coralPlaced;
    public Reef(){
        coralPlaced=new boolean[3][12];
        for (boolean[] bs : coralPlaced) {
            for (int i=0; i<bs.length; i++){
                bs[i]=false;
            }
        }
    }
    public void registerCoralPlaced(int level,int branch){
        coralPlaced[level-2][branch]=true;
    }
    public boolean isCoralPlaced(int level,int branch){
        return coralPlaced[level-2][branch];
    }
    public void setCoralPlaced(int level,int branch,boolean set){
        coralPlaced[level-2][branch]=set;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        for (int i=0; i<coralPlaced.length; i++){
            for (int j=0; j<coralPlaced[i].length; j++){
                int icopy=i;
                int jcopy=j;
                builder.addBooleanProperty("Branch "+j+" L"+(i+2), ()->isCoralPlaced(icopy+2,jcopy), (x)->{setCoralPlaced(icopy+2, jcopy, x);});
            }
        }
    }
    
}
