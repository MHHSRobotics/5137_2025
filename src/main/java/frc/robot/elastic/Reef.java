package frc.robot.elastic;

import org.json.simple.JSONObject;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import frc.robot.constants.FieldGeometry;

public class Reef implements NTSendable{

    // coral placed is L2, L3, L4 then 12 branches
    private boolean[][] coralPlaced;

    // algae placed is AB, CD, ..., KL
    private boolean[] algaePlaced;

    public Reef(){
        coralPlaced=new boolean[3][12];
        algaePlaced=new boolean[6];
        for(int i=0; i<algaePlaced.length; i++){
            algaePlaced[i]=true;
        }
    }

    // levels are 0 = L2, 1 = L3, 2 = L4
    public boolean isCoralPlaced(int level,int branch){
        return coralPlaced[level][branch];
    }

    public void setCoralPlaced(int level,int branch,boolean set){
        coralPlaced[level][branch]=set;
    }

    public boolean isAlgaePlaced(int side){
        return algaePlaced[side];
    }

    public void setAlgaePlaced(int side,boolean set){
        algaePlaced[side]=set;
    }

    public static boolean isAlgaeLow(int side) {
        return side%2 == 0;
    }

    public boolean isCoralBlocked(int level,int branch){
        int side=branch/2;
        if(algaePlaced[side]){
            if(isAlgaeLow(side)){
                return (level==0 || level==1);
            }else{
                return level==1;
            }
        }else{
            return false;
        }
    }

    public boolean isCoralOpen(int level,int branch){
        return !isCoralPlaced(level, branch) && !isCoralBlocked(level, branch);
    }

    public Pair<Integer,Integer> getNearest(int level,int branch){
        double minDist=Double.MAX_VALUE;
        Pair<Integer,Integer> minVal=null;
        for(int l=0;l<=2;l++){
            for(int b=0;b<FieldGeometry.reefSides*2;b++){
                if(isCoralOpen(level, branch)){
                    int modDist=Math.min(Math.floorMod(branch-b,FieldGeometry.reefSides*2),Math.floorMod(b-branch,FieldGeometry.reefSides*2));
                    double dist=Math.abs(l-level)+modDist;
                    if(dist<minDist){
                        minDist=dist;
                        minVal=new Pair<Integer,Integer>(level, branch);
                    }
                }
            }
        }
        return minVal;
    }

    public int getNearestLevel(int level,int branch){
        return getNearest(level, branch).getFirst();
    }

    public int getNearestBranch(int level,int branch){
        return getNearest(level, branch).getSecond();
    }

    @SuppressWarnings("unchecked")
    public String jsonify(){
        JSONObject obj = new JSONObject();
        for (int i=0; i<coralPlaced.length; i++){
            for (int j=0; j<coralPlaced[i].length; j++){
                String propName="B"+j+"L"+(i+2);
                if(isCoralBlocked(i,j)){
                    obj.put(propName, "Blocked");
                }else if(isCoralPlaced(i,j)){
                    obj.put(propName, "Placed");
                }else{
                    obj.put(propName, "Open");
                }
            }
        }
        return obj.toJSONString();
    }
    
    @Override
    public void initSendable(NTSendableBuilder builder) {
        for (int i=0; i<coralPlaced.length; i++){
            for (int j=0; j<coralPlaced[i].length; j++){
                int icopy=i;
                int jcopy=j;
                builder.addBooleanProperty("B"+j+"L"+(i+2), ()->isCoralPlaced(icopy, jcopy), (value)->setCoralPlaced(icopy, jcopy, value));
            }
        }
        for(int i=0; i<algaePlaced.length; i++){
            int icopy=i;
            builder.addBooleanProperty("Side "+i, ()->isAlgaePlaced(icopy), (value)->setAlgaePlaced(icopy, value));
        }
        builder.addStringProperty("ReefJson", ()->jsonify(), null);
        builder.setSmartDashboardType("Reef");
    }
}
