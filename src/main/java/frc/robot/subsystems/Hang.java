package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HangConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class Hang extends SubsystemBase {
    
    private final Solenoid clampSolenoid;
    private final Solenoid climbSolenoid;
    private final Compressor compressor;

    public Hang(){
        clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HangConstants.clampSolenoid);
        climbSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HangConstants.climbSolenoid);
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(HangConstants.minPressure, HangConstants.maxPressure);
    }
    
    public void clampActivate(){
        clampSolenoid.set(true);
    }

    public void clampDeactivate(){
        clampSolenoid.set(false);
    }

    public boolean isClampActivated(){
        return clampSolenoid.get();
    }

    public void climbExtend(){
        climbSolenoid.set(true);
    }

    public void climbRetract(){
        climbSolenoid.set(false);
    }

    public void isclimbExtended(){
        climbSolenoid.get();
    } 
}