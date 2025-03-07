package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.LEDConstants;
import frc.robot.other.ImprovedNoise;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LED extends SubsystemBase{
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDSim ledSim;
    private int iters = 0;
    private int inc = 0;
    private int timing = 0;

    public LED(){
        led = new AddressableLED(LEDConstants.ledPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.ledStripLength);
        ledSim = new AddressableLEDSim(led);
        ledSim.setRunning(true);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    private double getNoise(double x){
        return (Math.sin(1.5*x)+Math.sin(0.6*x)+Math.sin(2.3*x)+Math.sin(1.8*x)+Math.sin(0.4*x))/5;
    }

    public void redYellow(){
        if (timing % LEDConstants.redYellowSpeed == 0){
            inc++;
            for (int i=0;i<6;i++){
                for (int j=0;j<10;j++){
                    ledBuffer.setRGB((j+inc)%10 + (i*10), 200, 200 + (j*15), 0);
                }
            timing = 0;
            }
        }
        timing ++;
    }

    public void blue(){
        inc++;
        for (int i=0;i<6;i++){
            for (int j=0;j<10;j++){
                ledBuffer.setRGB((j+inc)%10 + (i*10),0,0,200-(j+15));
            }
        }
    }

    public void red(){
        inc++;
        for (int i=0;i<6;i++){
            for (int j=0;j<10;j++){
                ledBuffer.setRGB((j+inc)%10 + (i*10),200-(j+15),0,0);
            }
        }
    }
    public void noiseFire(){
        double noise=ImprovedNoise.noise(iters*LEDConstants.ledSpeed,1.4,8.6);
        double ledsOn=LEDConstants.ledStripLength/4.*(1+noise);
        for(int i=0;i<LEDConstants.ledStripLength/2;i++){
            double ratio=i/ledsOn;
            if(ratio<1){
                ledBuffer.setRGB(i,255,(int)(200*ratio),0);
                ledBuffer.setRGB(LEDConstants.ledStripLength-i-1,255,(int)(200*ratio),0);
            }else{
                ledBuffer.setRGB(i, 0, 0, 0);
                ledBuffer.setRGB(LEDConstants.ledStripLength-i-1, 0, 0, 0);
            }
        }
    }

    public void worseFire(){
        double ledsOn=LEDConstants.ledStripLength/4.*(1+getNoise(iters*LEDConstants.ledSpeed));
        for(int i=0;i<LEDConstants.ledStripLength/2;i++){
            double ratio=i/ledsOn;
            if(ratio<1){
                ledBuffer.setRGB(i,255,(int)(200*ratio),0);
                ledBuffer.setRGB(LEDConstants.ledStripLength-i-1,255,(int)(200*ratio),0);
            }else{
                ledBuffer.setRGB(i, 0, 0, 0);
                ledBuffer.setRGB(LEDConstants.ledStripLength-i-1, 0, 0, 0);
            }
        }
    }
    

    @Override
    public void periodic(){
        iters += 1;
        redYellow();
        byte[] dataConvert = new byte[LEDConstants.ledStripLength*4];
        for(int i = 0; i<LEDConstants.ledStripLength; i++){
            dataConvert[i*4] = (byte)(ledBuffer.getBlue(i));
            dataConvert[i*4+1] = (byte)(ledBuffer.getGreen(i));
            dataConvert[i*4+2] = (byte)(ledBuffer.getRed(i));
        }
        led.setData(ledBuffer);
        if(Robot.isSimulation()){
            ledSim.setData(dataConvert);
        }
    }
}

