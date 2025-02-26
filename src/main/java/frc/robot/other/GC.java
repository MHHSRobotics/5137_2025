package frc.robot.other;

import edu.wpi.first.wpilibj.Timer;

public class GC extends Thread {
    Timer timer;
    public GC() {
        timer = new Timer();
        timer.start();
    }

    public void run() {
        if (timer.advanceIfElapsed(5)) {
            System.gc();
        }
    }
}
