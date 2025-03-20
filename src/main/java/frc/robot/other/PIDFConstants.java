package frc.robot.other;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class PIDFConstants {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kG;
    public double kV;
    public double kA;
    public boolean isArm;

    public PIDFConstants() {}

    public PIDFConstants withKP(double kP) {
        this.kP = kP;
        return this;
    }

    public PIDFConstants withKI(double kI) {
        this.kI = kI;
        return this;
    }

    public PIDFConstants withKD(double kD) {
        this.kD = kD;
        return this;
    }

    public PIDFConstants withKS(double kS) {
        this.kS = kS;
        return this;
    }

    public PIDFConstants withKG(double kG) {
        this.kG = kG;
        return this;
    }

    public PIDFConstants withKV(double kV) {
        this.kV = kV;
        return this;
    }

    public PIDFConstants withKA(double kA) {
        this.kA = kA;
        return this;
    }

    public PIDFConstants withIsArm(boolean isArm) {
        this.isArm=isArm;
        return this;
    }

    public SlotConfigs slotConfigs(){
        return new SlotConfigs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKG(kG)
            .withKV(kV)
            .withKA(kA)
            .withGravityType(isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static);
    }
}
