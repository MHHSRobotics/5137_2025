package frc.robot.motorSystem;

// Interface to abstract different simulation types
public interface MechanismSim {
    void setInputVoltage(double voltage);
    void update(double period);
    double getPosition();
    double getVelocity();
    double getCurrentDrawAmps();
}
