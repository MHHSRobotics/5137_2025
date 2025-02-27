package frc.robot.motorSystem;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// Implementation for SingleJointedArmSim
public class ArmMechanismSim implements MechanismSim {
    private final SingleJointedArmSim sim;

    public ArmMechanismSim(SingleJointedArmSim sim) {
        this.sim = sim;
    }

    @Override
    public void setInputVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    public void update(double period) {
        sim.update(period);
    }

    @Override
    public double getPosition() {
        return sim.getAngleRads();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public double getCurrentDrawAmps() {
        return sim.getCurrentDrawAmps();
    }
}
