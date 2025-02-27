package frc.robot.motorSystem;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorMechanismSim implements MechanismSim {
    private final ElevatorSim sim;

    public ElevatorMechanismSim(ElevatorSim sim) {
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
        return sim.getPositionMeters();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public double getCurrentDrawAmps() {
        return sim.getCurrentDrawAmps();
    }
}
