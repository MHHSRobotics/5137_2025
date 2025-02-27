package frc.robot.motorSystem;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MotorSystem {
    private List<EnhancedTalonFX> motors;
    private EnhancedEncoder encoder;
    private double appliedVoltage;

    public MotorSystem(List<EnhancedTalonFX> motors, EnhancedEncoder encoder){
        this.motors=motors;
        this.encoder=encoder;
    }

    public List<EnhancedTalonFX> getMotors() {
        return motors;
    }

    public void set(double vel){
        motors.forEach(m->m.set(vel));
    }

    public void setVoltage(Voltage v){
        appliedVoltage=v.magnitude();
        motors.forEach(m->m.setVoltage(v.magnitude()));
    }

    public Voltage getVolts() {
        return motors.stream()
            .map(EnhancedTalonFX::getMotorVoltage)
            .map(StatusSignal::getValue)
            .reduce(Voltage::plus)
            .map(v -> v.div(motors.size()))
            .orElse(Volts.of(0));
    }

    public double getVelocity() {
        return motors.stream()
            .mapToDouble(EnhancedTalonFX::getVel)
            .average()
            .orElse(0.0);
    }

    public double getAcceleration() {
        return motors.stream()
            .mapToDouble(EnhancedTalonFX::getAcc)
            .average()
            .orElse(0.0);
    }

    public double getMeasurement() {
        return encoder.get();
    }

    public void periodic() {
        encoder.periodic();
    }

    public void log(String prefix) {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).log(prefix + "/motor" + (motors.size() > 1 ? (i + 1) : ""));
        }
        encoder.log(prefix + "/encoder");
        SmartDashboard.putNumber(prefix+"/appliedVoltage", appliedVoltage);
    }

    public void simulationPeriodic(MechanismSim sim, double period) {
        // Update motor supply voltages
        motors.forEach(EnhancedTalonFX::refreshSupplyVoltage);

        // Get average input voltage
        double input = motors.stream()
            .mapToDouble(EnhancedTalonFX::getSimVoltage)
            .average()
            .orElse(0.0);

        // Update simulation
        sim.setInputVoltage(input);
        sim.update(period);

        // Update motor and encoder states
        double position = sim.getPosition();
        double velocity = sim.getVelocity();
        motors.forEach(motor -> {
            motor.setSimVel(velocity);
            motor.setSimAcc(0.0);  // Reset acceleration since sim doesn't provide it
        });
        encoder.set(position);

        // Update RoboRIO simulation
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps())
        );
    }
}
