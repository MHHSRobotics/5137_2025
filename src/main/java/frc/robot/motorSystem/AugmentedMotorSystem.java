package frc.robot.motorSystem;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AugmentedMotorSystem {
    private List<BetterTalonFX> motors;
    private SyncedEncoder encoder;
    private double appliedVoltage;

    public AugmentedMotorSystem(List<BetterTalonFX> motors,SyncedEncoder encoder){
        this.motors=motors;
        this.encoder=encoder;
        if(encoder!=null){
            setMotorPos(encoder.get());
        }
    }

    public List<BetterTalonFX> getMotors() {
        return motors;
    }

    public SyncedEncoder getEncoder() {
        return encoder;
    }

    public Voltage getVolts() {
        return motors.stream()
            .map(BetterTalonFX::getMotorVoltage)
            .map(StatusSignal::getValue)
            .reduce(Voltage::plus)
            .map(v -> v.div(motors.size()))
            .orElse(Volts.of(0));
    }

    public double getVelocity() {
        return motors.stream()
            .mapToDouble(BetterTalonFX::getVel)
            .average()
            .orElse(0);
    }

    public double getAcceleration() {
        return motors.stream()
            .mapToDouble(BetterTalonFX::getAcc)
            .average()
            .orElse(0);
    }

    public void setTargetPosition(double pos) {
        motors.forEach((m)->m.setTargetPosition(pos));
    }

    public double getTargetPosition() {
        return motors.stream()
            .mapToDouble(BetterTalonFX::getTargetPosition)
            .average()
            .orElse(0);
    }

    public double getError(){
        return motors.stream()
            .mapToDouble(BetterTalonFX::getError)
            .average()
            .orElse(0);
    }

    private void setMotorPos(double pos){
        motors.forEach((m)->m.setPosition(pos));
    }

    private double getMotorPos() {
        return motors.stream()
            .mapToDouble(BetterTalonFX::getPos)
            .average()
            .orElse(0);
    }

    public double getMeasurement(){
        return getMotorPos();
    }

    public void log(String prefix) {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).log(prefix + "/motor" + (motors.size() > 1 ? (i + 1) : ""));
        }
        if(encoder!=null){
            encoder.log(prefix + "/encoder");
        }
        SmartDashboard.putNumber(prefix+"/appliedVoltage", appliedVoltage);
        SmartDashboard.putNumber(prefix+"/velocity", getVelocity());
        SmartDashboard.putNumber(prefix+"/position", getMeasurement());
        SmartDashboard.putNumber(prefix+"/goal", getTargetPosition());
        SmartDashboard.putNumber(prefix+"/acceleration", getAcceleration());
        SmartDashboard.putNumber(prefix+"/error", getError());
    }

    public void periodic(){
        if(encoder!=null){
            double motorPos=getMotorPos();
            double syncedPos=encoder.getSyncedPosition(motorPos);
            setMotorPos(syncedPos);
        }
    }

    public void simulationPeriodic(MechanismSim sim, double period) {
        // Update motor supply voltages
        motors.forEach(BetterTalonFX::refreshSimVoltage);

        // Get average input voltage
        double input = motors.stream()
            .mapToDouble(BetterTalonFX::getSimVoltage)
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
        setMotorPos(position);
        encoder.set(position);

        // Update RoboRIO simulation
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps())
        );
    }
}
