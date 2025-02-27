package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hang;

/**
 * The `HangCommand` class is a `SequentialCommandGroup` that sequences a series of commands
 * to control the hanging mechanism of the robot. This command is used to perform a sequence
 * of actions: deactivating the clamp, extending the climb mechanism, activating the clamp,
 * and then retracting the climb mechanism.
 */
public class HangCommands {

    private Hang hang;
    /**
     * Constructs a new `HangCommand` that sequences the necessary commands to operate the hang mechanism.
     *
     * @param hangSubsystem The `Hang` subsystem that this command will operate on.
     */
    public HangCommands(Hang hang) {
        this.hang=hang;
    }

    public Command setSpeed(DoubleSupplier speed){
        return new FunctionalCommand(
            () -> {},
            () -> hang.setSpeed(speed.getAsDouble()),
            (Boolean onEnd) -> hang.stop(),
            () -> false,
            hang);
    }

    public Command stop(){
        return new InstantCommand(()->hang.stop(),hang);
    }
}