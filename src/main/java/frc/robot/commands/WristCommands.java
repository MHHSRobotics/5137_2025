package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;

/**
 * The `WristCommands` class provides a set of commands for controlling the wrist subsystem.
 * These commands include running system identification routines.
 */
public class WristCommands{
    private Wrist wrist; // The wrist subsystem that these commands will control.

    /**
     * Constructs a new `WristCommands` object.
     *
     * @param wrist The wrist subsystem that this set of commands will control.
     */
    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    /**
     * Creates a command to set the elevator's goal to a specific value provided by a DoubleSupplier.
     *
     * @param goal A DoubleSupplier that provides the target goal position for the elevator.
     * @return A command that sets the elevator's goal when executed.
     */
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> wrist.setGoal(goal.getAsDouble()), wrist);
    }

    /**
     * Creates a command to adjust the elevator's goal by a specified amount.
     *
     * @param change A DoubleSupplier that provides the amount by which to change the current goal.
     * @return A command that adjusts the elevator's goal when executed.
     */
    public Command changeGoal(DoubleSupplier change) {
        return new FunctionalCommand(
            () -> {},
            () -> wrist.setGoal(wrist.getGoal() + change.getAsDouble()),
            (interrupted) -> {},
            () -> {return false;},
            wrist);
    }
}