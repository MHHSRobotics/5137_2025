package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

/**
 * The `ArmCommands` class provides a set of commands for controlling the arm subsystem.
 * These commands are used to set or adjust the arm's goal position, move to predefined positions,
 * and perform system identification (SysId) routines.
 */
public class ArmCommands{

    private Arm arm; // The arm subsystem that these commands will control.

    /**
     * Constructs an `ArmCommands` object.
     *
     * @param arm The arm subsystem to be controlled by these commands.
     */
    public ArmCommands(Arm arm) {
        this.arm = arm;
    }

    /**
     * Returns a command that sets the arm's goal position to a specified value.
     *
     * @param goal A `DoubleSupplier` that provides the goal position for the arm.
     * @return A command that sets the arm's goal position.
     */
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> arm.setGoal(goal.getAsDouble()), arm);
    }

    /**
     * Returns a command that adjusts the arm's goal position by a specified amount.
     *
     * @param change A `DoubleSupplier` that provides the amount to change the arm's goal position.
     * @return A command that adjusts the arm's goal position.
     */
    public Command changeGoal(DoubleSupplier change) {
        return new FunctionalCommand(
            () -> {},
            () -> arm.setGoal(arm.getGoal() + change.getAsDouble()),
            (Boolean onEnd) -> {},
            () -> {return false;},
            arm);
    }
}