package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Arm;

/**
 * The `ArmCommands` class provides a set of commands for controlling the arm subsystem.
 * These commands are used to set or adjust the arm's goal position, move to predefined positions,
 * and perform system identification (SysId) routines.
 */
public class ArmCommands implements SysIdCommands{

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

    /**
     * Returns a command that runs a quasi-static system identification routine on the arm.
     *
     * @param dir The direction of the SysId routine (forward or reverse).
     * @return A command that runs the quasi-static SysId routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return arm.getRoutine().quasistatic(dir);
    }

    /**
     * Returns a command that runs a dynamic system identification routine on the arm.
     *
     * @param dir The direction of the SysId routine (forward or reverse).
     * @return A command that runs the dynamic SysId routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return arm.getRoutine().dynamic(dir);
    }
}