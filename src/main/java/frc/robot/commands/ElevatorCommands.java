package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;

/**
 * A class that provides a set of commands for controlling the elevator subsystem.
 * These commands are used to set specific goals for the elevator or dynamically adjust the goal based on input.
 */
public class ElevatorCommands implements SysIdCommands{
    private final Elevator elevator;

    /**
     * Constructs an ElevatorCommands object with the specified elevator subsystem.
     *
     * @param elevator The elevator subsystem to be controlled by these commands.
     */
    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }

    /**
     * Creates a command to set the elevator's goal to a specific value provided by a DoubleSupplier.
     *
     * @param goal A DoubleSupplier that provides the target goal position for the elevator.
     * @return A command that sets the elevator's goal when executed.
     */
    public Command setGoal(DoubleSupplier goal) {
        return new InstantCommand(() -> elevator.setGoal(goal.getAsDouble()), elevator);
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
            () -> elevator.setGoal(elevator.getGoal() + change.getAsDouble()),
            (Boolean onEnd) -> {},
            () -> {return false;},
            elevator);
    }

    /**
     * Creates a command to run a quasistatic system identification routine on the elevator.
     *
     * @param dir The direction of the quasistatic test (forward or reverse).
     * @return A command that runs the quasistatic system identification routine when executed.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return elevator.getRoutine().quasistatic(dir);
    }

    /**
     * Creates a command to run a dynamic system identification routine on the elevator.
     *
     * @param dir The direction of the dynamic test (forward or reverse).
     * @return A command that runs the dynamic system identification routine when executed.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return elevator.getRoutine().dynamic(dir);
    }
}