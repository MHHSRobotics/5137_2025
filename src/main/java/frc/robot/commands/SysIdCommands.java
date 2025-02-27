package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface SysIdCommands {
    /**
     * Returns a command that runs a quasi-static system identification routine on the wrist.
     *
     * @param dir The direction of the system identification routine (forward or reverse).
     * @return A command that runs the quasi-static system identification routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir);

    /**
     * Returns a command that runs a dynamic system identification routine on the wrist.
     *
     * @param dir The direction of the system identification routine (forward or reverse).
     * @return A command that runs the dynamic system identification routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction dir);
}
