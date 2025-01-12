package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Arm;

public class Arm_Commands {
    Arm arm;
    public Arm_Commands(Arm arm) {
        this.arm = arm;
    }

    public Command position1() {
        return new InstantCommand(() -> arm.)
    }
}
