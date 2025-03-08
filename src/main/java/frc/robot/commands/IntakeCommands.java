package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.constants.IntakeConstants;

/**
 * A class that encapsulates all the commands related to the Intake subsystem.
 * This class provides methods to create commands for stopping the intake, running the intake
 * until a switch is triggered, and outtaking (reversing the intake) for a specified duration.
 */
public class IntakeCommands {
    private Intake intake;

    /**
     * Constructor for IntakeCommands.
     *
     * @param intake The Intake subsystem that these commands will operate on.
     */
    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    /**
     * Creates a command that stops the intake.
     *
     * @return A command that, when executed, stops the intake motor.
     */
    public Command stop() {
        return setSpeed(()->0);
    }

    /**
     * Creates a command that stops the intake.
     *
     * @return A command that, when executed, stops the intake motor.
     */
    public Command setSpeed(DoubleSupplier speed) {
        return new InstantCommand(() -> intake.setSpeed(speed.getAsDouble()),intake);
    }


    /**
     * Creates a command that runs the intake at a specified speed until a switch is triggered.
     * The command will continuously run the intake until the switch condition is met.
     *
     * @return A command that runs the intake until the switch is triggered.
     */
    public Command intake() {
        return new SequentialCommandGroup(
            setSpeed(()->IntakeConstants.intakeSpeed), // Start the intake
            new WaitCommand(IntakeConstants.intakeTime),                                                   // Wait for 1 second
            stop()                                                          // Stop the intake
        );
    }

    /**
     * Creates a command that will pulse the intake.
     * The command will pulse the intake until overriden.
     *
     * @return A command that runs the intake until the switch is triggered.
     */
    public Command pulseIntake() {
        return new RepeatCommand(
            new SequentialCommandGroup(
                setSpeed(() -> IntakeConstants.intakeSpeed),
                new WaitCommand(IntakeConstants.pulseTime),
                stop(),
                new WaitCommand(IntakeConstants.pulseWait)
            )                                                          // Stop the intake
        );
    }

    /**
     * Creates a command that outtakes (reverses the intake) for a specified duration.
     * The command will run the intake in reverse for 1 second and then stop it.
     *
     * @return A command that outtakes for 1 second and then stops.
     */
    public Command outtake() {
        return new SequentialCommandGroup(
            setSpeed(()->-IntakeConstants.intakeSpeed), // Start the intake
            new WaitCommand(IntakeConstants.intakeTime),                                                   // Wait for 1 second
            stop()                                                          // Stop the intake
        );
    }
}