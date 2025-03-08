package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.visualization.RobotPublisher;

/**
 * Commands for controlling the RobotPublisher.
 * These commands handle gamepiece simulation for visualization.
 */
public class RobotPublisherCommands {
    private final RobotPublisher robotPublisher;
    
    /**
     * Creates a new RobotPublisherCommands.
     * 
     * @param robotPublisher The RobotPublisher to control
     */
    public RobotPublisherCommands(RobotPublisher robotPublisher) {
        this.robotPublisher = robotPublisher;
    }
    
    /**
     * Simulates picking up a coral gamepiece.
     * 
     * @return A command that simulates picking up a coral gamepiece
     */
    public Command simCoralIntake() {
        return new InstantCommand(() -> robotPublisher.simCoralIntake());
    }
    
    /**
     * Simulates picking up an algae gamepiece.
     * 
     * @return A command that simulates picking up an algae gamepiece
     */
    public Command simAlgaeIntake() {
        return new InstantCommand(() -> robotPublisher.simAlgaeIntake());
    }
    
    /**
     * Simulates releasing a coral gamepiece.
     * 
     * @return A command that simulates releasing a coral gamepiece
     */
    public Command simCoralOuttake() {
        return new InstantCommand(() -> robotPublisher.simCoralOuttake());
    }
    
    /**
     * Simulates releasing an algae gamepiece.
     * 
     * @return A command that simulates releasing an algae gamepiece
     */
    public Command simAlgaeOuttake() {
        return new InstantCommand(() -> robotPublisher.simAlgaeOuttake());
    }
} 