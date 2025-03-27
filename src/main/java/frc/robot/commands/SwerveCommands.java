package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.other.RobotUtils;
import frc.robot.subsystems.Swerve;

/**
 * This class encapsulates all the commands related to the Swerve subsystem.
 * It provides methods to control the swerve drive, including driving to specific poses,
 * locking the swerve, resetting the gyro, and running system identification routines.
 */
public class SwerveCommands {
    private Swerve swerve;

    private PIDController xController;
    private PIDController yController;
    private PIDController rotController;

    /**
     * Constructor for SwerveCommands.
     *
     * @param swerve The Swerve subsystem that this class will control.
     */
    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
        this.xController = new PIDController(0.8, 0, 0);
        this.yController = new PIDController(0.8, 0, 0);
        this.rotController = new PIDController(1.0, 0, 0);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        this.xController.setTolerance(0.01);
        this.yController.setTolerance(0.01);
        this.rotController.setTolerance(Units.degreesToRadians(1));
    }

    /**
     * Creates a command to drive the swerve subsystem based on the provided inputs.
     *
     * @param dx The supplier for the x-axis (forward/backward) movement.
     * @param dy The supplier for the y-axis (left/right) movement.
     * @param dtheta The supplier for the rotational movement.
     * @param fieldOriented The supplier for whether the drive should be field-oriented.
     * @return A command that drives the swerve subsystem.
     */
    public Command drive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier dtheta, BooleanSupplier autoRotate) {
        return new FunctionalCommand(
            () -> {},
            () -> {
                if(dx!=null||dy!=null||dtheta!=null){
                    swerve.setPercentDrive(dx.getAsDouble(), dy.getAsDouble(), dtheta.getAsDouble(), autoRotate.getAsBoolean());
                }
            },
            (interrupted) -> {},
            () -> false,
            swerve
        ).withName("SwerveDefault");
    }

    public Command driveToPose(Pose2d target) {
        return new FunctionalCommand(
            () -> {
                swerve.setTargetPose(target);
            },
            () -> {
                Pose2d current = swerve.getPose();
                swerve.autoDrive(
                    xController.calculate(current.getX(), target.getX()),
                    yController.calculate(current.getY(), target.getY()),
                    rotController.calculate(current.getRotation().getRadians(), target.getRotation().getRadians()));
            },
            (interrupted) -> {},
            () -> swerve.atTarget(),
            swerve
        ).withName("AutoAlign");
    }

    public Command autoDriveWithY(Pose2d target, DoubleSupplier dy) {
        return new FunctionalCommand(
            () -> {
                swerve.setTargetPose(target);
            },
            () -> {
                Pose2d current = swerve.getPose();
                swerve.autoDrive(
                    xController.calculate(current.getX(), target.getX()),
                    RobotUtils.onRedAlliance() ? -dy.getAsDouble() : dy.getAsDouble(),
                    rotController.calculate(current.getRotation().getRadians(), target.getRotation().getRadians()));
            },
            (interrupted) -> {},
            () -> swerve.atTarget(),
            swerve
        ).withName("AutoAlign");
    }


    public Command overrideDrive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier dtheta, BooleanSupplier autoRotate, double time) {        
        return new ParallelRaceGroup(
            new RepeatCommand(drive(dx,dy,dtheta,autoRotate)),
            new WaitCommand(time)
        );
    }

    /*
    public Command driveBack(){
        return overrideDrive(()->-SwerveConstants.driveBackPower, ()->0, ()->0, ()->false, SwerveConstants.driveBackTime);
    }*/

    /**
     * Creates a command to lock the swerve subsystem in place.
     *
     * @return A command that locks the swerve subsystem.
     */
    public Command lock() {
        return new InstantCommand(() -> swerve.lock(), swerve);
    }

    /**
     * Creates a command to reset the gyro of the swerve subsystem.
     *
     * @return A command that resets the gyro.
     */
    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetGyro(), swerve);
    }
}