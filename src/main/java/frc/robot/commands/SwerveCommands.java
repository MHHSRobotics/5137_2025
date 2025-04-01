package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve;

/**
 * This class encapsulates all the commands related to the Swerve subsystem.
 * It provides methods to control the swerve drive, including driving to specific poses,
 * locking the swerve, resetting the gyro, and running system identification routines.
 */
public class SwerveCommands {
    private Swerve swerve;

    // private PIDController xController;
    // private PIDController yController;
    // private PIDController rotController;

    private HolonomicDriveController hdc;

    /**
     * Constructor for SwerveCommands.
     *
     * @param swerve The Swerve subsystem that this class will control.
     */
    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
        var xController = new PIDController(2.0, 0, 0);
        var yController = new PIDController(2.0, 0, 0);
        var rotController = new ProfiledPIDController(2.0, 0, 0, new Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)));
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        rotController.setTolerance(Units.degreesToRadians(1));
        hdc=new HolonomicDriveController(xController, yController, rotController);
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
                swerve.drive(hdc.calculate(current, target, 0, target.getRotation()));
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir){
        return new InstantCommand(()->swerve.sysIdQuasistatic(dir),swerve);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction dir){
        return new InstantCommand(()->swerve.sysIdDynamic(dir),swerve);
    }
}