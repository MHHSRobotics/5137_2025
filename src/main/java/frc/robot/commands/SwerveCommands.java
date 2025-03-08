package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

/**
 * This class encapsulates all the commands related to the Swerve subsystem.
 * It provides methods to control the swerve drive, including driving to specific poses,
 * locking the swerve, resetting the gyro, and running system identification routines.
 */
public class SwerveCommands {
    private Swerve swerve;

    /**
     * Constructor for SwerveCommands.
     *
     * @param swerve The Swerve subsystem that this class will control.
     */
    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
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
    public Command drive(DoubleSupplier dx, DoubleSupplier dy, DoubleSupplier dtheta, BooleanSupplier fieldOriented) {
        return new InstantCommand(() -> swerve.setPercentDrive(dx.getAsDouble(), dy.getAsDouble(), dtheta.getAsDouble(), fieldOriented.getAsBoolean()),swerve);
    }

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

    /**
     * Creates a command to drive to a target pose using PID control.
     * 
     * @param targetPoseSupplier A supplier for the target pose
     * @return A command that drives to the target pose using PID control
     */
    public Command driveToPoseWithPID(Supplier<Pose2d> targetPoseSupplier) {
        // Create PID controllers outside the command so they can be accessed by all lambdas
        PIDController xController = new PIDController(
            SwerveConstants.translationKP,
            SwerveConstants.translationKI,
            SwerveConstants.translationKD
        );
        PIDController yController = new PIDController(
            SwerveConstants.translationKP,
            SwerveConstants.translationKI,
            SwerveConstants.translationKD
        );
        PIDController rotController = new PIDController(
            SwerveConstants.rotationKP,
            SwerveConstants.rotationKI,
            SwerveConstants.rotationKD
        );
        
        // Configure rotation controller for continuous input
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set tolerances
        xController.setTolerance(SwerveConstants.transTol);
        yController.setTolerance(SwerveConstants.transTol);
        rotController.setTolerance(SwerveConstants.rotTol);
        
        return new FunctionalCommand(
            // Initialize
            () -> {
                // Reset controllers when starting
                xController.reset();
                yController.reset();
                rotController.reset();
                
                // Log that we're starting PID control
                SmartDashboard.putBoolean("PID Control Active", true);
            },
            // Execute - calculate and apply PID outputs
            () -> {
                Pose2d targetPose = targetPoseSupplier.get();
                Pose2d currentPose = swerve.getPose();
                
                // Calculate errors in field-relative coordinates
                double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
                double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
                double rotSpeed = rotController.calculate(
                    currentPose.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );
                
                // Limit speeds to maximum values
                double maxSpeed = swerve.getMaxSpeed();
                double maxAngularSpeed = swerve.getMaxAngularSpeed();
                
                xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
                ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);
                rotSpeed = MathUtil.clamp(rotSpeed, -maxAngularSpeed, maxAngularSpeed);
                
                // Apply speeds using field-relative control
                swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, currentPose.getRotation()
                ));
                
                // Log PID data to SmartDashboard
                SmartDashboard.putNumber("PID/X Error", currentPose.getX() - targetPose.getX());
                SmartDashboard.putNumber("PID/Y Error", currentPose.getY() - targetPose.getY());
                SmartDashboard.putNumber("PID/Rot Error", 
                    currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
                
                // Check if we're at the target
                boolean atXTarget = xController.atSetpoint();
                boolean atYTarget = yController.atSetpoint();
                boolean atRotTarget = rotController.atSetpoint();
                
                SmartDashboard.putBoolean("PID/At X Target", atXTarget);
                SmartDashboard.putBoolean("PID/At Y Target", atYTarget);
                SmartDashboard.putBoolean("PID/At Rot Target", atRotTarget);
                SmartDashboard.putBoolean("PID/At Target", atXTarget && atYTarget && atRotTarget);
            },
            // End - clean up when the command ends
            (interrupted) -> {
                // Stop the robot
                swerve.drive(new ChassisSpeeds());
                // Log that PID control is no longer active
                SmartDashboard.putBoolean("PID Control Active", false);
            },
            // isFinished - determine when the command should end
            () -> {
                // Use the PID controllers' atSetpoint methods to determine if we're at the target
                return xController.atSetpoint() && 
                       yController.atSetpoint() && 
                       rotController.atSetpoint();
            },
            // Requirements - this command requires the swerve subsystem
            swerve
        );
    }
    
    /**
     * Creates a command to drive to a specific pose using PID control.
     * 
     * @param targetPose The target pose to drive to
     * @return A command that drives to the target pose using PID control
     */
    public Command driveToPoseWithPID(Pose2d targetPose) {
        return driveToPoseWithPID(() -> targetPose);
    }
}