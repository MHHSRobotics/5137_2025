package frc.robot.other;

import frc.robot.constants.SwerveConstants;
import frc.robot.motorSystem.EnhancedTalonFX;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.io.FileReader;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

/**
 * The SwerveFactory class is responsible for creating and configuring a swerve drivetrain
 * using parameters defined in a JSON configuration file. It initializes the swerve modules,
 * motors, encoders, and other necessary components based on the provided configuration.
 */
public class SwerveFactory {

    private JSONParser parser = new JSONParser(); // JSON parser to read the configuration file

    private double maxSpeed; // Maximum speed of the robot in meters per second
    private double maxAngularSpeed; // Maximum angular speed of the robot in radians per second
    private SwerveDrivetrain<EnhancedTalonFX, EnhancedTalonFX, CANcoder> drivetrain; // The swerve drivetrain object

    /**
     * Constructor for SwerveFactory. Initializes the swerve drivetrain using the provided JSON configuration file.
     *
     * @param file The JSON file containing the configuration parameters for the swerve drivetrain.
     * @throws RuntimeException If there is an error parsing the JSON file or initializing the drivetrain.
     */
    public SwerveFactory(File file) {
        try {
            // Parse the JSON configuration file
            JSONObject constants = (JSONObject) parser.parse(new FileReader(file));

            // Extract drive and steer motor gains from the JSON configuration
            JSONObject drive_gains = (JSONObject) constants.get("drive_gains");
            JSONObject steer_gains = (JSONObject) constants.get("steer_gains");

            // Set the maximum speed and angular speed from the configuration
            maxSpeed = (double) constants.get("max_speed");
            maxAngularSpeed = (double) constants.get("max_angular_speed");

            // Create a SwerveModuleConstantsFactory to configure the swerve modules
            SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantCreator =
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio((double) constants.get("drive_ratio")) // Set drive motor gear ratio
                    .withSteerMotorGearRatio((double) constants.get("steer_ratio")) // Set steer motor gear ratio
                    .withCouplingGearRatio((double) constants.get("couple_ratio")) // Set coupling gear ratio
                    .withWheelRadius(Inches.of((double) constants.get("wheel_radius"))) // Set wheel radius
                    .withDriveMotorGains(new Slot0Configs() // Configure drive motor PID gains
                        .withKP((double) drive_gains.get("P"))
                        .withKI((double) drive_gains.get("I"))
                        .withKD((double) drive_gains.get("D"))
                        .withKS((double) drive_gains.get("kS"))
                        .withKV((double) drive_gains.get("kV")))
                    .withSteerMotorGains(new Slot0Configs() // Configure steer motor PID gains
                        .withKP((double) steer_gains.get("P"))
                        .withKI((double) steer_gains.get("I"))
                        .withKD((double) steer_gains.get("D"))
                        .withKS((double) steer_gains.get("kS"))
                        .withKV((double) steer_gains.get("kV"))
                        .withKA((double) steer_gains.get("kA"))
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage) // Set steer motor closed-loop output type
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage) // Set drive motor closed-loop output type
                    .withSlipCurrent(Amps.of((double) constants.get("slip_current"))) // Set slip current limit
                    .withSpeedAt12Volts(MetersPerSecond.of((double) constants.get("max_speed"))) // Set speed at 12 volts
                    .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated) // Set drive motor type
                    .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated) // Set steer motor type
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder) // Set feedback source for steering
                    .withDriveMotorInitialConfigs(new TalonFXConfiguration()) // Set initial drive motor configs
                    .withSteerMotorInitialConfigs(new TalonFXConfiguration() // Set initial steer motor configs
                        .withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(Amps.of((double) constants.get("current_limit")))
                            .withStatorCurrentLimitEnable(true)))
                    .withEncoderInitialConfigs(new CANcoderConfiguration()) // Set initial encoder configs
                    .withSteerInertia(KilogramSquareMeters.of((double) constants.get("steer_inertia"))) // Set steer inertia (simulation only)
                    .withDriveInertia(KilogramSquareMeters.of((double) constants.get("drive_inertia"))) // Set drive inertia (simulation only)
                    .withSteerFrictionVoltage(Volts.of((double) constants.get("steer_friction_voltage"))) // Set steer friction voltage (simulation only)
                    .withDriveFrictionVoltage(Volts.of((double) constants.get("drive_friction_voltage"))); // Set drive friction voltage (simulation only)

            // Extract module configurations from the JSON file
            JSONObject modules = (JSONObject) constants.get("modules");

            JSONObject frontLeft = (JSONObject) modules.get("front_left");
            JSONObject frontRight = (JSONObject) modules.get("front_right");
            JSONObject backLeft = (JSONObject) modules.get("back_left");
            JSONObject backRight = (JSONObject) modules.get("back_right");

            // Create the swerve drivetrain using the configured constants
            drivetrain = new SwerveDrivetrain<EnhancedTalonFX, EnhancedTalonFX, CANcoder>(
                EnhancedTalonFX::new, EnhancedTalonFX::new, CANcoder::new, // Motor and encoder constructors

                new SwerveDrivetrainConstants()
                    .withCANBusName("rhino") // Set CAN bus name
                    .withPigeon2Id((int) (long) constants.get("gyro_id")) // Set gyro ID
                    .withPigeon2Configs(null), // Set gyro configs (null for default)

                SwerveConstants.odometryFrequency, // Set odometry update frequency

                // Configure each swerve module using the extracted constants
                getConstants(frontLeft, constantCreator),
                getConstants(frontRight, constantCreator),
                getConstants(backLeft, constantCreator),
                getConstants(backRight, constantCreator)
            );
        } catch (Exception e) {
            // Throw a runtime exception if there is an error during initialization
            throw new RuntimeException(e);
        }
    }

    /**
     * Helper method to create SwerveModuleConstants for a specific module using the provided JSON configuration.
     *
     * @param module The JSON object containing the module's configuration.
     * @param constantCreator The SwerveModuleConstantsFactory used to create the module constants.
     * @return A SwerveModuleConstants object configured for the specified module.
     */
    private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getConstants(
            JSONObject module, SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantCreator) {
        return constantCreator.createModuleConstants(
            (int) (long) ((JSONObject) module.get("angle_motor")).get("id"), // Angle motor ID
            (int) (long) ((JSONObject) module.get("drive_motor")).get("id"), // Drive motor ID
            (int) (long) ((JSONObject) module.get("encoder")).get("id"), // Encoder ID
            Rotations.of((double) ((JSONObject) module.get("encoder")).get("offset")), // Encoder offset
            Inches.of((double) (long) module.get("x")), // Module X position
            Inches.of((double) (long) module.get("y")), // Module Y position
            (boolean) ((JSONObject) module.get("drive_motor")).get("inverted"), // Drive motor inversion
            (boolean) ((JSONObject) module.get("angle_motor")).get("inverted"), // Angle motor inversion
            (boolean) ((JSONObject) module.get("encoder")).get("inverted") // Encoder inversion
        );
    }

    /**
     * Returns the configured swerve drivetrain.
     *
     * @return The configured SwerveDrivetrain object.
     */
    public SwerveDrivetrain<EnhancedTalonFX, EnhancedTalonFX, CANcoder> create() {
        return drivetrain;
    }

    /**
     * Returns the maximum speed of the robot.
     *
     * @return The maximum speed in meters per second.
     */
    public double getMaxSpeed() {
        return maxSpeed;
    }

    /**
     * Returns the maximum angular speed of the robot.
     *
     * @return The maximum angular speed in radians per second.
     */
    public double getMaxAngularSpeed() {
        return maxAngularSpeed;
    }
}