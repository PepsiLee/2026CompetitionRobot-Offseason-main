package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Immutable collection of the geometry, limits, and CTRE constants used by Drive. */
public record DriveConfiguration(
    SwerveDrivetrainConstants drivetrainConstants,
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
        moduleConstants,
    SwerveDriveKinematics kinematics,
    double maxLinearSpeedMetersPerSecond,
    double maxAngularSpeedRadiansPerSecond) {}
