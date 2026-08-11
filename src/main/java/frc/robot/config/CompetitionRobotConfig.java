package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Hardware configuration for the competition robot.
 *
 * <p>
 * Hardware values are synchronized with the robot's Phoenix Tuner X swerve
 * project.
 */
public final class CompetitionRobotConfig implements RobotConfiguration {
    private static final String CAN_BUS = RobotBase.isSimulation() ? "*" : "canivore";

    private static final int PIGEON_ID = 0;

    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
    // SDS MK5i R2: 14T drive pinion, 6.03:1 overall drive ratio.
    private static final double DRIVE_PINION_TEETH = 14.0;
    private static final double COUPLING_RATIO = 54.0 / DRIVE_PINION_TEETH;
    private static final double DRIVE_RATIO = COUPLING_RATIO * (25.0 / 32.0) * (30.0 / 15.0);
    private static final double STEER_RATIO = 26.0;
    // Measured in Soildwork
    private static final double WHEELBASE_METERS = Units.inchesToMeters(22.25);
    private static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.25);
    // FIXME: Unknown Source
    private static final double SPEED_AT_12_VOLTS_METERS_PER_SECOND = 5.12;
    private static final double MAX_LINEAR_SPEED_METERS_PER_SECOND = 5.12;

    // Order is always front-left, front-right, back-left, back-right.
    private static final ModuleHardware FRONT_LEFT = new ModuleHardware(1, 2, 11, 0.094482421875, false);
    private static final ModuleHardware FRONT_RIGHT = new ModuleHardware(3, 4, 12, 0.003173828125, true);
    private static final ModuleHardware BACK_LEFT = new ModuleHardware(5, 6, 13, -0.36669921875, false);
    private static final ModuleHardware BACK_RIGHT = new ModuleHardware(7, 8, 14, 0.171142578125, true);

    private final DriveConfiguration driveConfiguration;
    private final IntakeConfiguration intakeConfiguration;
    private final ShooterConfiguration shooterConfiguration;
    private final VisionConfiguration visionConfiguration;
    private final FeederConfiguration feederConfiguration;

    public CompetitionRobotConfig() {
        var drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(CAN_BUS)
                .withPigeon2Id(PIGEON_ID);

        var driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;

        var steerMotorConfig = new TalonFXConfiguration();
        steerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerMotorConfig.CurrentLimits.StatorCurrentLimit = 60.0;

        double halfWheelbase = WHEELBASE_METERS / 2.0;
        double halfTrackWidth = TRACK_WIDTH_METERS / 2.0;

        @SuppressWarnings("unchecked")
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] modules = new SwerveModuleConstants[] {
                createModule(
                        FRONT_LEFT, halfWheelbase, halfTrackWidth, driveMotorConfig, steerMotorConfig),
                createModule(
                        FRONT_RIGHT, halfWheelbase, -halfTrackWidth, driveMotorConfig, steerMotorConfig),
                createModule(
                        BACK_LEFT, -halfWheelbase, halfTrackWidth, driveMotorConfig, steerMotorConfig),
                createModule(
                        BACK_RIGHT, -halfWheelbase, -halfTrackWidth, driveMotorConfig, steerMotorConfig)
        };

        var kinematics = new SwerveDriveKinematics(
                new Translation2d(halfWheelbase, halfTrackWidth),
                new Translation2d(halfWheelbase, -halfTrackWidth),
                new Translation2d(-halfWheelbase, halfTrackWidth),
                new Translation2d(-halfWheelbase, -halfTrackWidth));

        double drivebaseRadius = Math.hypot(halfWheelbase, halfTrackWidth);
        driveConfiguration = new DriveConfiguration(
                drivetrainConstants,
                modules,
                kinematics,
                MAX_LINEAR_SPEED_METERS_PER_SECOND,
                MAX_LINEAR_SPEED_METERS_PER_SECOND / drivebaseRadius);

        // CAN 9 runs at +7 V whenever teleop is enabled. CAN 10 runs at +3 V while
        // Circle is held
        // (and while an autonomous intake request is active).
        intakeConfiguration = new IntakeConfiguration(CAN_BUS, 9, 10, 7.0, 3.0, 40.0, 80.0);

        // All three shooter motors are independent VoltageOut devices; none is
        // configured as a
        // follower. Signed voltages encode the requested physical direction.
        shooterConfiguration = new ShooterConfiguration(
                CAN_BUS,
                30,
                31,
                -7.0,
                7.0,
                35.0,
                60.0,
                1.222);

        // Provisional rear-facing transform. Replace with a measured robot-to-camera
        // transform.
        visionConfiguration = new VisionConfiguration(
                "limelight-rear",
                // TODO: Need to be check in the CAD
                new Transform3d(
                        new Translation3d(-0.332486, 0.0, 0.171958),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                6.0,
                720.0,
                1.0,
                0.5);

        feederConfiguration = new FeederConfiguration(
                CAN_BUS,
                32,
                7.0,
                -7.0,
                12.0,
                120);
    }

    @Override
    public DriveConfiguration drive() {
        return driveConfiguration;
    }

    @Override
    public IntakeConfiguration intake() {
        return intakeConfiguration;
    }

    @Override
    public ShooterConfiguration shooter() {
        return shooterConfiguration;
    }

    @Override
    public VisionConfiguration vision() {
        return visionConfiguration;
    }

    @Override
    public FeederConfiguration feeder() {
        return feederConfiguration;
    }

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createModule(
            ModuleHardware hardware,
            double locationX,
            double locationY,
            TalonFXConfiguration driveMotorConfig,
            TalonFXConfiguration steerMotorConfig) {
        return new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(hardware.driveMotorId())
                .withSteerMotorId(hardware.steerMotorId())
                .withEncoderId(hardware.encoderId())
                .withDriveMotorGearRatio(DRIVE_RATIO)
                .withSteerMotorGearRatio(STEER_RATIO)
                .withCouplingGearRatio(COUPLING_RATIO)
                .withDriveMotorInverted(hardware.driveMotorInverted())
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(hardware.encoderOffsetRotations())
                .withLocationX(locationX)
                .withLocationY(locationY)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(
                        new Slot0Configs()
                                .withKP(0.1)
                                .withKI(0.0)
                                .withKD(0.0)
                                .withKS(0.0)
                                .withKV(0.124))
                .withSteerMotorGains(
                        new Slot0Configs()
                                .withKP(100.0)
                                .withKI(0.0)
                                .withKD(0.5)
                                .withKS(0.1)
                                .withKV(3.23)
                                .withKA(0.0)
                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveMotorConfig)
                .withSteerMotorInitialConfigs(steerMotorConfig)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.2)
                .withSteerFrictionVoltage(0.2)
                .withDriveInertia(0.035)
                .withSteerInertia(0.01)
                .withSlipCurrent(120.0)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(SPEED_AT_12_VOLTS_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);
    }

    private record ModuleHardware(
            int driveMotorId,
            int steerMotorId,
            int encoderId,
            double encoderOffsetRotations,
            boolean driveMotorInverted) {
    }
}
