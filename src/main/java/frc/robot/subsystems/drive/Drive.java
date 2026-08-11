package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.config.DriveConfiguration;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Driver-facing swerve subsystem with mutually exclusive teleop, aim, and auto
 * modes.
 */
public final class Drive extends SubsystemBase {
  public enum ControlMode {
    TELEOP,
    AIM_STATIONARY,
    DRIVE_TO_POSE,
    STOPPED
  }

  // TODO: move this field to the constant file
  private static final double JOYSTICK_DEADBAND = 0.10;
  private static final double TELEOP_TRANSLATION_SCALE = 0.4;
  private static final double TELEOP_ROTATION_SCALE = 0.2;
  private static final double SETPOINT_STABLE_TIME_SECONDS = 0.10;

  // Why there is a robot state here?
  private final RobotState robotState;
  private final DriveIO io;
  // ?
  private final DriveIO.DriveIOInputs inputs = new DriveIO.DriveIOInputs();
  private final Field2d field = new Field2d();
  private final double maxLinearSpeedMetersPerSecond;
  private final double maxAngularSpeedRadiansPerSecond;
  private final PIDController xController = new PIDController(2.5, 0.0, 0.05);
  private final PIDController yController = new PIDController(2.5, 0.0, 0.05);
  private final ProfiledPIDController headingController;
  private final Debouncer driveToPoseDebouncer = new Debouncer(SETPOINT_STABLE_TIME_SECONDS,
      Debouncer.DebounceType.kRising);

  private ControlMode controlMode = ControlMode.TELEOP;
  private double teleopXInput;
  private double teleopYInput;
  private double teleopOmegaInput;
  private Rotation2d targetHeading = Rotation2d.kZero;
  private Pose2d targetPose = Pose2d.kZero;
  private double driveToPoseMaxSpeedMetersPerSecond;
  private double driveToPosePositionToleranceMeters = 0.10;
  private double driveToPoseHeadingToleranceRadians = Math.toRadians(4.0);

  public Drive(RobotState robotState, DriveIO io, DriveConfiguration configuration) {
    this.robotState = robotState;
    this.io = io;
    maxLinearSpeedMetersPerSecond = configuration.maxLinearSpeedMetersPerSecond();
    maxAngularSpeedRadiansPerSecond = configuration.maxAngularSpeedRadiansPerSecond();
    driveToPoseMaxSpeedMetersPerSecond = maxLinearSpeedMetersPerSecond;
    headingController = new ProfiledPIDController(
        5.0,
        0.0,
        0.25,
        new TrapezoidProfile.Constraints(
            Math.min(maxAngularSpeedRadiansPerSecond, 6.0),
            Math.min(maxAngularSpeedRadiansPerSecond * 2.0, 12.0)));
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    robotState.addDriveObservation(
        inputs.timestampSeconds, inputs.pose, inputs.measuredRobotRelativeSpeeds);
    field.setRobotPose(inputs.pose);

    applyControlMode();

    Logger.recordOutput("Drive/ControlMode", controlMode);
    Logger.recordOutput("Drive/Pose", inputs.pose);
    Logger.recordOutput("Drive/GyroYaw", inputs.gyroYaw);
    Logger.recordOutput("Drive/MeasuredSpeeds", inputs.measuredRobotRelativeSpeeds);
    Logger.recordOutput("Drive/ModuleStates", inputs.moduleStates);
    Logger.recordOutput("Drive/ModuleTargets", inputs.moduleTargets);
    Logger.recordOutput("Drive/ModulePositions", inputs.modulePositions);
    Logger.recordOutput("Drive/TargetHeading", targetHeading);
    Logger.recordOutput("Drive/TargetPose", targetPose);
    Logger.recordOutput("Drive/HeadingErrorRadians", getHeadingErrorRadians(targetHeading));
  }

  /**
   * Stores driver input; the periodic control-mode switch is the only writer to
   * DriveIO.
   */
  public void acceptTeleopInput(double xInput, double yInput, double omegaInput) {
    teleopXInput = xInput;
    teleopYInput = yInput;
    teleopOmegaInput = omegaInput;
  }

  public void requestTeleop() {
    controlMode = ControlMode.TELEOP;
  }

  public void requestAimStationary(Rotation2d heading) {
    if (controlMode != ControlMode.AIM_STATIONARY) {
      headingController.reset(
          inputs.pose.getRotation().getRadians(),
          inputs.measuredRobotRelativeSpeeds.omegaRadiansPerSecond);
    }
    targetHeading = heading;
    controlMode = ControlMode.AIM_STATIONARY;
  }

  public void releaseAim() {
    if (controlMode == ControlMode.AIM_STATIONARY
        || (controlMode == ControlMode.STOPPED && !DriverStation.isAutonomousEnabled())) {
      controlMode = DriverStation.isAutonomousEnabled() ? ControlMode.STOPPED : ControlMode.TELEOP;
    }
  }

  public void requestDriveToPose(
      Pose2d pose,
      double maxSpeedMetersPerSecond,
      double positionToleranceMeters,
      double headingToleranceRadians) {
    targetPose = pose;
    targetHeading = pose.getRotation();
    driveToPoseMaxSpeedMetersPerSecond = MathUtil.clamp(maxSpeedMetersPerSecond, 0.0, maxLinearSpeedMetersPerSecond);
    driveToPosePositionToleranceMeters = positionToleranceMeters;
    driveToPoseHeadingToleranceRadians = headingToleranceRadians;
    xController.reset();
    yController.reset();
    headingController.reset(
        inputs.pose.getRotation().getRadians(),
        inputs.measuredRobotRelativeSpeeds.omegaRadiansPerSecond);
    driveToPoseDebouncer.calculate(false);
    controlMode = ControlMode.DRIVE_TO_POSE;
  }

  public boolean isAtDriveToPoseSetpoint() {
    boolean rawAtSetpoint = inputs.pose.getTranslation()
        .getDistance(targetPose.getTranslation()) <= driveToPosePositionToleranceMeters
        && Math.abs(getHeadingErrorRadians(targetPose.getRotation())) <= driveToPoseHeadingToleranceRadians;
    return driveToPoseDebouncer.calculate(rawAtSetpoint);
  }

  public boolean isAtHeading(Rotation2d heading, double toleranceRadians) {
    return Math.abs(getHeadingErrorRadians(heading)) <= toleranceRadians;
  }

  public double getHeadingErrorRadians(Rotation2d heading) {
    return MathUtil.angleModulus(
        heading.getRadians() - inputs.pose.getRotation().getRadians());
  }

  public double getAngularVelocityRadiansPerSecond() {
    return inputs.measuredRobotRelativeSpeeds.omegaRadiansPerSecond;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void stop() {
    controlMode = ControlMode.STOPPED;
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  public void resetHeadingForAlliance() {
    Rotation2d heading = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? Rotation2d.k180deg
        : Rotation2d.kZero;
    resetPose(new Pose2d(getPose().getTranslation(), heading));
  }

  public Pose2d getPose() {
    return robotState.getPose();
  }

  public Field2d getField() {
    return field;
  }

  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return io.samplePoseAt(timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {
    io.addVisionMeasurement(pose, timestampSeconds, standardDeviations);
    robotState.recordAcceptedVisionMeasurement(timestampSeconds);
  }

  public MapleSimSwerveDrivetrain getMapleSimDrive() {
    if (io instanceof DriveIOSim simulationIO) {
      return simulationIO.getMapleSimDrive();
    }
    return null;
  }

  private void applyControlMode() {
    switch (controlMode) {
      case TELEOP -> io.runVelocity(calculateTeleopSpeeds());
      case AIM_STATIONARY -> {
        double omega = headingController.calculate(
            inputs.pose.getRotation().getRadians(), targetHeading.getRadians());
        omega = MathUtil.clamp(
            omega,
            -maxAngularSpeedRadiansPerSecond,
            maxAngularSpeedRadiansPerSecond);
        io.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));
      }
      case DRIVE_TO_POSE -> io.runVelocity(calculateDriveToPoseSpeeds());
      case STOPPED -> io.stop();
    }
  }

  private ChassisSpeeds calculateTeleopSpeeds() {
    double x = shapeJoystick(teleopXInput);
    double y = shapeJoystick(teleopYInput);
    double omega = shapeJoystick(teleopOmegaInput);
    double allianceMultiplier = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1.0 : 1.0;

    var fieldRelativeSpeeds = new ChassisSpeeds(
        x * maxLinearSpeedMetersPerSecond * TELEOP_TRANSLATION_SCALE * allianceMultiplier,
        y * maxLinearSpeedMetersPerSecond * TELEOP_TRANSLATION_SCALE * allianceMultiplier,
        omega * maxAngularSpeedRadiansPerSecond * TELEOP_ROTATION_SCALE);
    return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, inputs.pose.getRotation());
  }

  private ChassisSpeeds calculateDriveToPoseSpeeds() {
    double vx = xController.calculate(inputs.pose.getX(), targetPose.getX());
    double vy = yController.calculate(inputs.pose.getY(), targetPose.getY());
    double speed = Math.hypot(vx, vy);
    if (speed > driveToPoseMaxSpeedMetersPerSecond && speed > 1.0e-9) {
      double scalar = driveToPoseMaxSpeedMetersPerSecond / speed;
      vx *= scalar;
      vy *= scalar;
    }
    double omega = headingController.calculate(
        inputs.pose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    omega = MathUtil.clamp(
        omega,
        -maxAngularSpeedRadiansPerSecond,
        maxAngularSpeedRadiansPerSecond);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(vx, vy, omega), inputs.pose.getRotation());
  }

  private static double shapeJoystick(double input) {
    double value = MathUtil.applyDeadband(input, JOYSTICK_DEADBAND);
    return Math.copySign(value * value, value);
  }
}
