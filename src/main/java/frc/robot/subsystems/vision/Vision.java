package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.config.VisionConfiguration;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Filters MegaTag2 observations before adding them to CTRE's latency-aware pose
 * estimator.
 */
public final class Vision extends SubsystemBase {

  public enum RejectionReason {
    ACCEPTED,
    NO_TAGS,
    INVALID_TIMESTAMP,
    STALE,
    OUTSIDE_FIELD,
    TAG_TOO_FAR,
    ROTATING_TOO_FAST
  }

  private static final double MAX_MEASUREMENT_AGE_SECONDS = 0.50;
  private static final double RECENT_MEASUREMENT_SECONDS = 1.0;
  private static final double HEADING_STANDARD_DEVIATION_RADIANS = 1.0e6;

  private final RobotState robotState;
  private final Drive drive;
  private final VisionConfiguration configuration;
  private final VisionIO io;
  private final VisionIO.Inputs inputs = new VisionIO.Inputs();
  private RejectionReason lastRejectionReason = RejectionReason.NO_TAGS;

  public Vision(
      RobotState robotState, Drive drive, VisionConfiguration configuration, VisionIO io) {
    this.robotState = robotState;
    this.drive = drive;
    this.configuration = configuration;
    this.io = io;
  }

  @Override
  public void periodic() {
    double yawDegrees = robotState.getPose().getRotation().getDegrees();
    double yawRateDegreesPerSecond = Units.radiansToDegrees(drive.getAngularVelocityRadiansPerSecond());

    io.setRobotOrientation(yawDegrees, yawRateDegreesPerSecond);
    io.updateInputs(inputs);

    VisionMeasurement measurement = new VisionMeasurement(
        inputs.estimatedPose,
        inputs.timestampSeconds,
        inputs.tagCount,
        inputs.averageDistanceMeters);

    Pose2d referencePose = robotState.getPose();
    double poseDifferenceMeters = Double.NaN;
    if (measurement.tagCount() > 0
        && Double.isFinite(measurement.timestampSeconds())
        && measurement.timestampSeconds() > 0.0) {
      referencePose =
          drive.samplePoseAt(measurement.timestampSeconds()).orElse(referencePose);
      poseDifferenceMeters =
          referencePose
              .getTranslation()
              .getDistance(measurement.pose().getTranslation());
    }

    lastRejectionReason = evaluate(measurement, yawRateDegreesPerSecond);

    if (lastRejectionReason == RejectionReason.ACCEPTED) {
      double xyStandardDeviation = calculateXyStandardDeviation(measurement);
      drive.addVisionMeasurement(
          measurement.pose(),
          measurement.timestampSeconds(),
          VecBuilder.fill(
              xyStandardDeviation,
              xyStandardDeviation,
              HEADING_STANDARD_DEVIATION_RADIANS));
      Logger.recordOutput("Vision/XYStandardDeviation", xyStandardDeviation);
    }

    Logger.recordOutput("Vision/Heartbeat", inputs.heartbeat);
    Logger.recordOutput("Vision/Connected", inputs.connected);
    Logger.recordOutput("Vision/TargetValid", inputs.targetValid);
    Logger.recordOutput("Vision/PrimaryTagId", inputs.primaryTagId);
    Logger.recordOutput("Vision/PipelineIndex", inputs.pipelineIndex);
    Logger.recordOutput("Vision/PipelineType", inputs.pipelineType);
    Logger.recordOutput("Vision/Status", getStatus());
    Logger.recordOutput("Vision/HasTargets", inputs.hasTargets);
    Logger.recordOutput("Vision/EstimatedPose", inputs.estimatedPose);
    Logger.recordOutput("Vision/ReferencePose", referencePose);
    Logger.recordOutput("Vision/PoseDifferenceMeters", poseDifferenceMeters);
    Logger.recordOutput("Vision/TagCount", inputs.tagCount);
    Logger.recordOutput("Vision/AverageDistanceMeters", inputs.averageDistanceMeters);
    Logger.recordOutput("Vision/TimestampSeconds", inputs.timestampSeconds);
    Logger.recordOutput("Vision/RejectionReason", lastRejectionReason);
    double secondsSinceAcceptedMeasurement = Utils.getCurrentTimeSeconds()
        - robotState.getLastAcceptedVisionTimestampSeconds();
    boolean hasRecentMeasurement = Double.isFinite(secondsSinceAcceptedMeasurement)
        && secondsSinceAcceptedMeasurement <= RECENT_MEASUREMENT_SECONDS;
    Logger.recordOutput("Vision/HasRecentAcceptedMeasurement", hasRecentMeasurement);
    Logger.recordOutput("Vision/SecondsSinceAcceptedMeasurement",
        Double.isFinite(secondsSinceAcceptedMeasurement) ? secondsSinceAcceptedMeasurement
            : -1.0);
  }

  public RejectionReason evaluate(
      VisionMeasurement measurement, double yawRateDegreesPerSecond) {
    if (measurement.tagCount() <= 0) {
      return RejectionReason.NO_TAGS;
    }
    if (!Double.isFinite(measurement.timestampSeconds())
        || measurement.timestampSeconds() <= 0.0) {
      return RejectionReason.INVALID_TIMESTAMP;
    }
    double ageSeconds = Utils.getCurrentTimeSeconds() - measurement.timestampSeconds();
    if (ageSeconds < -0.05 || ageSeconds > MAX_MEASUREMENT_AGE_SECONDS) {
      return RejectionReason.STALE;
    }
    if (!isInsideField(measurement.pose())) {
      return RejectionReason.OUTSIDE_FIELD;
    }
    if (!Double.isFinite(measurement.averageDistanceMeters())
        || measurement.averageDistanceMeters() < 0.0
        || measurement.averageDistanceMeters() > configuration.maxTagDistanceMeters()) {
      return RejectionReason.TAG_TOO_FAR;
    }
    if (Math.abs(yawRateDegreesPerSecond) > configuration.maxAngularVelocityDegreesPerSecond()) {
      return RejectionReason.ROTATING_TOO_FAST;
    }
    return RejectionReason.ACCEPTED;
  }

  public RejectionReason getLastRejectionReason() {
    return lastRejectionReason;
  }

  private String getStatus() {
    if (!inputs.connected) {
      return "DISCONNECTED";
    }
    if (lastRejectionReason == RejectionReason.ACCEPTED) {
      return "ACCEPTED";
    }
    if (!inputs.targetValid) {
      return "NO_TARGET";
    }
    if (inputs.tagCount <= 0) {
      return "NO_MT2_POSE";
    }
    return "REJECTED_" + lastRejectionReason;
  }

  private boolean isInsideField(Pose2d pose) {
    double margin = configuration.fieldBoundaryMarginMeters();
    return pose.getX() >= -margin
        && pose.getX() <= FieldConstants.FIELD_LENGTH_METERS + margin
        && pose.getY() >= -margin
        && pose.getY() <= FieldConstants.FIELD_WIDTH_METERS + margin;
  }

  private static double calculateXyStandardDeviation(VisionMeasurement measurement) {
    double distanceSquared = measurement.averageDistanceMeters() * measurement.averageDistanceMeters();
    return measurement.tagCount() >= 2
        ? 0.20 + 0.05 * distanceSquared
        : 0.50 + 0.12 * distanceSquared;
  }
}
