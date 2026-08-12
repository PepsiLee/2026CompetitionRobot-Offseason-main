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

  public enum HeadingRejectionReason {
    ACCEPTED,
    NO_MT1_POSE,
    SINGLE_TAG_AMBIGUOUS,
    INVALID_TIMESTAMP,
    STALE,
    OUTSIDE_FIELD,
    TAG_TOO_FAR,
    ROTATING_TOO_FAST
  }

  private static final double MAX_MEASUREMENT_AGE_SECONDS = 0.50;
  private static final double RECENT_MEASUREMENT_SECONDS = 1.0;
  private static final double HEADING_STANDARD_DEVIATION_RADIANS = 1.0e6;
  private static final double MT1_XY_STANDARD_DEVIATION_METERS = 1.0e6;
  private static final double MT1_HEADING_STANDARD_DEVIATION_RADIANS = 0.10;
  private static final double MT1_SINGLE_TAG_MAX_AMBIGUITY = 0.15;
  private static final double MT1_SINGLE_TAG_MAX_DISTANCE_METERS = 3.0;

  private final RobotState robotState;
  private final Drive drive;
  private final VisionConfiguration configuration;
  private final VisionIO io;
  private final VisionIO.Inputs inputs = new VisionIO.Inputs();
  private RejectionReason lastRejectionReason = RejectionReason.NO_TAGS;
  private HeadingRejectionReason lastHeadingRejectionReason =
      HeadingRejectionReason.NO_MT1_POSE;

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

    VisionMeasurement mt1Measurement = new VisionMeasurement(
        inputs.mt1EstimatedPose,
        inputs.mt1TimestampSeconds,
        inputs.mt1TagCount,
        inputs.mt1AverageDistanceMeters);
    lastHeadingRejectionReason = evaluateMt1Heading(
        mt1Measurement,
        inputs.mt1MaximumAmbiguity,
        yawRateDegreesPerSecond);
    if (lastHeadingRejectionReason == HeadingRejectionReason.ACCEPTED) {
      drive.addVisionMeasurement(
          mt1Measurement.pose(),
          mt1Measurement.timestampSeconds(),
          VecBuilder.fill(
              MT1_XY_STANDARD_DEVIATION_METERS,
              MT1_XY_STANDARD_DEVIATION_METERS,
              MT1_HEADING_STANDARD_DEVIATION_RADIANS));
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
    Logger.recordOutput("Vision/MT1HeadingPose", inputs.mt1EstimatedPose);
    Logger.recordOutput("Vision/MT1HeadingTagCount", inputs.mt1TagCount);
    Logger.recordOutput("Vision/MT1HeadingAverageDistanceMeters", inputs.mt1AverageDistanceMeters);
    Logger.recordOutput("Vision/MT1HeadingMaximumAmbiguity", inputs.mt1MaximumAmbiguity);
    Logger.recordOutput("Vision/MT1HeadingTimestampSeconds", inputs.mt1TimestampSeconds);
    Logger.recordOutput("Vision/MT1HeadingStatus", lastHeadingRejectionReason);
    Logger.recordOutput(
        "Vision/MT1HeadingAccepted",
        lastHeadingRejectionReason == HeadingRejectionReason.ACCEPTED);
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

  public HeadingRejectionReason evaluateMt1Heading(
      VisionMeasurement measurement,
      double maximumAmbiguity,
      double yawRateDegreesPerSecond) {
    if (measurement.tagCount() <= 0) {
      return HeadingRejectionReason.NO_MT1_POSE;
    }
    if (measurement.tagCount() == 1
        && (!Double.isFinite(maximumAmbiguity)
            || maximumAmbiguity > MT1_SINGLE_TAG_MAX_AMBIGUITY
            || !Double.isFinite(measurement.averageDistanceMeters())
            || measurement.averageDistanceMeters() > MT1_SINGLE_TAG_MAX_DISTANCE_METERS)) {
      return HeadingRejectionReason.SINGLE_TAG_AMBIGUOUS;
    }
    if (!Double.isFinite(measurement.timestampSeconds())
        || measurement.timestampSeconds() <= 0.0) {
      return HeadingRejectionReason.INVALID_TIMESTAMP;
    }
    double ageSeconds = Utils.getCurrentTimeSeconds() - measurement.timestampSeconds();
    if (ageSeconds < -0.05 || ageSeconds > MAX_MEASUREMENT_AGE_SECONDS) {
      return HeadingRejectionReason.STALE;
    }
    if (!isInsideField(measurement.pose())) {
      return HeadingRejectionReason.OUTSIDE_FIELD;
    }
    if (!Double.isFinite(measurement.averageDistanceMeters())
        || measurement.averageDistanceMeters() < 0.0
        || measurement.averageDistanceMeters() > configuration.maxTagDistanceMeters()) {
      return HeadingRejectionReason.TAG_TOO_FAR;
    }
    if (Math.abs(yawRateDegreesPerSecond) > configuration.maxAngularVelocityDegreesPerSecond()) {
      return HeadingRejectionReason.ROTATING_TOO_FAST;
    }
    return HeadingRejectionReason.ACCEPTED;
  }

  public HeadingRejectionReason getLastHeadingRejectionReason() {
    return lastHeadingRejectionReason;
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
