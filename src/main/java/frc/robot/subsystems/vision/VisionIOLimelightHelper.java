package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.config.VisionConfiguration;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

/** Raw NetworkTables adapter for a rear-facing Limelight running MegaTag2. */
public final class VisionIOLimelightHelper implements VisionIO {
  private static final double CONNECTION_TIMEOUT_SECONDS = 0.5;

  private final String name;
  private final DoubleSupplier currentTimeSeconds;
  private final DoubleUnaryOperator fpgaToCurrentTime;
  private double lastHeartbeat = Double.NaN;
  private double lastHeartbeatChangeTimestampSeconds = Double.NEGATIVE_INFINITY;

  public VisionIOLimelightHelper(VisionConfiguration configuration) {
    this(configuration, Utils::getCurrentTimeSeconds, Utils::fpgaToCurrentTime);
  }

  VisionIOLimelightHelper(
      VisionConfiguration configuration,
      DoubleSupplier currentTimeSeconds,
      DoubleUnaryOperator fpgaToCurrentTime) {
    name = configuration.limelightName();
    this.currentTimeSeconds = currentTimeSeconds;
    this.fpgaToCurrentTime = fpgaToCurrentTime;
    publishCameraTransform(configuration.robotToCamera());
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSecond) {
    LimelightHelpers.SetRobotOrientation(name, yawDegrees, yawRateDegreesPerSecond, 0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    updateDiagnostics(inputs);

    final PoseEstimate megaTag1Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    updateMegaTag1Inputs(inputs, megaTag1Estimate);

    final PoseEstimate megaTag2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (megaTag2Estimate == null || megaTag2Estimate.tagCount == 0) {
      clearMeasurement(inputs);
      return;
    }

    inputs.pipelineLatencyMilliseconds = megaTag2Estimate.latency;
    inputs.tagCount = megaTag2Estimate.tagCount;
    inputs.averageDistanceMeters = megaTag2Estimate.avgTagDist;
    inputs.hasTargets = inputs.tagCount > 0;
    inputs.estimatedPose = megaTag2Estimate.pose;
    inputs.timestampSeconds = fpgaToCurrentTime.applyAsDouble(megaTag2Estimate.timestampSeconds);
  }

  @Override
  public String getName() {
    return name;
  }

  private void publishCameraTransform(Transform3d robotToCamera) {
    LimelightHelpers.setCameraPose_RobotSpace(name, robotToCamera.getX(),
        robotToCamera.getY(),
        robotToCamera.getZ(),
        Units.radiansToDegrees(robotToCamera.getRotation().getX()),
        Units.radiansToDegrees(robotToCamera.getRotation().getY()),
        Units.radiansToDegrees(robotToCamera.getRotation().getZ()));
  }

  private void updateDiagnostics(Inputs inputs) {
    double nowSeconds = currentTimeSeconds.getAsDouble();
    inputs.heartbeat = LimelightHelpers.getHeartbeat(name);

    if (!Double.isFinite(lastHeartbeat)) {
      lastHeartbeat = inputs.heartbeat;
      if (inputs.heartbeat != 0.0) {
        lastHeartbeatChangeTimestampSeconds = nowSeconds;
      }
    } else if (Double.compare(inputs.heartbeat, lastHeartbeat) != 0) {
      lastHeartbeat = inputs.heartbeat;
      lastHeartbeatChangeTimestampSeconds = nowSeconds;
    }

    inputs.connected =
        nowSeconds - lastHeartbeatChangeTimestampSeconds <= CONNECTION_TIMEOUT_SECONDS;
    inputs.targetValid = inputs.connected && LimelightHelpers.getTV(name);
    inputs.primaryTagId =
        inputs.targetValid ? (int) Math.round(LimelightHelpers.getFiducialID(name)) : -1;
    inputs.pipelineIndex =
        inputs.connected
            ? (int) Math.round(LimelightHelpers.getCurrentPipelineIndex(name))
            : -1;
    inputs.pipelineType =
        inputs.connected ? LimelightHelpers.getCurrentPipelineType(name) : "";
  }

  private void updateMegaTag1Inputs(Inputs inputs, PoseEstimate estimate) {
    if (estimate == null || estimate.tagCount == 0) {
      clearMegaTag1Measurement(inputs);
      return;
    }

    inputs.mt1EstimatedPose = estimate.pose;
    inputs.mt1TimestampSeconds = fpgaToCurrentTime.applyAsDouble(estimate.timestampSeconds);
    inputs.mt1TagCount = estimate.tagCount;
    inputs.mt1AverageDistanceMeters = estimate.avgTagDist;
    inputs.mt1MaximumAmbiguity = maximumAmbiguity(estimate);
    inputs.mt1PipelineLatencyMilliseconds = estimate.latency;
  }

  private static double maximumAmbiguity(PoseEstimate estimate) {
    if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return Double.NaN;
    }
    double maximum = 0.0;
    for (LimelightHelpers.RawFiducial fiducial : estimate.rawFiducials) {
      maximum = Math.max(maximum, fiducial.ambiguity);
    }
    return maximum;
  }

  private static void clearMeasurement(Inputs inputs) {
    inputs.hasTargets = false;
    inputs.estimatedPose = Pose2d.kZero;
    inputs.tagCount = 0;
    inputs.averageDistanceMeters = Double.NaN;
    inputs.timestampSeconds = Double.NaN;
    inputs.pipelineLatencyMilliseconds = Double.NaN;
  }

  private static void clearMegaTag1Measurement(Inputs inputs) {
    inputs.mt1EstimatedPose = Pose2d.kZero;
    inputs.mt1TimestampSeconds = Double.NaN;
    inputs.mt1TagCount = 0;
    inputs.mt1AverageDistanceMeters = Double.NaN;
    inputs.mt1MaximumAmbiguity = Double.NaN;
    inputs.mt1PipelineLatencyMilliseconds = Double.NaN;
  }
}
