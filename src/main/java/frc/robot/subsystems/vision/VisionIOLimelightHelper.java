package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.config.VisionConfiguration;

/** Raw NetworkTables adapter for a rear-facing Limelight running MegaTag2. */
public final class VisionIOLimelightHelper implements VisionIO {
  private final String name;

  public VisionIOLimelightHelper(VisionConfiguration configuration) {
    name = configuration.limelightName();
    publishCameraTransform(configuration.robotToCamera());
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSecond) {
    LimelightHelpers.SetRobotOrientation(name, yawDegrees, yawRateDegreesPerSecond, 0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(Inputs inputs) {

    final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    inputs.heartbeat = LimelightHelpers.getHeartbeat(name);

    if (poseEstimate_MegaTag1 == null
        || poseEstimate_MegaTag2 == null
        || poseEstimate_MegaTag1.tagCount == 0
        || poseEstimate_MegaTag2.tagCount == 0) {
      clearMeasurement(inputs);
      return;
    }

    poseEstimate_MegaTag2.pose = new Pose2d(
        poseEstimate_MegaTag2.pose.getTranslation(),
        poseEstimate_MegaTag1.pose.getRotation());

    inputs.pipelineLatencyMilliseconds = poseEstimate_MegaTag2.latency;
    inputs.tagCount = poseEstimate_MegaTag2.tagCount;
    inputs.averageDistanceMeters = poseEstimate_MegaTag2.avgTagDist;
    inputs.hasTargets = inputs.tagCount > 0;
    inputs.estimatedPose = poseEstimate_MegaTag2.pose;
    inputs.timestampSeconds = poseEstimate_MegaTag2.timestampSeconds;
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

  private static void clearMeasurement(Inputs inputs) {
    inputs.hasTargets = false;
    inputs.tagCount = 0;
    inputs.averageDistanceMeters = Double.NaN;
    inputs.timestampSeconds = Double.NaN;
    inputs.pipelineLatencyMilliseconds = Double.NaN;
  }
}
