package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.config.VisionConfiguration;

/** Raw NetworkTables adapter for a rear-facing Limelight running MegaTag2. */
public final class VisionIOLimelight implements VisionIO {
  private static final int MINIMUM_BOTPOSE_LENGTH = 11;
  private final String name;
  private final NetworkTable table;
  private final DoubleArraySubscriber megaTag2PoseSubscriber;
  private final DoubleSubscriber heartbeatSubscriber;
  private long lastProcessedPoseTimestampMicroseconds;

  public VisionIOLimelight(VisionConfiguration configuration) {
    name = configuration.limelightName();
    table = NetworkTableInstance.getDefault().getTable(name);
    megaTag2PoseSubscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(Double.NaN);
    publishCameraTransform(configuration.robotToCamera());
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSecond) {
    table
        .getEntry("robot_orientation_set")
        .setDoubleArray(
            new double[] {yawDegrees, yawRateDegreesPerSecond, 0.0, 0.0, 0.0, 0.0});
  }

  @Override
  public void updateInputs(Inputs inputs) {
    TimestampedDouble heartbeat = heartbeatSubscriber.getAtomic();
    inputs.heartbeat = heartbeat.value;
    // long nowMicroseconds = (long) (Timer.getFPGATimestamp() * 1.0e6);
    // inputs.heartbeat =
    //     heartbeat.timestamp > 0
    //         && Math.abs(nowMicroseconds - heartbeat.timestamp) <= CONNECTION_TIMEOUT_MICROSECONDS;

    TimestampedDoubleArray atomicPose = megaTag2PoseSubscriber.getAtomic();
    double[] poseArray = atomicPose.value;
    if (poseArray.length < MINIMUM_BOTPOSE_LENGTH
        || atomicPose.timestamp <= 0
        || atomicPose.timestamp == lastProcessedPoseTimestampMicroseconds) {
      clearMeasurement(inputs);
      return;
    }
    lastProcessedPoseTimestampMicroseconds = atomicPose.timestamp;

    inputs.pipelineLatencyMilliseconds = poseArray[6];
    inputs.tagCount = (int) Math.round(poseArray[7]);
    inputs.averageDistanceMeters = poseArray[9];
    inputs.hasTargets = inputs.tagCount > 0;
    inputs.estimatedPose =
        new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
    inputs.timestampSeconds =
        Utils.fpgaToCurrentTime(
            atomicPose.timestamp / 1.0e6 - inputs.pipelineLatencyMilliseconds / 1000.0);
  }

  @Override
  public String getName() {
    return name;
  }

  private void publishCameraTransform(Transform3d robotToCamera) {
    table
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(
            new double[] {
              robotToCamera.getX(),
              robotToCamera.getY(),
              robotToCamera.getZ(),
              Units.radiansToDegrees(robotToCamera.getRotation().getX()),
              Units.radiansToDegrees(robotToCamera.getRotation().getY()),
              Units.radiansToDegrees(robotToCamera.getRotation().getZ())
            });
  }

  private static void clearMeasurement(Inputs inputs) {
    inputs.hasTargets = false;
    inputs.tagCount = 0;
    inputs.averageDistanceMeters = Double.NaN;
    inputs.timestampSeconds = Double.NaN;
    inputs.pipelineLatencyMilliseconds = Double.NaN;
  }
}
