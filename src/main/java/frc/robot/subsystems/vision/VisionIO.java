package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Camera boundary. Pose-estimator filtering belongs in Vision, not the Limelight adapter. */
public interface VisionIO {
  class Inputs {
    public double heartbeat;
    public boolean hasTargets;
    public Pose2d estimatedPose = Pose2d.kZero;
    public double timestampSeconds = Double.NaN;
    public int tagCount;
    public double averageDistanceMeters;
    public double pipelineLatencyMilliseconds;
  }

  default void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSecond) {}

  default void updateInputs(Inputs inputs) {}

  default String getName() {
    return "Vision";
  }
}
