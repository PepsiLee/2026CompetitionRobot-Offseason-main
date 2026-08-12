package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Camera boundary. Pose-estimator filtering belongs in Vision, not the Limelight adapter. */
public interface VisionIO {
  class Inputs {
    public double heartbeat = Double.NaN;
    public boolean connected;
    public boolean targetValid;
    public int primaryTagId = -1;
    public int pipelineIndex = -1;
    public String pipelineType = "";
    public boolean hasTargets;
    public Pose2d estimatedPose = Pose2d.kZero;
    /** Timestamp in the same timebase as {@code Utils.getCurrentTimeSeconds()}. */
    public double timestampSeconds = Double.NaN;
    public int tagCount;
    public double averageDistanceMeters = Double.NaN;
    public double pipelineLatencyMilliseconds = Double.NaN;
    public Pose2d mt1EstimatedPose = Pose2d.kZero;
    /** MT1 timestamp in the same timebase as {@code Utils.getCurrentTimeSeconds()}. */
    public double mt1TimestampSeconds = Double.NaN;
    public int mt1TagCount;
    public double mt1AverageDistanceMeters = Double.NaN;
    public double mt1MaximumAmbiguity = Double.NaN;
    public double mt1PipelineLatencyMilliseconds = Double.NaN;
  }

  default void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSecond) {}

  default void updateInputs(Inputs inputs) {}

  default String getName() {
    return "Vision";
  }
}
