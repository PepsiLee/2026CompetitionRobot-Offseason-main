package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Shared, hardware-independent state that other subsystems can read without depending on Drive. */
public final class RobotState {
  private Pose2d fieldToRobot = Pose2d.kZero;
  private ChassisSpeeds measuredRobotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds measuredFieldRelativeSpeeds = new ChassisSpeeds();
  private double lastTimestampSeconds = Double.NEGATIVE_INFINITY;
  private double lastAcceptedVisionTimestampSeconds = Double.NEGATIVE_INFINITY;

  public synchronized void addDriveObservation(
      double timestampSeconds, Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
    if (timestampSeconds < lastTimestampSeconds) {
      return;
    }

    lastTimestampSeconds = timestampSeconds;
    fieldToRobot = pose;
    measuredRobotRelativeSpeeds = robotRelativeSpeeds;
    measuredFieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());
  }

  public synchronized Pose2d getPose() {
    return fieldToRobot;
  }

  public synchronized ChassisSpeeds getMeasuredRobotRelativeSpeeds() {
    return measuredRobotRelativeSpeeds;
  }

  public synchronized ChassisSpeeds getMeasuredFieldRelativeSpeeds() {
    return measuredFieldRelativeSpeeds;
  }

  public synchronized void recordAcceptedVisionMeasurement(double timestampSeconds) {
    lastAcceptedVisionTimestampSeconds =
        Math.max(lastAcceptedVisionTimestampSeconds, timestampSeconds);
  }

  public synchronized double getLastAcceptedVisionTimestampSeconds() {
    return lastAcceptedVisionTimestampSeconds;
  }
}
