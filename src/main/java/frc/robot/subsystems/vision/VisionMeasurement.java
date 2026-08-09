package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Timestamped MegaTag2 measurement expressed in the WPILib blue-origin frame. */
public record VisionMeasurement(
    Pose2d pose, double timestampSeconds, int tagCount, double averageDistanceMeters) {}
