package frc.robot.config;

import edu.wpi.first.math.geometry.Transform3d;

/** Limelight mounting and measurement-rejection limits. */
public record VisionConfiguration(
    String limelightName,
    Transform3d robotToCamera,
    double maxTagDistanceMeters,
    double maxAngularVelocityDegreesPerSecond,
    double maxPoseJumpMeters,
    double fieldBoundaryMarginMeters) {}
