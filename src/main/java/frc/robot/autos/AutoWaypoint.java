package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;

/** One bounded drive-to-pose leg. */
public record AutoWaypoint(
    Pose2d pose,
    double maxSpeedMetersPerSecond,
    double positionToleranceMeters,
    double headingToleranceRadians,
    double timeoutSeconds) {}
