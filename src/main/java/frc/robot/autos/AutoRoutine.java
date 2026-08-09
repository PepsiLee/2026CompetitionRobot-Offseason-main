package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An autonomous command paired with the estimator pose that it expects at enable. */
public record AutoRoutine(Pose2d startingPose, Command command) {}
