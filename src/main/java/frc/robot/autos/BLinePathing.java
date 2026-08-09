package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Robot-specific BLine loading, feedback, output, field transforms, and logging. */
public final class BLinePathing {
  public static final String INTAKE_SHOOT_PATH_NAME = "phase-1-canvas-draft";

  private static final double TRANSLATION_KP = 2.0;
  private static final double ROTATION_KP = 1.0;
  private static final double CROSS_TRACK_KP = 0.2;

  private final Drive drive;

  public BLinePathing(Drive drive) {
    this.drive = drive;

    FlippingUtil.fieldSizeX = FieldConstants.FIELD_LENGTH_METERS;
    FlippingUtil.fieldSizeY = FieldConstants.FIELD_WIDTH_METERS;
    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;

    FollowPath.setPoseLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setDoubleLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setBooleanLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
  }

  public Path loadIntakeShootPath() {
    return new Path(INTAKE_SHOOT_PATH_NAME);
  }

  public FollowPath createFollowCommand(Path path, Alliance alliance) {
    return new FollowPath.Builder(
            drive,
            drive::getPose,
            drive::getMeasuredRobotRelativeSpeeds,
            drive::requestBLineSpeeds,
            new PIDController(TRANSLATION_KP, 0.0, 0.0),
            new PIDController(ROTATION_KP, 0.0, 0.0),
            new PIDController(CROSS_TRACK_KP, 0.0, 0.0))
        .withShouldFlip(() -> alliance == Alliance.Red)
        .build(path);
  }

  public Pose2d getStartPose(Path path, Alliance alliance) {
    Path alliancePath = path.copy();
    if (alliance == Alliance.Red) {
      alliancePath.flip();
    }
    return alliancePath.getStartPose();
  }
}
