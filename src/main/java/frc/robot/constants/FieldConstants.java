package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** 2026 field geometry, hubs, and the Shoot Only starting pose. */
public final class FieldConstants {
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);
  public static final double FIELD_LENGTH_METERS = FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = FIELD_LAYOUT.getFieldWidth();

  public static final Translation2d BLUE_TAG_AIM_TARGET = tagMidpoint(25, 26);
  public static final Translation2d RED_TAG_AIM_TARGET = tagMidpoint(9, 10);

  //In Meter
  public static final Translation2d BLUE_HUB = new Translation2d(4.626, 4.035);
  public static final Translation2d RED_HUB = new Translation2d(FIELD_LENGTH_METERS - BLUE_HUB.getX(),
      FIELD_WIDTH_METERS - BLUE_HUB.getY());

  // Must be field-validated before use.
  public static final Pose2d BLUE_LEFT_START = new Pose2d(4.50, 7.35, Rotation2d.fromDegrees(90.0));

  private FieldConstants() {
  }

  public static Translation2d hubForAlliance(Alliance alliance) {
    return alliance == Alliance.Red ? RED_HUB : BLUE_HUB;
  }

  public static Translation2d tagAimTargetForAlliance(Alliance alliance) {
    return alliance == Alliance.Red ? RED_TAG_AIM_TARGET : BLUE_TAG_AIM_TARGET;
  }

  public static Pose2d blueToAlliance(Pose2d pose, Alliance alliance) {
    if (alliance != Alliance.Red) {
      return pose;
    }
    return new Pose2d(
        FIELD_LENGTH_METERS - pose.getX(),
        FIELD_WIDTH_METERS - pose.getY(),
        pose.getRotation().plus(Rotation2d.k180deg));
  }

  private static Translation2d tagMidpoint(int firstId, int secondId) {
    Translation2d first = FIELD_LAYOUT
        .getTagPose(firstId)
        .orElseThrow(() -> new IllegalStateException("Missing AprilTag " + firstId))
        .getTranslation()
        .toTranslation2d();
    Translation2d second = FIELD_LAYOUT
        .getTagPose(secondId)
        .orElseThrow(() -> new IllegalStateException("Missing AprilTag " + secondId))
        .getTranslation()
        .toTranslation2d();
    return first.plus(second).div(2.0);
  }
}
