package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** 2026 field geometry and alliance hub locations. */
public final class FieldConstants {
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);
  public static final double FIELD_LENGTH_METERS = FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_METERS = FIELD_LAYOUT.getFieldWidth();

  //In Meter
  public static final Translation2d BLUE_HUB = new Translation2d(4.626, 4.035);
  public static final Translation2d RED_HUB = new Translation2d(FIELD_LENGTH_METERS - BLUE_HUB.getX(),
      FIELD_WIDTH_METERS - BLUE_HUB.getY());

  private FieldConstants() {
  }

  public static Translation2d hubForAlliance(Alliance alliance) {
    return alliance == Alliance.Red ? RED_HUB : BLUE_HUB;
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
}
