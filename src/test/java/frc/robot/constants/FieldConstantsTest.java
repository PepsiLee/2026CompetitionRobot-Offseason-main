package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

class FieldConstantsTest {
  private static final double EPSILON = 1.0e-9;

  @Test
  void blueAllianceKeepsBlueOriginPoseUnchanged() {
    Pose2d source = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(35.0));

    assertEquals(source, FieldConstants.blueToAlliance(source, Alliance.Blue));
  }

  @Test
  void redAllianceRotatesBlueOriginPoseAcrossField() {
    Pose2d source = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(35.0));

    Pose2d red = FieldConstants.blueToAlliance(source, Alliance.Red);

    assertEquals(FieldConstants.FIELD_LENGTH_METERS - 2.0, red.getX(), EPSILON);
    assertEquals(FieldConstants.FIELD_WIDTH_METERS - 1.5, red.getY(), EPSILON);
    assertEquals(
        0.0,
        MathUtil.angleModulus(
            red.getRotation().getRadians() - Math.toRadians(-145.0)),
        EPSILON);
  }
}
