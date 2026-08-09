package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.AutoFactory;
import org.junit.jupiter.api.Test;

class FieldConstantsTest {
  private static final double EPSILON = 1.0e-9;

  @Test
  void leavesBlueLeftCanonicalPoseUnchanged() {
    Pose2d source = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(35.0));

    Pose2d canonical = AutoFactory.transform(source, false, Alliance.Blue);

    assertEquals(source, canonical);
  }

  @Test
  void mirrorsBlueLeftPoseToBlueRight() {
    Pose2d source = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(35.0));

    Pose2d mirrored = AutoFactory.transform(source, true, Alliance.Blue);

    assertEquals(2.0, mirrored.getX(), EPSILON);
    assertEquals(FieldConstants.FIELD_WIDTH_METERS - 1.5, mirrored.getY(), EPSILON);
    assertAngleEquals(-35.0, mirrored.getRotation().getDegrees());
  }

  @Test
  void rotatesBlueLeftPoseToRedLeftEquivalent() {
    Pose2d source = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(35.0));

    Pose2d red = AutoFactory.transform(source, false, Alliance.Red);

    assertEquals(FieldConstants.FIELD_LENGTH_METERS - 2.0, red.getX(), EPSILON);
    assertEquals(FieldConstants.FIELD_WIDTH_METERS - 1.5, red.getY(), EPSILON);
    assertAngleEquals(-145.0, red.getRotation().getDegrees());
  }

  @Test
  void redRightAppliesSideMirrorThenAllianceRotation() {
    Pose2d source = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(35.0));

    Pose2d redRight = AutoFactory.transform(source, true, Alliance.Red);

    assertEquals(FieldConstants.FIELD_LENGTH_METERS - 2.0, redRight.getX(), EPSILON);
    assertEquals(1.5, redRight.getY(), EPSILON);
    assertAngleEquals(145.0, redRight.getRotation().getDegrees());
  }

  private static void assertAngleEquals(double expectedDegrees, double actualDegrees) {
    assertEquals(
        0.0,
        MathUtil.angleModulus(
            Math.toRadians(actualDegrees) - Math.toRadians(expectedDegrees)),
        EPSILON);
  }
}
