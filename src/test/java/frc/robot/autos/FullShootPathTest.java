package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

class FullShootPathTest {
  private static final double EPSILON = 1.0e-6;

  @Test
  void loadsSuppliedBluePathAndConstraints() {
    Path path = new Path("full-shoot");

    assertTrue(path.isValid());
    List<Translation2d> translations = path.getTranslations();
    assertEquals(8, translations.size());
    assertTranslationEquals(new Translation2d(4.45653, 7.64792), translations.get(0));
    assertTranslationEquals(new Translation2d(1.51273, 5.47455), translations.get(7));
    assertEquals(-1.5708, path.getStartPose().getRotation().getRadians(), EPSILON);
    assertEquals(4.65, path.getDefaultGlobalConstraints().getMaxVelocityMetersPerSec(), EPSILON);

    List<Path.RangedConstraint> velocityConstraints = path.getPathConstraints()
        .getMaxVelocityMetersPerSec()
        .orElseThrow();
    assertEquals(6, velocityConstraints.size());
    assertEquals(4.5, velocityConstraints.get(0).value(), EPSILON);
    assertEquals(0, velocityConstraints.get(0).startOrdinal());
    assertEquals(0, velocityConstraints.get(0).endOrdinal());
    assertEquals(0.65, velocityConstraints.get(5).value(), EPSILON);
    assertEquals(7, velocityConstraints.get(5).startOrdinal());
    assertEquals(7, velocityConstraints.get(5).endOrdinal());
  }

  @Test
  void redFlipRotatesEveryTranslationAndHeadingAcrossFieldCenter() {
    Path bluePath = new Path("full-shoot");
    Path redPath = bluePath.copy();
    redPath.flip();

    List<Translation2d> blueTranslations = bluePath.getTranslations();
    List<Translation2d> redTranslations = redPath.getTranslations();
    assertEquals(blueTranslations.size(), redTranslations.size());
    for (int i = 0; i < blueTranslations.size(); i++) {
      assertTranslationEquals(
          new Translation2d(
              FieldConstants.FIELD_LENGTH_METERS - blueTranslations.get(i).getX(),
              FieldConstants.FIELD_WIDTH_METERS - blueTranslations.get(i).getY()),
          redTranslations.get(i));
    }

    List<Path.PathElement> blueElements = bluePath.getPathElements();
    List<Path.PathElement> redElements = redPath.getPathElements();
    for (int i = 0; i < blueElements.size(); i++) {
      Rotation2d blueRotation = rotationOf(blueElements.get(i));
      Rotation2d redRotation = rotationOf(redElements.get(i));
      if (blueRotation != null) {
        assertEquals(
            0.0,
            redRotation.minus(blueRotation.minus(Rotation2d.kPi)).getRadians(),
            EPSILON);
      }
    }
  }

  private static Rotation2d rotationOf(Path.PathElement element) {
    if (element instanceof Path.Waypoint waypoint) {
      return waypoint.rotationTarget().rotation();
    }
    if (element instanceof Path.RotationTarget rotationTarget) {
      return rotationTarget.rotation();
    }
    return null;
  }

  private static void assertTranslationEquals(Translation2d expected, Translation2d actual) {
    assertEquals(expected.getX(), actual.getX(), EPSILON);
    assertEquals(expected.getY(), actual.getY(), EPSILON);
  }
}
