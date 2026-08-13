package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

class FarToBallPathTest {
  private static final double EPSILON = 1.0e-6;

  @Test
  void loadsSuppliedBluePathAndOneMeterPerSecondConstraints() {
    Path path = new Path("fartobal");

    assertTrue(path.isValid());
    List<Translation2d> translations = path.getTranslations();
    assertEquals(8, translations.size());
    assertTranslationEquals(new Translation2d(4.44733, 0.4641), translations.get(0));
    assertTranslationEquals(new Translation2d(2.1414, 2.6662), translations.get(7));
    assertEquals(1.58307, path.getStartPose().getRotation().getRadians(), EPSILON);

    List<Path.RangedConstraint> velocityConstraints = path.getPathConstraints()
        .getMaxVelocityMetersPerSec()
        .orElseThrow();
    assertEquals(3, velocityConstraints.size());
    assertConstraint(velocityConstraints.get(0), 1.0, 0, 4);
    assertConstraint(velocityConstraints.get(1), 1.0, 5, 6);
    assertConstraint(velocityConstraints.get(2), 1.0, 7, 7);
  }

  @Test
  void redFlipRotatesStartAndEndAcrossFieldCenter() {
    Path bluePath = new Path("fartobal");
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
  }

  private static void assertConstraint(
      Path.RangedConstraint constraint, double value, int startOrdinal, int endOrdinal) {
    assertEquals(value, constraint.value(), EPSILON);
    assertEquals(startOrdinal, constraint.startOrdinal());
    assertEquals(endOrdinal, constraint.endOrdinal());
  }

  private static void assertTranslationEquals(Translation2d expected, Translation2d actual) {
    assertEquals(expected.getX(), actual.getX(), EPSILON);
    assertEquals(expected.getY(), actual.getY(), EPSILON);
  }
}
