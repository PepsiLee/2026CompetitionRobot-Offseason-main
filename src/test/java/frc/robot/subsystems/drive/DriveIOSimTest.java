package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveIOSimTest {
  private static final double HALF_WHEELBASE_METERS = 0.28;
  private static final double HALF_TRACK_WIDTH_METERS = 0.28;
  private static final double EPSILON = 1.0e-9;

  private SwerveDriveKinematics kinematics;

  @BeforeEach
  void setUp() {
    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(HALF_WHEELBASE_METERS, HALF_TRACK_WIDTH_METERS),
            new Translation2d(HALF_WHEELBASE_METERS, -HALF_TRACK_WIDTH_METERS),
            new Translation2d(-HALF_WHEELBASE_METERS, HALF_TRACK_WIDTH_METERS),
            new Translation2d(-HALF_WHEELBASE_METERS, -HALF_TRACK_WIDTH_METERS));
  }

  @Test
  void forwardPointsAllModulesForward() {
    var moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));

    for (var state : moduleStates) {
      assertEquals(1.0, state.speedMetersPerSecond, EPSILON);
      assertAngleEquals(0.0, state.angle.getRadians());
    }
  }

  @Test
  void positiveYPointsAllModulesLeft() {
    var moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 1.0, 0.0));

    for (var state : moduleStates) {
      assertEquals(1.0, state.speedMetersPerSecond, EPSILON);
      assertAngleEquals(Math.PI / 2.0, state.angle.getRadians());
    }
  }

  @Test
  void positiveOmegaTurnsCounterClockwiseWithCorrectModuleDirections() {
    var moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));

    assertAngleEquals(3.0 * Math.PI / 4.0, moduleStates[0].angle.getRadians());
    assertAngleEquals(Math.PI / 4.0, moduleStates[1].angle.getRadians());
    assertAngleEquals(-3.0 * Math.PI / 4.0, moduleStates[2].angle.getRadians());
    assertAngleEquals(-Math.PI / 4.0, moduleStates[3].angle.getRadians());
  }

  private static void assertAngleEquals(double expectedRadians, double actualRadians) {
    assertEquals(0.0, MathUtil.angleModulus(actualRadians - expectedRadians), EPSILON);
  }
}
