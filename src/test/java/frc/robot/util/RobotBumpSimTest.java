package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotBumpSimTest {
  private static final double EPSILON = 1.0e-9;
  private RobotBumpSim bumpSim;

  @BeforeEach
  void setUp() {
    bumpSim =
        new RobotBumpSim(
            new Translation2d[] {
              new Translation2d(0.28, 0.28),
              new Translation2d(0.28, -0.28),
              new Translation2d(-0.28, 0.28),
              new Translation2d(-0.28, -0.28)
            });
  }

  @Test
  void flatFieldKeepsRobotLevel() {
    var pose3d =
        bumpSim.update(
            new Pose2d(2.0, 2.0, Rotation2d.kZero), new ChassisSpeeds(), 5);

    assertFalse(bumpSim.isOnRamp());
    assertEquals(0.0, pose3d.getZ(), EPSILON);
    assertEquals(0.0, pose3d.getRotation().getX(), EPSILON);
    assertEquals(0.0, pose3d.getRotation().getY(), EPSILON);
  }

  @Test
  void frontModulesContactingBumpProduceHeightAndPitch() {
    // Front modules are at X=3.98 m, just inside the blue BUMP ascending face.
    var pose3d =
        bumpSim.update(
            new Pose2d(3.70, 2.0, Rotation2d.kZero),
            new ChassisSpeeds(2.0, 0.0, 0.0),
            5);

    assertTrue(bumpSim.isOnRamp());
    assertTrue(pose3d.getZ() > 0.0);
    assertNotEquals(0.0, pose3d.getRotation().getY(), EPSILON);
  }
}
