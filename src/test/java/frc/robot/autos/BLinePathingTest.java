package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.config.DriveConfiguration;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class BLinePathingTest {
  private static final double EPSILON = 1.0e-6;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void loadsAllEightWaypointsFromDeploy() {
    BLinePathing pathing = createPathing();
    Path path = pathing.loadIntakeShootPath();

    assertTrue(path.isValid());
    assertEquals(8, path.getPathElements().size());
    assertEquals(3.28868, path.getTranslations().get(0).getX(), EPSILON);
    assertEquals(7.26792, path.getTranslations().get(0).getY(), EPSILON);
    assertEquals(1.34532, path.getTranslations().get(7).getX(), EPSILON);
    assertEquals(6.58422, path.getTranslations().get(7).getY(), EPSILON);
  }

  @Test
  void redStartPoseUsesRotationalFieldFlip() {
    BLinePathing pathing = createPathing();
    Path path = pathing.loadIntakeShootPath();
    Pose2d blueStart = pathing.getStartPose(path, Alliance.Blue);
    Pose2d redStart = pathing.getStartPose(path, Alliance.Red);

    assertEquals(3.28868, blueStart.getX(), EPSILON);
    assertEquals(7.26792, blueStart.getY(), EPSILON);
    assertEquals(-0.01585, blueStart.getRotation().getRadians(), EPSILON);
    assertEquals(FieldConstants.FIELD_LENGTH_METERS - blueStart.getX(), redStart.getX(), EPSILON);
    assertEquals(FieldConstants.FIELD_WIDTH_METERS - blueStart.getY(), redStart.getY(), EPSILON);
    assertEquals(
        0.0,
        MathUtil.angleModulus(
            redStart.getRotation().getRadians()
                - blueStart.getRotation().plus(Rotation2d.k180deg).getRadians()),
        EPSILON);
  }

  private static BLinePathing createPathing() {
    Drive drive =
        new Drive(
            new RobotState(),
            new FakeDriveIO(),
            new DriveConfiguration(null, null, null, 5.0, 10.0));
    return new BLinePathing(drive);
  }

  private static final class FakeDriveIO implements DriveIO {
    @Override
    public void updateInputs(DriveIOInputs inputs) {
      inputs.measuredRobotRelativeSpeeds = new ChassisSpeeds();
    }

    @Override
    public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {}

    @Override
    public void resetPose(Pose2d pose) {}
  }
}
