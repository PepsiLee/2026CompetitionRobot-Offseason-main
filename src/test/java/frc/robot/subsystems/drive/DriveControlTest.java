package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.config.DriveConfiguration;
import org.junit.jupiter.api.Test;

class DriveControlTest {
  private static final double EPSILON = 1.0e-9;

  @Test
  void stationaryAimNeverCommandsTranslation() {
    FakeDriveIO io = new FakeDriveIO();
    io.pose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
    Drive drive = createDrive(io);
    drive.periodic();

    drive.requestAimStationary(Rotation2d.kCCW_90deg);
    drive.periodic();

    assertEquals(0.0, io.commanded.vxMetersPerSecond, EPSILON);
    assertEquals(0.0, io.commanded.vyMetersPerSecond, EPSILON);
    assertTrue(io.commanded.omegaRadiansPerSecond > 0.0);
  }

  @Test
  void driveToPoseCommandsMotionTowardTarget() {
    FakeDriveIO io = new FakeDriveIO();
    Drive drive = createDrive(io);
    drive.periodic();

    drive.requestDriveToPose(new Pose2d(1.0, 0.0, Rotation2d.kZero), 2.0, 0.10, 0.05);
    drive.periodic();

    assertTrue(io.commanded.vxMetersPerSecond > 0.0);
    assertEquals(0.0, io.commanded.vyMetersPerSecond, EPSILON);
  }

  @Test
  void fieldWidgetTracksEstimatorPose() {
    FakeDriveIO io = new FakeDriveIO();
    io.pose = new Pose2d(4.2, 3.1, Rotation2d.fromDegrees(67.0));
    Drive drive = createDrive(io);

    drive.periodic();

    assertEquals(io.pose, drive.getField().getRobotPose());
  }

  private static Drive createDrive(FakeDriveIO io) {
    return new Drive(
        new RobotState(),
        io,
        new DriveConfiguration(null, null, null, 5.0, 10.0));
  }

  private static final class FakeDriveIO implements DriveIO {
    private Pose2d pose = Pose2d.kZero;
    private ChassisSpeeds commanded = new ChassisSpeeds();

    @Override
    public void updateInputs(DriveIOInputs inputs) {
      inputs.pose = pose;
      inputs.gyroYaw = pose.getRotation();
      inputs.measuredRobotRelativeSpeeds = new ChassisSpeeds();
      inputs.timestampSeconds += 0.02;
    }

    @Override
    public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
      commanded = robotRelativeSpeeds;
    }

    @Override
    public void resetPose(Pose2d pose) {
      this.pose = pose;
    }
  }
}
