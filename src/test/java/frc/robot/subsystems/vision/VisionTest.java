package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotState;
import frc.robot.config.DriveConfiguration;
import frc.robot.config.VisionConfiguration;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class VisionTest {
  private static final Pose2d REFERENCE_POSE =
      new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(20.0));

  private Vision vision;
  private RobotState robotState;
  private FakeDriveIO driveIO;

  @BeforeEach
  void setUp() {
    robotState = new RobotState();
    driveIO = new FakeDriveIO();
    Drive drive =
        new Drive(
            robotState,
            driveIO,
            new DriveConfiguration(null, null, null, 5.0, 10.0));
    drive.periodic();
    vision =
        new Vision(
            robotState,
            drive,
            new VisionConfiguration("test", new Transform3d(), 6.0, 720.0, 0.5),
            new VisionIO() {});
  }

  @Test
  void acceptsFreshNearbyMeasurement() {
    VisionMeasurement measurement =
        new VisionMeasurement(
            new Pose2d(2.2, 2.1, Rotation2d.kZero),
            Utils.getCurrentTimeSeconds(),
            2,
            3.0);

    assertEquals(Vision.RejectionReason.ACCEPTED, vision.evaluate(measurement, 10.0));
  }

  @Test
  void forwardsLargePoseDifferenceToDriveEstimatorWithOriginalTimestamp() {
    double timestamp = Utils.getCurrentTimeSeconds();
    VisionIO measurementIO =
        new VisionIO() {
          @Override
          public void updateInputs(Inputs inputs) {
            inputs.heartbeat = 0;
            inputs.hasTargets = true;
            inputs.estimatedPose = new Pose2d(4.2, 2.1, Rotation2d.kZero);
            inputs.timestampSeconds = timestamp;
            inputs.tagCount = 2;
            inputs.averageDistanceMeters = 3.0;
          }
        };
    Drive drive =
        new Drive(
            robotState,
            driveIO,
            new DriveConfiguration(null, null, null, 5.0, 10.0));
    Vision forwardingVision =
        new Vision(
            robotState,
            drive,
            new VisionConfiguration("test", new Transform3d(), 6.0, 720.0, 0.5),
            measurementIO);

    forwardingVision.periodic();

    assertEquals(timestamp, driveIO.lastVisionTimestampSeconds, 1.0e-9);
    assertEquals(timestamp, robotState.getLastAcceptedVisionTimestampSeconds(), 1.0e-9);
  }

  @Test
  void rejectsInvalidMeasurementsButAcceptsLargePoseDifference() {
    double now = Utils.getCurrentTimeSeconds();
    assertEquals(
        Vision.RejectionReason.NO_TAGS,
        vision.evaluate(new VisionMeasurement(REFERENCE_POSE, now, 0, 2.0), 0.0));
    assertEquals(
        Vision.RejectionReason.OUTSIDE_FIELD,
        vision.evaluate(new VisionMeasurement(new Pose2d(-1.0, 2.0, Rotation2d.kZero), now, 1, 2.0), 0.0));
    assertEquals(
        Vision.RejectionReason.ROTATING_TOO_FAST,
        vision.evaluate(new VisionMeasurement(REFERENCE_POSE, now, 1, 2.0), 721.0));
    assertEquals(
        Vision.RejectionReason.ACCEPTED,
        vision.evaluate(new VisionMeasurement(new Pose2d(3.1, 2.0, Rotation2d.kZero), now, 1, 2.0), 0.0));
  }

  @Test
  void forwardsMultiTagMt1AsHeadingOnlyMeasurement() {
    double timestamp = Utils.getCurrentTimeSeconds();
    Pose2d mt1Pose = new Pose2d(2.1, 2.2, Rotation2d.fromDegrees(180.0));
    VisionIO measurementIO = new VisionIO() {
      @Override
      public void updateInputs(Inputs inputs) {
        inputs.mt1EstimatedPose = mt1Pose;
        inputs.mt1TimestampSeconds = timestamp;
        inputs.mt1TagCount = 2;
        inputs.mt1AverageDistanceMeters = 2.5;
        inputs.mt1MaximumAmbiguity = 0.25;
      }
    };
    Drive drive = new Drive(
        robotState,
        driveIO,
        new DriveConfiguration(null, null, null, 5.0, 10.0));
    Vision headingVision = new Vision(
        robotState,
        drive,
        new VisionConfiguration("test", new Transform3d(), 6.0, 720.0, 0.5),
        measurementIO);

    headingVision.periodic();

    assertEquals(Vision.HeadingRejectionReason.ACCEPTED,
        headingVision.getLastHeadingRejectionReason());
    assertEquals(1, driveIO.visionMeasurementCount);
    assertEquals(mt1Pose, driveIO.lastVisionPose);
    assertEquals(timestamp, driveIO.lastVisionTimestampSeconds, 1.0e-9);
    assertEquals(1.0e6, driveIO.lastVisionStandardDeviations.get(0, 0), 1.0e-9);
    assertEquals(1.0e6, driveIO.lastVisionStandardDeviations.get(1, 0), 1.0e-9);
    assertEquals(0.10, driveIO.lastVisionStandardDeviations.get(2, 0), 1.0e-9);
  }

  @Test
  void acceptsUnambiguousNearbySingleTagButRejectsAmbiguousSingleTagHeading() {
    double now = Utils.getCurrentTimeSeconds();
    VisionMeasurement singleTag =
        new VisionMeasurement(REFERENCE_POSE, now, 1, 2.0);

    assertEquals(
        Vision.HeadingRejectionReason.ACCEPTED,
        vision.evaluateMt1Heading(singleTag, 0.10, 0.0));
    assertEquals(
        Vision.HeadingRejectionReason.SINGLE_TAG_AMBIGUOUS,
        vision.evaluateMt1Heading(singleTag, 0.20, 0.0));
    assertEquals(
        Vision.HeadingRejectionReason.SINGLE_TAG_AMBIGUOUS,
        vision.evaluateMt1Heading(
            new VisionMeasurement(REFERENCE_POSE, now, 1, 3.1), 0.10, 0.0));
  }

  private static final class FakeDriveIO implements DriveIO {
    private double lastVisionTimestampSeconds = Double.NaN;
    private Pose2d lastVisionPose = Pose2d.kZero;
    private Matrix<N3, N1> lastVisionStandardDeviations;
    private int visionMeasurementCount;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
      inputs.pose = REFERENCE_POSE;
      inputs.gyroYaw = REFERENCE_POSE.getRotation();
      inputs.measuredRobotRelativeSpeeds = new ChassisSpeeds();
      inputs.timestampSeconds = Utils.getCurrentTimeSeconds();
    }

    @Override
    public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {}

    @Override
    public void resetPose(Pose2d pose) {}

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
      return Optional.of(REFERENCE_POSE);
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionPose,
        double timestampSeconds,
        Matrix<N3, N1> standardDeviations) {
      lastVisionPose = visionPose;
      lastVisionTimestampSeconds = timestampSeconds;
      lastVisionStandardDeviations = standardDeviations;
      visionMeasurementCount++;
    }
  }
}
