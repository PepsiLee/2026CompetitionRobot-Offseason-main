// package frc.robot.subsystems.vision;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import com.ctre.phoenix6.Utils;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import frc.robot.RobotState;
// import frc.robot.config.DriveConfiguration;
// import frc.robot.config.VisionConfiguration;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.DriveIO;
// import java.util.Optional;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// class VisionTest {
//   private static final Pose2d REFERENCE_POSE =
//       new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(20.0));

//   private Vision vision;
//   private RobotState robotState;
//   private FakeDriveIO driveIO;

//   @BeforeEach
//   void setUp() {
//     robotState = new RobotState();
//     driveIO = new FakeDriveIO();
//     Drive drive =
//         new Drive(
//             robotState,
//             driveIO,
//             new DriveConfiguration(null, null, null, 5.0, 10.0));
//     drive.periodic();
//     vision =
//         new Vision(
//             robotState,
//             drive,
//             new VisionConfiguration("test", new Transform3d(), 6.0, 720.0, 1.0, 0.5),
//             new VisionIO() {});
//   }

//   @Test
//   void acceptsFreshNearbyMeasurement() {
//     VisionMeasurement measurement =
//         new VisionMeasurement(
//             new Pose2d(2.2, 2.1, Rotation2d.kZero),
//             Utils.getCurrentTimeSeconds(),
//             2,
//             3.0);

//     assertEquals(Vision.RejectionReason.ACCEPTED, vision.evaluate(measurement, 10.0));
//   }

//   @Test
//   void forwardsAcceptedMeasurementToDriveEstimatorWithOriginalTimestamp() {
//     double timestamp = Utils.getCurrentTimeSeconds();
//     VisionIO measurementIO =
//         new VisionIO() {
//           @Override
//           public void updateInputs(Inputs inputs) {
//             inputs.heartbeat = 0;
//             inputs.hasTargets = true;
//             inputs.estimatedPose = new Pose2d(2.2, 2.1, Rotation2d.kZero);
//             inputs.timestampSeconds = timestamp;
//             inputs.tagCount = 2;
//             inputs.averageDistanceMeters = 3.0;
//           }
//         };
//     Drive drive =
//         new Drive(
//             robotState,
//             driveIO,
//             new DriveConfiguration(null, null, null, 5.0, 10.0));
//     Vision forwardingVision =
//         new Vision(
//             robotState,
//             drive,
//             new VisionConfiguration("test", new Transform3d(), 6.0, 720.0, 1.0, 0.5),
//             measurementIO);

//     forwardingVision.periodic();

//     assertEquals(timestamp, driveIO.lastVisionTimestampSeconds, 1.0e-9);
//     assertEquals(timestamp, robotState.getLastAcceptedVisionTimestampSeconds(), 1.0e-9);
//   }

//   @Test
//   void rejectsNoTagsOutsideFieldFastRotationAndPoseJump() {
//     double now = Utils.getCurrentTimeSeconds();
//     assertEquals(
//         Vision.RejectionReason.NO_TAGS,
//         vision.evaluate(new VisionMeasurement(REFERENCE_POSE, now, 0, 2.0), 0.0));
//     assertEquals(
//         Vision.RejectionReason.OUTSIDE_FIELD,
//         vision.evaluate(new VisionMeasurement(new Pose2d(-1.0, 2.0, Rotation2d.kZero), now, 1, 2.0), 0.0));
//     assertEquals(
//         Vision.RejectionReason.ROTATING_TOO_FAST,
//         vision.evaluate(new VisionMeasurement(REFERENCE_POSE, now, 1, 2.0), 721.0));
//     assertEquals(
//         Vision.RejectionReason.POSE_JUMP,
//         vision.evaluate(new VisionMeasurement(new Pose2d(3.1, 2.0, Rotation2d.kZero), now, 1, 2.0), 0.0));
//   }

//   private static final class FakeDriveIO implements DriveIO {
//     private double lastVisionTimestampSeconds = Double.NaN;

//     @Override
//     public void updateInputs(DriveIOInputs inputs) {
//       inputs.pose = REFERENCE_POSE;
//       inputs.gyroYaw = REFERENCE_POSE.getRotation();
//       inputs.measuredRobotRelativeSpeeds = new ChassisSpeeds();
//       inputs.timestampSeconds = Utils.getCurrentTimeSeconds();
//     }

//     @Override
//     public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {}

//     @Override
//     public void resetPose(Pose2d pose) {}

//     @Override
//     public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
//       return Optional.of(REFERENCE_POSE);
//     }

//     @Override
//     public void addVisionMeasurement(
//         Pose2d visionPose,
//         double timestampSeconds,
//         edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1>
//             standardDeviations) {
//       lastVisionTimestampSeconds = timestampSeconds;
//     }
//   }
// }
