package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

/** Hardware boundary for Drive. Real hardware and simulation implement this independently. */
public interface DriveIO {
  class DriveIOInputs {
    public Pose2d pose = Pose2d.kZero;
    public Rotation2d gyroYaw = Rotation2d.kZero;
    public ChassisSpeeds measuredRobotRelativeSpeeds = new ChassisSpeeds();
    public SwerveModuleState[] moduleStates = zeroModuleStates();
    public SwerveModuleState[] moduleTargets = zeroModuleStates();
    public SwerveModulePosition[] modulePositions = zeroModulePositions();
    public double timestampSeconds = 0.0;

    private static SwerveModuleState[] zeroModuleStates() {
      return new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
    }

    private static SwerveModulePosition[] zeroModulePositions() {
      return new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
    }
  }

  void updateInputs(DriveIOInputs inputs);

  void runVelocity(ChassisSpeeds robotRelativeSpeeds);

  void resetPose(Pose2d pose);

  default void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {}

  default Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return Optional.empty();
  }

  default void stop() {
    runVelocity(new ChassisSpeeds());
  }
}
