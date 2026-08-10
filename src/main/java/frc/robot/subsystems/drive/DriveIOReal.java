package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.config.DriveConfiguration;
import java.util.Optional;

/// CTRE Powered swerve drivertrain.
public class DriveIOReal extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DriveIO {
  private final SwerveRequest.ApplyRobotSpeeds velocityRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withDesaturateWheelSpeeds(true);

  public DriveIOReal(DriveConfiguration configuration) {
    this(configuration.drivetrainConstants(), configuration.moduleConstants());
  }

  @SafeVarargs
  private DriveIOReal(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, moduleConstants);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    var state = getState();
    inputs.pose = state.Pose;
    inputs.gyroYaw = state.RawHeading;
    inputs.measuredRobotRelativeSpeeds = state.Speeds;
    inputs.moduleStates = copyStates(state.ModuleStates);
    inputs.moduleTargets = copyStates(state.ModuleTargets);
    inputs.modulePositions = copyPositions(state.ModulePositions);
    inputs.timestampSeconds = state.Timestamp;
  }

  @Override
  public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
    setControl(velocityRequest.withSpeeds(robotRelativeSpeeds));
  }

  @Override
  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {
    super.addVisionMeasurement(visionPose, timestampSeconds, standardDeviations);
  }

  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
    return super.samplePoseAt(timestampSeconds);
  }

  private static SwerveModuleState[] copyStates(SwerveModuleState[] source) {
    if (source == null) {
      return new SwerveModuleState[0];
    }
    SwerveModuleState[] copy = new SwerveModuleState[source.length];
    for (int i = 0; i < source.length; i++) {
      copy[i] = new SwerveModuleState(source[i].speedMetersPerSecond, source[i].angle);
    }
    return copy;
  }

  private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] source) {
    if (source == null) {
      return new SwerveModulePosition[0];
    }
    SwerveModulePosition[] copy = new SwerveModulePosition[source.length];
    for (int i = 0; i < source.length; i++) {
      copy[i] = new SwerveModulePosition(source[i].distanceMeters, source[i].angle);
    }
    return copy;
  }
}
