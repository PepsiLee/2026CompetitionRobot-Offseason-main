package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterCalculator;

import org.littletonrobotics.junction.Logger;

/**
 * Coordinates the fixed intake, gravity-fed shooter, and stationary chassis
 * aiming.
 */
public final class SuperStructure extends SubsystemBase {
  public enum SystemState {
    IDLE,
    INTAKING,
    AIMING,
    SHOOTING,
    DIRECT_SHOOTING,
    FAULT,
    STOPPED
  }

  private static final double AIM_ENTER_TOLERANCE_RADIANS = Units.degreesToRadians(3.0);
  private static final double AIM_EXIT_TOLERANCE_RADIANS = Units.degreesToRadians(6.0);
  private static final double AIM_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Units.degreesToRadians(15.0);
  private static final double AIM_STABLE_SECONDS = 0.10;
  private static final double SPEED_STABLE_SECONDS = 0.20;
  // subsystem references
  private final Drive drive;
  private final RobotState robotState;
  private final Intake intake;
  private final Shooter shooter;
  private final Feeder feeder;

  private final Debouncer aimReadyDebouncer = new Debouncer(AIM_STABLE_SECONDS, Debouncer.DebounceType.kRising);
  private final Debouncer speedReadyDebouncer = new Debouncer(SPEED_STABLE_SECONDS, Debouncer.DebounceType.kRising);
  // Flags for requested actions
  private boolean intakeRequested;
  private boolean shootRequested;
  private boolean directShootRequested;
  private boolean stopped;
  // Current state of the superstructure
  private SystemState systemState = SystemState.IDLE;
  private Rotation2d desiredAimHeading = Rotation2d.kZero;
  private Rotation2d lockedShotHeading = Rotation2d.kZero;
  private double lockedShotDistanceMeters;
  private String faultReason = "";

  public SuperStructure(Drive drive, RobotState robotState, Intake intake, Shooter shooter, Feeder feeder) {
    this.drive = drive;
    this.robotState = robotState;
    this.intake = intake;
    this.shooter = shooter;
    this.feeder = feeder;
  }

  @Override
  public void periodic() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Translation2d hub = FieldConstants.hubForAlliance(alliance);
    desiredAimHeading = calculateRearShooterHeading(robotState.getPose(), hub);

    systemState = determineState();
    applyState();

    Logger.recordOutput("SuperStructure/SystemState", systemState);
    Logger.recordOutput("SuperStructure/IntakeRequested", intakeRequested);
    Logger.recordOutput("SuperStructure/ShootRequested", shootRequested);
    Logger.recordOutput("SuperStructure/DirectShootRequested", directShootRequested);
    Logger.recordOutput("SuperStructure/DesiredAimHeading", desiredAimHeading);
    Logger.recordOutput("SuperStructure/LockedShotHeading", lockedShotHeading);
    Logger.recordOutput("SuperStructure/LockedShotDistanceMeters", lockedShotDistanceMeters);
    Logger.recordOutput("SuperStructure/FaultReason", faultReason);
  }

  private SystemState determineState() {
    // Check for stop
    if (stopped) {
      return SystemState.STOPPED;
    }
    // Direct Shoot
    if (directShootRequested) {
      faultReason = "";
      aimReadyDebouncer.calculate(false);
      return SystemState.DIRECT_SHOOTING;
    }
    // Shoot request
    if (!shootRequested) {
      faultReason = "";
      aimReadyDebouncer.calculate(false);
      return intakeRequested ? SystemState.INTAKING : SystemState.IDLE;
    }

    // Check if we are already shooting and if we are still aimed at the target
    if (systemState == SystemState.SHOOTING) {
      if (Math.abs(drive.getHeadingErrorRadians(lockedShotHeading)) > AIM_EXIT_TOLERANCE_RADIANS) {
        aimReadyDebouncer.calculate(false);
        return SystemState.AIMING;
      }
      return SystemState.SHOOTING;
    }

    drive.requestAimStationary(desiredAimHeading);
    boolean aimReady = drive.isAtHeading(desiredAimHeading, AIM_ENTER_TOLERANCE_RADIANS)
        && Math.abs(drive.getAngularVelocityRadiansPerSecond()) <= AIM_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    if (!aimReadyDebouncer.calculate(aimReady)) {
      return SystemState.AIMING;
    }

    lockedShotHeading = desiredAimHeading;
    lockedShotDistanceMeters = robotState.getPose().getTranslation().getDistance(
        FieldConstants.hubForAlliance(
            DriverStation.getAlliance().orElse(Alliance.Blue)));
    return SystemState.SHOOTING;
  }

  private void applyState() {

    if (systemState == SystemState.FAULT || systemState == SystemState.STOPPED) {
      intake.setWantedState(Intake.WantedState.STOPPED);
    } else {
      if (intake.getWantedState() != Intake.WantedState.DEPLOY) {
        intake.setWantedState(
            intakeRequested ? Intake.WantedState.INTAKE : Intake.WantedState.ONLY_ROLLER);
      }
    }

    switch (systemState) {
      case IDLE -> {
        shooter.setWantedState(Shooter.WantedState.OFF);
        feeder.setWantedState(Feeder.WantedState.OFF);
        drive.releaseAim();
      }
      case INTAKING -> {
        shooter.setWantedState(Shooter.WantedState.OFF);
        feeder.setWantedState(Feeder.WantedState.OFF);
        drive.releaseAim();
      }
      case AIMING -> {
        shooter.setRPM(ShooterCalculator.calculateRPM(lockedShotDistanceMeters));
        shooter.setWantedState(Shooter.WantedState.SHOOTING);
        feeder.setWantedState(Feeder.WantedState.OFF);
        drive.requestAimStationary(desiredAimHeading);
      }
      case SHOOTING -> {
        shooter.setRPM(ShooterCalculator.calculateRPM(lockedShotDistanceMeters));
        shooter.setWantedState(Shooter.WantedState.SHOOTING);
        if (speedReadyDebouncer.calculate(shooter.isReady())) {
          feeder.setWantedState(Feeder.WantedState.FEED_SHOOTER);
        } else {
          feeder.setWantedState(Feeder.WantedState.OFF);
        }
        drive.requestAimStationary(lockedShotHeading);
      }
      case DIRECT_SHOOTING -> {
        shooter.setRPM(ShooterCalculator.calculateRPM(lockedShotDistanceMeters));
        shooter.setWantedState(Shooter.WantedState.SHOOTING);
        if (speedReadyDebouncer.calculate(shooter.isReady())) {
          feeder.setWantedState(Feeder.WantedState.FEED_SHOOTER);
        } else {
          feeder.setWantedState(Feeder.WantedState.OFF);
        }
        drive.releaseAim();
      }
      case FAULT, STOPPED -> {
        shooter.setWantedState(Shooter.WantedState.OFF);
        feeder.setWantedState(Feeder.WantedState.OFF);
        drive.stop();
      }
    }
  }

  public static Rotation2d calculateRearShooterHeading(Pose2d robotPose, Translation2d target) {
    return target
        .minus(robotPose.getTranslation())
        .getAngle()
        .plus(Rotation2d.k180deg);
  }

  public void setIntakeRequested(boolean requested) {
    intakeRequested = requested;
  }

  public void setShootRequested(boolean requested) {
    shootRequested = requested;
  }

  public void setDirectShootRequested(boolean requested) {
    directShootRequested = requested;
  }

  public boolean isShooting() {
    return systemState == SystemState.SHOOTING
        || systemState == SystemState.DIRECT_SHOOTING;
  }

  public boolean isFaulted() {
    return systemState == SystemState.FAULT || systemState == SystemState.STOPPED;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  public void stopAll() {
    intakeRequested = false;
    shootRequested = false;
    directShootRequested = false;
    stopped = true;
    systemState = SystemState.STOPPED;
    intake.setWantedState(Intake.WantedState.STOPPED);
    shooter.setWantedState(Shooter.WantedState.OFF);
    feeder.setWantedState(Feeder.WantedState.OFF);
    drive.stop();
  }

  public void clearStopped() {
    stopped = false;
    faultReason = "";
    systemState = intakeRequested ? SystemState.INTAKING : SystemState.IDLE;
    intake.setWantedState(
        intakeRequested ? Intake.WantedState.INTAKE : Intake.WantedState.STOPPED);
    shooter.setWantedState(Shooter.WantedState.OFF);
    feeder.setWantedState(Feeder.WantedState.OFF);
    drive.releaseAim();
  }
}
