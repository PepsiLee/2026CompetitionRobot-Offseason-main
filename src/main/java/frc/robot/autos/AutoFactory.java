package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Builds driver-selectable autonomous routines. */
public final class AutoFactory {
  private static final String SHOOT_SECONDS_KEY = "Auto/ShootSeconds";
  private static final String SHOOT_PATH_NAME = "shoot";
  private static final String FULL_SHOOT_PATH_NAME = "full-shoot";
  private static final String FULL_SHOOT_LOG_PREFIX = "Auto/BLine/FullShoot";
  private static final double DEFAULT_SHOOT_SECONDS = 3.0;
  private static final double AIM_TIMEOUT_SECONDS = 3.0;
  private static final double FULL_SHOOT_AIM_TIMEOUT_SECONDS = 10.0;

  private final Drive drive;
  private final SuperStructure superStructure;
  private final DoubleSupplier shootSecondsSupplier;
  private final Path shootPath;
  private final String shootPathLoadError;
  private final Path fullShootPath;
  private final String fullShootPathLoadError;

  public AutoFactory(Drive drive, SuperStructure superStructure) {
    this(
        drive,
        superStructure,
        () -> SmartDashboard.getNumber(SHOOT_SECONDS_KEY, DEFAULT_SHOOT_SECONDS),
        SHOOT_PATH_NAME,
        FULL_SHOOT_PATH_NAME);
  }

  AutoFactory(
      Drive drive,
      SuperStructure superStructure,
      DoubleSupplier shootSecondsSupplier) {
    this(
        drive,
        superStructure,
        shootSecondsSupplier,
        SHOOT_PATH_NAME,
        FULL_SHOOT_PATH_NAME);
  }

  AutoFactory(
      Drive drive,
      SuperStructure superStructure,
      DoubleSupplier shootSecondsSupplier,
      String shootPathName,
      String fullShootPathName) {
    this.drive = drive;
    this.superStructure = superStructure;
    this.shootSecondsSupplier = shootSecondsSupplier;
    SmartDashboard.putNumber(SHOOT_SECONDS_KEY, DEFAULT_SHOOT_SECONDS);

    // Keep BLine's transforms on the exact same 2026 field dimensions used by WPILib.
    FlippingUtil.fieldSizeX = FieldConstants.FIELD_LENGTH_METERS;
    FlippingUtil.fieldSizeY = FieldConstants.FIELD_WIDTH_METERS;
    configureBLineLogging();

    LoadedPath loadedShootPath = loadPath(shootPathName);
    shootPath = loadedShootPath.path();
    shootPathLoadError = loadedShootPath.error();
    Logger.recordOutput("Auto/BLine/PathLoaded", shootPath != null);
    Logger.recordOutput("Auto/BLine/PathLoadError", shootPathLoadError);
    if (shootPath != null) {
      Logger.recordOutput(
          "Auto/BLine/BluePathPoints",
          shootPath.getTranslations().toArray(Translation2d[]::new));
    }

    LoadedPath loadedFullShootPath = loadPath(fullShootPathName);
    fullShootPath = loadedFullShootPath.path();
    fullShootPathLoadError = loadedFullShootPath.error();
    Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/PathLoaded", fullShootPath != null);
    Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/PathLoadError", fullShootPathLoadError);
    Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", false);
    Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", false);
    if (fullShootPath != null) {
      Logger.recordOutput(
          FULL_SHOOT_LOG_PREFIX + "/BluePathPoints",
          fullShootPath.getTranslations().toArray(Translation2d[]::new));
    }
  }

  private static LoadedPath loadPath(String pathName) {
    try {
      Path path = new Path(pathName);
      if (!path.isValid()) {
        return new LoadedPath(null, "BLine path '" + pathName + "' is invalid");
      }
      return new LoadedPath(path, "");
    } catch (RuntimeException exception) {
      return new LoadedPath(
          null,
          "Could not load BLine path '" + pathName + "': " + exception.getMessage());
    }
  }

  private record LoadedPath(Path path, String error) {}

  public AutoRoutine create(AutoMode mode, Alliance alliance) {
    return switch (mode) {
      case DO_NOTHING -> createDoNothing();
      case SHOOT_ONLY -> createShootOnly(alliance);
      case BLINE_SHOOT -> createBLineShoot(alliance);
      case BLINE_FULL_SHOOT -> createFullBLineShoot(alliance);
    };
  }

  private AutoRoutine createDoNothing() {
    Command routine =
        Commands.runOnce(
                () -> {
                  superStructure.stopAll();
                  drive.stop();
                },
                drive,
                superStructure)
            .withName("Do Nothing");
    return new AutoRoutine(Pose2d.kZero, routine);
  }

  private AutoRoutine createShootOnly(Alliance alliance) {
    Pose2d startingPose =
        FieldConstants.blueToAlliance(FieldConstants.BLUE_LEFT_START, alliance);
    Command routine =
        Commands.sequence(
                resetForAuto(startingPose),
                shootForConfiguredDuration("Preload"),
                stopAll())
            .finallyDo(
                interrupted -> {
                  superStructure.stopAll();
                  drive.stop();
                  Logger.recordOutput("Auto/Interrupted", interrupted);
                })
            .withName(AutoMode.SHOOT_ONLY.name());
    return new AutoRoutine(startingPose, routine);
  }

  private AutoRoutine createBLineShoot(Alliance alliance) {
    if (shootPath == null) {
      Command safeFailure = Commands.runOnce(
              () -> {
                DriverStation.reportError(shootPathLoadError, false);
                superStructure.stopAll();
                drive.stop();
              },
              drive,
              superStructure)
          .withName("BLine Shoot - Path Error");
      return new AutoRoutine(Pose2d.kZero, safeFailure);
    }

    boolean shouldFlip = alliance == Alliance.Red;
    Path transformedPath = shootPath.copy();
    if (shouldFlip) {
      transformedPath.flip();
    }
    Pose2d startingPose = transformedPath.getStartPose();

    Command followPath = new FollowPath.Builder(
            drive,
            drive::getPose,
            drive::getMeasuredRobotRelativeSpeeds,
            drive::acceptPathSpeeds,
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
        .withShouldFlip(() -> shouldFlip)
        .withShouldMirror(() -> false)
        .withPoseReset(drive::resetPose)
        .build(shootPath);

    Command routine = Commands.sequence(
            resetForAuto(startingPose),
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Auto/BLine/Alliance", alliance.name());
                  Logger.recordOutput("Auto/BLine/ShouldFlip", shouldFlip);
                  Logger.recordOutput("Auto/BLine/ShouldMirror", false);
                  Logger.recordOutput("Auto/BLine/ReachedFinalPoint", false);
                  Logger.recordOutput("Auto/BLine/StartingPose", startingPose);
                  Logger.recordOutput(
                      "Auto/BLine/ActivePathPoints",
                      transformedPath.getTranslations().toArray(Translation2d[]::new));
                }),
            followPath,
            Commands.runOnce(
                () -> {
                  drive.acceptPathSpeeds(new ChassisSpeeds());
                  drive.stop();
                  Logger.recordOutput("Auto/BLine/ReachedFinalPoint", true);
                },
                drive),
            shootForConfiguredDuration("BLine Final Point"),
            stopAll())
        .finallyDo(
            interrupted -> {
              clearShootRequests();
              superStructure.stopAll();
              drive.stop();
              Logger.recordOutput("Auto/Interrupted", interrupted);
              Logger.recordOutput("Auto/BLine/ReachedFinalPoint", false);
            })
        .withName(AutoMode.BLINE_SHOOT.name());
    return new AutoRoutine(startingPose, routine);
  }

  private AutoRoutine createFullBLineShoot(Alliance alliance) {
    if (fullShootPath == null) {
      Command safeFailure = Commands.runOnce(
              () -> {
                DriverStation.reportError(fullShootPathLoadError, false);
                superStructure.setIntakeRequested(false);
                superStructure.stopAll();
                drive.stop();
                Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", false);
                Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", false);
              },
              drive,
              superStructure)
          .withName("BLine Full Shoot - Path Error");
      return new AutoRoutine(Pose2d.kZero, safeFailure);
    }

    boolean shouldFlip = alliance == Alliance.Red;
    Path transformedPath = fullShootPath.copy();
    if (shouldFlip) {
      transformedPath.flip();
    }
    Pose2d startingPose = transformedPath.getStartPose();

    // FollowPath mutates the path when alliance flipping, so every routine gets a fresh copy.
    Path followerPath = fullShootPath.copy();
    Command followPath = new FollowPath.Builder(
            drive,
            drive::getPose,
            drive::getMeasuredRobotRelativeSpeeds,
            drive::acceptPathSpeeds,
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
        .withShouldFlip(() -> shouldFlip)
        .withShouldMirror(() -> false)
        .withPoseReset(drive::resetPose)
        .build(followerPath);

    Command routine = Commands.sequence(
            resetForAuto(startingPose),
            Commands.runOnce(
                () -> {
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/Alliance", alliance.name());
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ShouldFlip", shouldFlip);
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ShouldMirror", false);
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", false);
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/StartingPose", startingPose);
                  Logger.recordOutput(
                      FULL_SHOOT_LOG_PREFIX + "/ActivePathPoints",
                      transformedPath.getTranslations().toArray(Translation2d[]::new));
                  superStructure.setIntakeRequested(true);
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", true);
                },
                superStructure),
            followPath,
            Commands.runOnce(
                () -> {
                  drive.acceptPathSpeeds(new ChassisSpeeds());
                  drive.stop();
                  superStructure.setIntakeRequested(false);
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", false);
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", true);
                },
                drive,
                superStructure),
            shootForConfiguredDuration(
                "BLine Full Final Point",
                FULL_SHOOT_AIM_TIMEOUT_SECONDS,
                FULL_SHOOT_LOG_PREFIX),
            stopAll())
        .finallyDo(
            interrupted -> {
              superStructure.setIntakeRequested(false);
              clearShootRequests(FULL_SHOOT_LOG_PREFIX);
              superStructure.stopAll();
              drive.stop();
              Logger.recordOutput("Auto/Interrupted", interrupted);
              Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", false);
              Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", false);
            })
        .withName(AutoMode.BLINE_FULL_SHOOT.name());
    return new AutoRoutine(startingPose, routine);
  }

  private Command resetForAuto(Pose2d startingPose) {
    return Commands.runOnce(
        () -> {
          superStructure.clearStopped();
          superStructure.setIntakeRequested(false);
          clearShootRequests();
          drive.resetPose(startingPose);
          drive.stop();
        },
        drive,
        superStructure);
  }

  private Command stopAll() {
    return Commands.runOnce(
        () -> {
          superStructure.stopAll();
          drive.stop();
        },
        drive,
        superStructure);
  }

  private Command shootForConfiguredDuration(String name) {
    return shootForConfiguredDuration(name, AIM_TIMEOUT_SECONDS, null);
  }

  private Command shootForConfiguredDuration(
      String name,
      double aimTimeoutSeconds,
      String logPrefix) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  superStructure.setDirectShootRequested(false);
                  superStructure.setShootRequested(true);
                  recordShootLog(logPrefix, "AimTimedOut", false);
                  recordShootLog(logPrefix, "ForcedShot", false);
                },
                superStructure),
            Commands.waitUntil(
                    () -> superStructure.isShooting() || superStructure.isFaulted())
                .withTimeout(aimTimeoutSeconds),
            Commands.runOnce(
                () -> {
                  if (!superStructure.isShooting() && !superStructure.isFaulted()) {
                    superStructure.setShootRequested(false);
                    superStructure.setDirectShootRequested(true);
                    DriverStation.reportWarning(
                        "Auto: " + name + " did not aim within " + aimTimeoutSeconds
                            + " seconds; forcing shot",
                        false);
                    recordShootLog(logPrefix, "AimTimedOut", true);
                    recordShootLog(logPrefix, "ForcedShot", true);
                  }
                },
                superStructure),
            Commands.waitSeconds(Math.max(0.0, shootSecondsSupplier.getAsDouble()))
                .onlyIf(() -> !superStructure.isFaulted()),
            Commands.runOnce(() -> clearShootRequests(logPrefix), superStructure))
        .finallyDo(interrupted -> clearShootRequests(logPrefix))
        .withName("Shoot-" + name);
  }

  private void clearShootRequests() {
    clearShootRequests(null);
  }

  private void clearShootRequests(String logPrefix) {
    superStructure.setShootRequested(false);
    superStructure.setDirectShootRequested(false);
    recordShootLog(logPrefix, "ForcedShot", false);
  }

  private static void recordShootLog(String logPrefix, String key, boolean value) {
    Logger.recordOutput("Auto/" + key, value);
    if (logPrefix != null) {
      Logger.recordOutput(logPrefix + "/" + key, value);
    }
  }

  private static void configureBLineLogging() {
    FollowPath.setPoseLoggingConsumer(
        value -> Logger.recordOutput(blineLogKey(value.getFirst()), value.getSecond()));
    FollowPath.setDoubleLoggingConsumer(
        value -> Logger.recordOutput(blineLogKey(value.getFirst()), value.getSecond()));
    FollowPath.setBooleanLoggingConsumer(
        value -> Logger.recordOutput(blineLogKey(value.getFirst()), value.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        value -> Logger.recordOutput(blineLogKey(value.getFirst()), value.getSecond()));
  }

  private static String blineLogKey(String libraryKey) {
    return "Auto/BLine/" + libraryKey.replaceFirst("^FollowPath/", "");
  }
}
