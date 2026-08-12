package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;

/** Builds the safe Do Nothing and single-shot autonomous routines. */
public final class AutoFactory {
  private static final String SHOOT_SECONDS_KEY = "Auto/ShootSeconds";
  private static final String SHOOT_PATH_NAME = "shoot";
  private static final double DEFAULT_SHOOT_SECONDS = 3.0;
  private static final double AIM_TIMEOUT_SECONDS = 3.0;

  private final Drive drive;
  private final SuperStructure superStructure;
  private final DoubleSupplier shootSecondsSupplier;
  private final Path shootPath;
  private final String shootPathLoadError;

  public AutoFactory(Drive drive, SuperStructure superStructure) {
    this(
        drive,
        superStructure,
        () -> SmartDashboard.getNumber(SHOOT_SECONDS_KEY, DEFAULT_SHOOT_SECONDS));
  }

  AutoFactory(
      Drive drive,
      SuperStructure superStructure,
      DoubleSupplier shootSecondsSupplier) {
    this.drive = drive;
    this.superStructure = superStructure;
    this.shootSecondsSupplier = shootSecondsSupplier;
    SmartDashboard.putNumber(SHOOT_SECONDS_KEY, DEFAULT_SHOOT_SECONDS);

    // Keep BLine's transforms on the exact same 2026 field dimensions used by
    // WPILib.
    FlippingUtil.fieldSizeX = FieldConstants.FIELD_LENGTH_METERS;
    FlippingUtil.fieldSizeY = FieldConstants.FIELD_WIDTH_METERS;
    configureBLineLogging();

    Path loadedPath = null;
    String loadError = "";
    try {
      loadedPath = new Path(SHOOT_PATH_NAME);
      if (!loadedPath.isValid()) {
        loadError = "BLine path '" + SHOOT_PATH_NAME + "' is invalid";
        loadedPath = null;
      }
    } catch (RuntimeException exception) {
      loadError = "Could not load BLine path '" + SHOOT_PATH_NAME + "': "
          + exception.getMessage();
    }
    shootPath = loadedPath;
    shootPathLoadError = loadError;
    Logger.recordOutput("Auto/BLine/PathLoaded", shootPath != null);
    Logger.recordOutput("Auto/BLine/PathLoadError", shootPathLoadError);
    if (shootPath != null) {
      Logger.recordOutput(
          "Auto/BLine/BluePathPoints",
          shootPath.getTranslations().toArray(Translation2d[]::new));
    }
  }

  public AutoRoutine create(AutoMode mode, Alliance alliance) {
    return switch (mode) {
      case DO_NOTHING -> createDoNothing();
      case SHOOT_ONLY -> createShootOnly(alliance);
      case BLINE_SHOOT -> createBLineShoot(alliance);
    };
  }

  private AutoRoutine createDoNothing() {
    Command routine = Commands.runOnce(
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
    Pose2d startingPose = FieldConstants.blueToAlliance(FieldConstants.BLUE_LEFT_START, alliance);
    Command routine = Commands.sequence(
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
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              superStructure.setDirectShootRequested(false);
              superStructure.setShootRequested(true);
              Logger.recordOutput("Auto/AimTimedOut", false);
              Logger.recordOutput("Auto/ForcedShot", false);
            },
            superStructure),
        Commands.waitUntil(
            () -> superStructure.isShooting() || superStructure.isFaulted())
            .withTimeout(AIM_TIMEOUT_SECONDS),
        Commands.runOnce(
            () -> {
              if (!superStructure.isShooting() && !superStructure.isFaulted()) {
                superStructure.setShootRequested(false);
                superStructure.setDirectShootRequested(true);
                DriverStation.reportWarning(
                    "Auto: " + name + " did not aim within 3 seconds; forcing shot",
                    false);
                Logger.recordOutput("Auto/AimTimedOut", true);
                Logger.recordOutput("Auto/ForcedShot", true);
              }
            },
            superStructure),
        Commands.waitSeconds(Math.max(0.0, shootSecondsSupplier.getAsDouble()))
            .onlyIf(() -> !superStructure.isFaulted()),
        Commands.runOnce(this::clearShootRequests, superStructure))
        .finallyDo(interrupted -> clearShootRequests())
        .withName("Shoot-" + name);
  }

  private void clearShootRequests() {
    superStructure.setShootRequested(false);
    superStructure.setDirectShootRequested(false);
    Logger.recordOutput("Auto/ForcedShot", false);
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
