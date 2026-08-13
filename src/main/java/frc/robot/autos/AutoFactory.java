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
  private static final Translation2d BLUE_SHOT_AIM_TARGET = tagMidpoint(25, 26);
  private static final Translation2d RED_SHOT_AIM_TARGET = tagMidpoint(9, 10);

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
    // 只供 AdvantageKit／Elastic 顯示：true 表示短路徑載入成功。
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
    // 只供 AdvantageKit／Elastic 顯示：true 表示 Full Shoot 路徑載入成功。
    Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/PathLoaded", fullShootPath != null);
    Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/PathLoadError", fullShootPathLoadError);
    // 以下兩個 false 只是初始化顯示狀態，不會控制 Intake 或底盤。
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
      case STOP_AT_FULL_SHOOT_START -> createStopAtFullShootStart(alliance);
      case SHOOT_ONLY -> createShootOnly(alliance);
      case BLINE_SHOOT -> createBLineShoot(alliance);
      case BLINE_FULL_SHOOT -> createFullBLineShoot(alliance);
    };
  }

  private AutoRoutine createDoNothing() {
    Command routine =
        Commands.runOnce(
                () -> {
                  cleanupAutoState();
                },
                drive,
                superStructure)
            .withName("Do Nothing");
    return new AutoRoutine(Pose2d.kZero, routine);
  }

  private AutoRoutine createStopAtFullShootStart(Alliance alliance) {
    if (fullShootPath == null) {
      Command safeFailure = Commands.runOnce(
              () -> {
                // false 表示錯誤訊息不附加 Java stack trace。
                DriverStation.reportError(fullShootPathLoadError, false);
                cleanupAutoState();
              },
              drive,
              superStructure)
          .withName("Stop at Full Shoot Start - Path Error");
      return new AutoRoutine(Pose2d.kZero, safeFailure);
    }

    Path transformedPath = fullShootPath.copy();
    if (alliance == Alliance.Red) {
      transformedPath.flip();
    }
    Pose2d startingPose = transformedPath.getStartPose();

    Command routine = Commands.sequence(
            resetForAuto(startingPose, alliance),
            stopAll())
        .finallyDo(
            interrupted -> {
              cleanupAutoState();
              Logger.recordOutput("Auto/StopAtFullShootStart/Interrupted", interrupted);
            })
        .withName(AutoMode.STOP_AT_FULL_SHOOT_START.name());
    return new AutoRoutine(startingPose, routine);
  }

  private AutoRoutine createShootOnly(Alliance alliance) {
    Pose2d startingPose =
        FieldConstants.blueToAlliance(FieldConstants.BLUE_LEFT_START, alliance);
    Command routine =
        Commands.sequence(
                resetForAuto(startingPose, alliance),
                shootForConfiguredDuration("Preload"),
                stopAll())
            .finallyDo(
                interrupted -> {
                  cleanupAutoState();
                  Logger.recordOutput("Auto/Interrupted", interrupted);
                })
            .withName(AutoMode.SHOOT_ONLY.name());
    return new AutoRoutine(startingPose, routine);
  }

  private AutoRoutine createBLineShoot(Alliance alliance) {
    if (shootPath == null) {
      Command safeFailure = Commands.runOnce(
              () -> {
                // false 表示錯誤訊息不附加 Java stack trace。
                DriverStation.reportError(shootPathLoadError, false);
                cleanupAutoState();
              },
              drive,
              superStructure)
          .withName("BLine Shoot - Path Error");
      return new AutoRoutine(Pose2d.kZero, safeFailure);
    }

    // Red 為 true，執行場地中心 180 度 alliance flip；Blue 為 false，使用原路徑。
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
        // false 表示不使用同源鏡像；Red 只做上面的 alliance flip。
        .withShouldMirror(() -> false)
        .withPoseReset(drive::resetPose)
        .build(shootPath);

    Command routine = Commands.sequence(
            resetForAuto(startingPose, alliance),
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Auto/BLine/Alliance", alliance.name());
                  Logger.recordOutput("Auto/BLine/ShouldFlip", shouldFlip);
                  // Logger 的 false 只表示 Elastic 顯示「沒有 mirror」，不控制路徑。
                  Logger.recordOutput("Auto/BLine/ShouldMirror", false);
                  // 尚未走完路徑，先在 Elastic 顯示 false。
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
                  // 路徑完成後只把 Elastic 的終點狀態標成 true。
                  Logger.recordOutput("Auto/BLine/ReachedFinalPoint", true);
                },
                drive),
            shootForConfiguredDuration("BLine Final Point"),
            stopAll())
        .finallyDo(
            interrupted -> {
              cleanupAutoState();
              Logger.recordOutput("Auto/Interrupted", interrupted);
              // Auto 結束或中斷後重設顯示，false 只影響 Logger。
              Logger.recordOutput("Auto/BLine/ReachedFinalPoint", false);
            })
        .withName(AutoMode.BLINE_SHOOT.name());
    return new AutoRoutine(startingPose, routine);
  }

  private AutoRoutine createFullBLineShoot(Alliance alliance) {
    if (fullShootPath == null) {
      Command safeFailure = Commands.runOnce(
              () -> {
                // false 表示錯誤訊息不附加 Java stack trace。
                DriverStation.reportError(fullShootPathLoadError, false);
                // 真正的機構 request：false 表示不要求 Intake 收球。
                superStructure.setIntakeRequested(false);
                cleanupAutoState();
                // 以下 false 只重設 Elastic 顯示，不會控制硬體。
                Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", false);
                Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", false);
              },
              drive,
              superStructure)
          .withName("BLine Full Shoot - Path Error");
      return new AutoRoutine(Pose2d.kZero, safeFailure);
    }

    // Red 為 true，執行場地中心 180 度 alliance flip；Blue 為 false，使用原路徑。
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
        // false 表示不使用同源鏡像；Red 只做上面的 alliance flip。
        .withShouldMirror(() -> false)
        .withPoseReset(drive::resetPose)
        .build(followerPath);

    Command routine = Commands.sequence(
            resetForAuto(startingPose, alliance),
            Commands.runOnce(
                () -> {
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/Alliance", alliance.name());
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ShouldFlip", shouldFlip);
                  // Logger 的 false 只表示 Elastic 顯示「沒有 mirror」，不控制路徑。
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ShouldMirror", false);
                  // 尚未走完路徑，先在 Elastic 顯示 false。
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", false);
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/StartingPose", startingPose);
                  Logger.recordOutput(
                      FULL_SHOOT_LOG_PREFIX + "/ActivePathPoints",
                      transformedPath.getTranslations().toArray(Translation2d[]::new));
                  // Deliberately keep Intake disabled while following this Auto path.
                  // 真正的機構 request：false 表示走路徑期間不要求 Intake 收球。
                  superStructure.setIntakeRequested(false);
                  // 這個 false 只是讓 Elastic 顯示 Intake 未啟動。
                  Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", false);
                },
                superStructure),
            followPath,
            Commands.runOnce(
                () -> {
                  drive.acceptPathSpeeds(new ChassisSpeeds());
                  drive.stop();
                  // 到終點後仍不要求 Intake 收球。
                  superStructure.setIntakeRequested(false);
                  // 第一個 false 是 Intake 顯示；true 表示已抵達路徑終點。
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
              // 不論正常完成或中途取消，都關閉 Intake request。
              superStructure.setIntakeRequested(false);
              cleanupAutoState(FULL_SHOOT_LOG_PREFIX);
              // interrupted=true 表示 Command 被取消；false 表示正常完成。
              Logger.recordOutput("Auto/Interrupted", interrupted);
              // 以下 false 只把 Elastic 狀態恢復成未啟動／未抵達。
              Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/IntakeActive", false);
              Logger.recordOutput(FULL_SHOOT_LOG_PREFIX + "/ReachedFinalPoint", false);
            })
        .withName(AutoMode.BLINE_FULL_SHOOT.name());
    return new AutoRoutine(startingPose, routine);
  }

  private Command resetForAuto(Pose2d startingPose, Alliance alliance) {
    return Commands.runOnce(
        () -> {
          // true 表示在 Auto 中禁止平時的 ONLY_ROLLER，沒有 Intake request 時全部輸出 0 V。
          superStructure.setAutoIntakeIdleSuppressed(true);
          superStructure.setAutoShotTargets(
              shotAimTargetForAlliance(alliance), FieldConstants.hubForAlliance(alliance));
          superStructure.clearStopped();
          // Auto 初始化時不要求 Intake 收球。
          superStructure.setIntakeRequested(false);
          clearShootRequests();
          drive.resetPose(startingPose);
          drive.stop();
          Logger.recordOutput("Auto/ShotAimTarget", shotAimTargetForAlliance(alliance));
        },
        drive,
        superStructure);
  }

  private Command stopAll() {
    return Commands.runOnce(
        () -> {
          cleanupAutoState();
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
                  // 先關閉強制射擊，再開啟需要瞄準的正常射擊流程。
                  superStructure.setDirectShootRequested(false);
                  superStructure.setShootRequested(true);
                  // 以下 false 只初始化 Logger：尚未 timeout，也尚未強制射擊。
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
                    // 瞄準超時：關閉正常瞄準射擊，改開啟 direct shoot。
                    superStructure.setShootRequested(false);
                    superStructure.setDirectShootRequested(true);
                    DriverStation.reportWarning(
                        "Auto: " + name + " did not aim within " + aimTimeoutSeconds
                            + " seconds; forcing shot",
                        false); // false 表示警告不附加 Java stack trace。
                    // 以下 true 只記錄 Elastic：已 timeout 並進入強制射擊。
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
    // 真正停止兩種射擊 request。
    superStructure.setShootRequested(false);
    superStructure.setDirectShootRequested(false);
    // Logger 恢復成沒有強制射擊。
    recordShootLog(logPrefix, "ForcedShot", false);
  }

  private void cleanupAutoState() {
    cleanupAutoState(null);
  }

  private void cleanupAutoState(String logPrefix) {
    clearShootRequests(logPrefix);
    superStructure.stopAll();
    superStructure.clearAutoShotTargets();
    // false 表示取消 Auto 的 idle 抑制，恢復 Teleop 原本的 ONLY_ROLLER 行為。
    superStructure.setAutoIntakeIdleSuppressed(false);
    //check need
    drive.stop();
  }

  private static Translation2d shotAimTargetForAlliance(Alliance alliance) {
    return alliance == Alliance.Red ? RED_SHOT_AIM_TARGET : BLUE_SHOT_AIM_TARGET;
  }

  private static Translation2d tagMidpoint(int firstId, int secondId) {
    Translation2d first = FieldConstants.FIELD_LAYOUT
        .getTagPose(firstId)
        .orElseThrow(() -> new IllegalStateException("Missing AprilTag " + firstId))
        .getTranslation()
        .toTranslation2d();
    Translation2d second = FieldConstants.FIELD_LAYOUT
        .getTagPose(secondId)
        .orElseThrow(() -> new IllegalStateException("Missing AprilTag " + secondId))
        .getTranslation()
        .toTranslation2d();
    return first.plus(second).div(2.0);
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
