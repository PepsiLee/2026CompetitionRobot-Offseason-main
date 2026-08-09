package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Builds fixed-coordinate and BLine autonomous routines. */
public final class AutoFactory {
  private static final String SHOOT_SECONDS_KEY = "Auto/ShootSeconds";
  private static final String BLINE_PATH_TIMEOUT_SECONDS_KEY =
      "Auto/BLinePathTimeoutSeconds";
  private static final double DEFAULT_SHOOT_SECONDS = 3.0;
  private static final double DEFAULT_BLINE_PATH_TIMEOUT_SECONDS = 12.0;
  private static final double AIM_TIMEOUT_SECONDS = 3.0;
  private static final double DEFAULT_POSITION_TOLERANCE_METERS = 0.10;
  private static final double DEFAULT_HEADING_TOLERANCE_RADIANS = Units.degreesToRadians(4.0);

  private final Drive drive;
  private final SuperStructure superStructure;
  private final BLinePathing blinePathing;
  private final DoubleSupplier shootSecondsSupplier;
  private final DoubleSupplier blinePathTimeoutSecondsSupplier;
  private boolean autoFault;
  private String autoFaultReason = "";

  public AutoFactory(Drive drive, SuperStructure superStructure) {
    this(
        drive,
        superStructure,
        () -> SmartDashboard.getNumber(SHOOT_SECONDS_KEY, DEFAULT_SHOOT_SECONDS),
        () ->
            SmartDashboard.getNumber(
                BLINE_PATH_TIMEOUT_SECONDS_KEY, DEFAULT_BLINE_PATH_TIMEOUT_SECONDS));
  }

  AutoFactory(
      Drive drive,
      SuperStructure superStructure,
      DoubleSupplier shootSecondsSupplier,
      DoubleSupplier blinePathTimeoutSecondsSupplier) {
    this.drive = drive;
    this.superStructure = superStructure;
    this.shootSecondsSupplier = shootSecondsSupplier;
    this.blinePathTimeoutSecondsSupplier = blinePathTimeoutSecondsSupplier;
    blinePathing = new BLinePathing(drive);
    SmartDashboard.putNumber(SHOOT_SECONDS_KEY, DEFAULT_SHOOT_SECONDS);
    SmartDashboard.putNumber(
        BLINE_PATH_TIMEOUT_SECONDS_KEY, DEFAULT_BLINE_PATH_TIMEOUT_SECONDS);
  }

  public AutoRoutine create(AutoMode mode, Alliance alliance) {
    if (mode == AutoMode.DO_NOTHING) {
      return new AutoRoutine(
          Pose2d.kZero,
          Commands.runOnce(
                  () -> {
                    clearFault();
                    superStructure.stopAll();
                    drive.stop();
                  },
                  drive,
                  superStructure)
              .withName("Do Nothing"));
    }

    if (mode == AutoMode.SHOOT_ONLY) {
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
              .withName(mode.name());
      return new AutoRoutine(startingPose, routine);
    }

    if (mode == AutoMode.BLINE_INTAKE_SHOOT) {
      return createBLineIntakeShoot(alliance);
    }

    boolean mirrorToRight = mode == AutoMode.RIGHT_COLLECT_RETURN;
    Pose2d startingPose = transform(FieldConstants.BLUE_LEFT_START, mirrorToRight, alliance);
    AutoWaypoint collectEntry =
        waypoint(
            transform(FieldConstants.BLUE_LEFT_COLLECT_ENTRY, mirrorToRight, alliance),
            2.5,
            4.0);
    AutoWaypoint collectSweep =
        waypoint(
            transform(FieldConstants.BLUE_LEFT_COLLECT_SWEEP, mirrorToRight, alliance),
            2.0,
            4.0);
    AutoWaypoint returnToScore =
        waypoint(
            transform(FieldConstants.BLUE_LEFT_RETURN_SCORE, mirrorToRight, alliance),
            2.5,
            5.0);

    Command routine =
        Commands.sequence(
                resetForAuto(startingPose),
                shootForConfiguredDuration("Preload"),
                Commands.runOnce(() -> superStructure.setIntakeRequested(true), superStructure)
                    .onlyIf(this::isHealthy),
                driveToWaypoint("CollectEntry", collectEntry).onlyIf(this::isHealthy),
                driveToWaypoint("CollectSweep", collectSweep).onlyIf(this::isHealthy),
                Commands.runOnce(() -> superStructure.setIntakeRequested(false), superStructure),
                driveToWaypoint("ReturnToScore", returnToScore).onlyIf(this::isHealthy),
                shootForConfiguredDuration("ReturnShot").onlyIf(this::isHealthy),
                stopAll())
            .finallyDo(
                interrupted -> {
                  superStructure.stopAll();
                  drive.stop();
                  Logger.recordOutput("Auto/Interrupted", interrupted);
                })
            .withName(mode.name());

    return new AutoRoutine(startingPose, routine);
  }

  public boolean isFaulted() {
    return autoFault;
  }

  public String getFaultReason() {
    return autoFaultReason;
  }

  private AutoRoutine createBLineIntakeShoot(Alliance alliance) {
    final Path path;
    final Pose2d startingPose;
    try {
      path = blinePathing.loadIntakeShootPath();
      if (!path.isValid()) {
        return invalidBLineRoutine("BLine path is invalid");
      }
      startingPose = blinePathing.getStartPose(path, alliance);
    } catch (RuntimeException exception) {
      return invalidBLineRoutine("BLine path failed to load: " + exception.getMessage());
    }

    FollowPath followPath = blinePathing.createFollowCommand(path, alliance);
    double pathTimeoutSeconds =
        Math.max(0.0, blinePathTimeoutSecondsSupplier.getAsDouble());

    Command guardedPath =
        Commands.sequence(
                followPath.withTimeout(pathTimeoutSeconds),
                Commands.runOnce(
                    () -> {
                      if (!followPath.isFinished()) {
                        fail("BLine path timed out");
                      }
                    }))
            .withName("Follow-BLine-Intake-Shoot");

    Command routine =
        Commands.sequence(
                resetForAuto(startingPose),
                Commands.runOnce(
                        () -> superStructure.setIntakeRequested(true), superStructure)
                    .onlyIf(this::isHealthy),
                guardedPath.onlyIf(this::isHealthy),
                Commands.runOnce(
                    () -> superStructure.setIntakeRequested(false), superStructure),
                shootForConfiguredDuration("BLineReturnShot").onlyIf(this::isHealthy),
                stopAll())
            .finallyDo(
                interrupted -> {
                  superStructure.stopAll();
                  drive.stop();
                  Logger.recordOutput("Auto/Interrupted", interrupted);
                })
            .withName(AutoMode.BLINE_INTAKE_SHOOT.name());

    return new AutoRoutine(startingPose, routine);
  }

  private AutoRoutine invalidBLineRoutine(String reason) {
    Command routine =
        Commands.runOnce(
                () -> {
                  clearFault();
                  fail(reason);
                },
                drive,
                superStructure)
            .withName("Invalid-BLine-Auto");
    return new AutoRoutine(Pose2d.kZero, routine);
  }

  private Command resetForAuto(Pose2d startingPose) {
    return Commands.runOnce(
        () -> {
          clearFault();
          superStructure.clearStopped();
          superStructure.setIntakeRequested(false);
          superStructure.setShootRequested(false);
          superStructure.setDirectShootRequested(false);
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

  private Command driveToWaypoint(String name, AutoWaypoint waypoint) {
    return Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.requestDriveToPose(
                        waypoint.pose(),
                        waypoint.maxSpeedMetersPerSecond(),
                        waypoint.positionToleranceMeters(),
                        waypoint.headingToleranceRadians()),
                drive),
            Commands.waitUntil(drive::isAtDriveToPoseSetpoint)
                .withTimeout(waypoint.timeoutSeconds()),
            Commands.runOnce(
                () -> {
                  if (!drive.isAtDriveToPoseSetpoint()) {
                    fail(name + " timed out");
                  } else {
                    drive.stop();
                  }
                },
                drive))
        .withName("DriveTo-" + name);
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
            Commands.waitSeconds(
                    Math.max(0.0, shootSecondsSupplier.getAsDouble()))
                .onlyIf(this::isHealthy),
            Commands.runOnce(this::clearShootRequests, superStructure))
        .finallyDo(interrupted -> clearShootRequests())
        .withName("Shoot-" + name);
  }

  private void clearShootRequests() {
    superStructure.setShootRequested(false);
    superStructure.setDirectShootRequested(false);
    Logger.recordOutput("Auto/ForcedShot", false);
  }

  private void fail(String reason) {
    autoFault = true;
    autoFaultReason = reason;
    superStructure.stopAll();
    drive.stop();
    DriverStation.reportError("Auto fault: " + reason, false);
    Logger.recordOutput("Auto/Faulted", true);
    Logger.recordOutput("Auto/FaultReason", reason);
  }

  private void clearFault() {
    autoFault = false;
    autoFaultReason = "";
    Logger.recordOutput("Auto/Faulted", false);
    Logger.recordOutput("Auto/FaultReason", "");
  }

  private boolean isHealthy() {
    return !autoFault && !superStructure.isFaulted();
  }

  private static AutoWaypoint waypoint(Pose2d pose, double maxSpeed, double timeoutSeconds) {
    return new AutoWaypoint(
        pose,
        maxSpeed,
        DEFAULT_POSITION_TOLERANCE_METERS,
        DEFAULT_HEADING_TOLERANCE_RADIANS,
        timeoutSeconds);
  }

  public static Pose2d transform(Pose2d blueLeftPose, boolean mirrorToRight, Alliance alliance) {
    Pose2d sidePose =
        mirrorToRight ? FieldConstants.mirrorLeftToRight(blueLeftPose) : blueLeftPose;
    return FieldConstants.blueToAlliance(sidePose, alliance);
  }
}
