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

/** Builds the safe Do Nothing and single-shot autonomous routines. */
public final class AutoFactory {
  private static final String SHOOT_SECONDS_KEY = "Auto/ShootSeconds";
  private static final double DEFAULT_SHOOT_SECONDS = 3.0;
  private static final double AIM_TIMEOUT_SECONDS = 3.0;

  private final Drive drive;
  private final SuperStructure superStructure;
  private final DoubleSupplier shootSecondsSupplier;

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
  }

  public AutoRoutine create(AutoMode mode, Alliance alliance) {
    return switch (mode) {
      case DO_NOTHING -> createDoNothing();
      case SHOOT_ONLY -> createShootOnly(alliance);
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
}
