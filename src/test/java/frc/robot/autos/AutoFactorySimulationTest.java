package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.config.DriveConfiguration;
import frc.robot.config.IntakeConfiguration;
import frc.robot.config.ShooterConfiguration;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutoFactorySimulationTest {
  private static final double LOOP_PERIOD_SECONDS = 0.020;

  private FakeDriveIO driveIO;
  private FakeIntakeIO intakeIO;
  private FakeShooterIO shooterIO;
  private AutoFactory autoFactory;
  private Command autoCommand;
  private double blinePathTimeoutSeconds;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    SimHooks.restartTiming();
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    RobotState robotState = new RobotState();
    driveIO = new FakeDriveIO();
    Drive drive =
        new Drive(
            robotState,
            driveIO,
            new DriveConfiguration(null, null, null, 5.0, 10.0));
    intakeIO = new FakeIntakeIO();
    Intake intake =
        new Intake(
            intakeIO, new IntakeConfiguration("", 9, 10, 7.0, 3.0, 40.0, 80.0));
    shooterIO = new FakeShooterIO();
    shooterIO.intakeRunningSupplier = () -> intakeIO.currentlyRunning;
    shooterIO.poseSupplier = () -> driveIO.pose;
    ShooterConfiguration shooterConfiguration = shooterConfiguration();
    Shooter shooter = new Shooter(shooterIO, shooterConfiguration);
    Feeder feeder = new Feeder(null, null);
    SuperStructure superStructure = new SuperStructure(drive, robotState, intake, shooter, feeder);
    blinePathTimeoutSeconds = 12.0;
    autoFactory =
        new AutoFactory(
            drive,
            superStructure,
            () -> 0.10,
            () -> blinePathTimeoutSeconds);
    autoCommand =
        autoFactory.create(AutoMode.LEFT_COLLECT_RETURN, Alliance.Blue).command();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  @Test
  void completesCollectReturnAndTwoShotWindows() {
    CommandScheduler.getInstance().schedule(autoCommand);

    for (int i = 0; i < 1000 && autoCommand.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(autoCommand.isScheduled(), "auto should finish within twenty simulated seconds");
    assertFalse(autoFactory.isFaulted(), autoFactory.getFaultReason());
    assertTrue(intakeIO.everRanForward, "collect and shooting phases should run intake");
    assertTrue(shooterIO.startCount >= 2, "preload and return shots should both start shooter");
  }

  @Test
  void shootOnlyAimsFiresOnceAndNeverTranslates() {
    Command shootOnly = autoFactory.create(AutoMode.SHOOT_ONLY, Alliance.Blue).command();
    CommandScheduler.getInstance().schedule(shootOnly);

    for (int i = 0; i < 300 && shootOnly.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(shootOnly.isScheduled());
    assertFalse(autoFactory.isFaulted(), autoFactory.getFaultReason());
    assertEquals(1, shooterIO.startCount);
    assertFalse(intakeIO.everRanForward);
    assertEquals(0.0, driveIO.maximumTranslationSpeed, 1.0e-9);
  }

  @Test
  void shootOnlyForcesShotWhenAimDoesNotFinishWithinThreeSeconds() {
    driveIO.frozen = true;
    driveIO.resetHeadingOffsetRadians = Math.PI / 2.0;
    Command shootOnly = autoFactory.create(AutoMode.SHOOT_ONLY, Alliance.Blue).command();
    CommandScheduler.getInstance().schedule(shootOnly);

    for (int i = 0; i < 125; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertEquals(0, shooterIO.startCount, "shooter must wait during the three-second aim window");

    for (int i = 0; i < 100 && shootOnly.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(shootOnly.isScheduled());
    assertFalse(autoFactory.isFaulted(), "aim timeout must not fault the auto");
    assertEquals(1, shooterIO.startCount, "auto must force one shot after aim timeout");
    assertEquals(0.0, driveIO.maximumTranslationSpeed, 1.0e-9);
  }

  @Test
  void blineAutoIntakesThroughFinalPointThenAimsAndShootsOnce() {
    Command blineAuto =
        autoFactory.create(AutoMode.BLINE_INTAKE_SHOOT, Alliance.Blue).command();
    CommandScheduler.getInstance().schedule(blineAuto);

    for (int i = 0; i < 1200 && blineAuto.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(blineAuto.isScheduled(), "BLine auto should finish within twenty-four seconds");
    assertFalse(autoFactory.isFaulted(), autoFactory.getFaultReason());
    assertTrue(intakeIO.everRanForward);
    assertTrue(driveIO.maximumTranslationSpeed > 0.1);
    assertEquals(1, shooterIO.startCount, "BLine auto must not shoot a preload first");
    assertTrue(shooterIO.intakeWasOffAtFirstStart);
    assertEquals(1.34532, shooterIO.firstStartPose.getX(), 0.05);
    assertEquals(6.58422, shooterIO.firstStartPose.getY(), 0.05);
  }

  @Test
  void blineRedStartPoseIsFlippedBeforeAutoRuns() {
    AutoRoutine routine =
        autoFactory.create(AutoMode.BLINE_INTAKE_SHOOT, Alliance.Red);

    assertEquals(
        frc.robot.constants.FieldConstants.FIELD_LENGTH_METERS - 3.28868,
        routine.startingPose().getX(),
        1.0e-6);
    assertEquals(
        frc.robot.constants.FieldConstants.FIELD_WIDTH_METERS - 7.26792,
        routine.startingPose().getY(),
        1.0e-6);
  }

  @Test
  void blinePathTimeoutStopsIntakeAndNeverShoots() {
    driveIO.frozen = true;
    blinePathTimeoutSeconds = 0.10;
    Command blineAuto =
        autoFactory.create(AutoMode.BLINE_INTAKE_SHOOT, Alliance.Blue).command();
    CommandScheduler.getInstance().schedule(blineAuto);

    for (int i = 0; i < 100 && blineAuto.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }
    CommandScheduler.getInstance().run();

    assertFalse(
        blineAuto.isScheduled(),
        "fault="
            + autoFactory.getFaultReason()
            + ", intake="
            + intakeIO.currentlyRunning
            + ", shooterStarts="
            + shooterIO.startCount);
    assertTrue(autoFactory.isFaulted());
    assertTrue(autoFactory.getFaultReason().contains("BLine path timed out"));
    assertEquals(0, shooterIO.startCount);
    assertFalse(intakeIO.currentlyRunning);
  }

  @Test
  void stuckDriveTimesOutAndStopsTheRoutine() {
    driveIO.frozen = true;
    CommandScheduler.getInstance().schedule(autoCommand);

    for (int i = 0; i < 500 && autoCommand.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }
    CommandScheduler.getInstance().run();

    assertFalse(autoCommand.isScheduled());
    assertTrue(autoFactory.isFaulted());
    assertTrue(autoFactory.getFaultReason().contains("timed out"));
    assertTrue(
        Math.abs(driveIO.commandedSpeeds.vxMetersPerSecond) < 1.0e-9
            && Math.abs(driveIO.commandedSpeeds.vyMetersPerSecond) < 1.0e-9
            && Math.abs(driveIO.commandedSpeeds.omegaRadiansPerSecond) < 1.0e-9,
        "a faulted auto must command zero chassis speed");
  }

  private static ShooterConfiguration shooterConfiguration() {
    return new ShooterConfiguration(
        "", 30, 31, -7.0, 7.0, 7.0, 1.0, 35.0, 60.0);
  }

  private static final class FakeDriveIO implements DriveIO {
    private Pose2d pose = Pose2d.kZero;
    private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
    private double timestampSeconds;
    private boolean frozen;
    private double maximumTranslationSpeed;
    private double resetHeadingOffsetRadians;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
      if (!frozen) {
        ChassisSpeeds fieldRelative =
            ChassisSpeeds.fromRobotRelativeSpeeds(commandedSpeeds, pose.getRotation());
        pose =
            new Pose2d(
                pose.getX() + fieldRelative.vxMetersPerSecond * LOOP_PERIOD_SECONDS,
                pose.getY() + fieldRelative.vyMetersPerSecond * LOOP_PERIOD_SECONDS,
                pose.getRotation()
                    .plus(
                        Rotation2d.fromRadians(
                            commandedSpeeds.omegaRadiansPerSecond * LOOP_PERIOD_SECONDS)));
      }
      timestampSeconds += LOOP_PERIOD_SECONDS;
      inputs.pose = pose;
      inputs.gyroYaw = pose.getRotation();
      inputs.measuredRobotRelativeSpeeds = frozen ? new ChassisSpeeds() : commandedSpeeds;
      inputs.timestampSeconds = timestampSeconds;
    }

    @Override
    public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {
      commandedSpeeds = robotRelativeSpeeds;
      maximumTranslationSpeed =
          Math.max(
              maximumTranslationSpeed,
              Math.hypot(
                  robotRelativeSpeeds.vxMetersPerSecond,
                  robotRelativeSpeeds.vyMetersPerSecond));
    }

    @Override
    public void resetPose(Pose2d pose) {
      this.pose =
          new Pose2d(
              pose.getTranslation(),
              pose.getRotation().plus(Rotation2d.fromRadians(resetHeadingOffsetRadians)));
      commandedSpeeds = new ChassisSpeeds();
    }
  }

  private static final class FakeIntakeIO implements IntakeIO {
    private boolean everRanForward;
    private boolean currentlyRunning;

    @Override
    public void setVoltages(double alwaysOnVolts, double circleMotorVolts) {
      currentlyRunning = alwaysOnVolts > 0.0 || circleMotorVolts > 0.0;
      everRanForward |= currentlyRunning;
    }
  }

  private static final class FakeShooterIO implements ShooterIO {
    private boolean running;
    private int startCount;
    private BooleanSupplier intakeRunningSupplier = () -> false;
    private Supplier<Pose2d> poseSupplier = () -> Pose2d.kZero;
    private boolean intakeWasOffAtFirstStart;
    private Pose2d firstStartPose = Pose2d.kZero;

    @Override
    public void setVoltages(double shootOneVolts, double shootTwoVolts, double shootUpVolts) {
      boolean requested =
          Math.abs(shootOneVolts) > 0.0
              || Math.abs(shootTwoVolts) > 0.0
              || Math.abs(shootUpVolts) > 0.0;
      if (requested && !running) {
        if (startCount == 0) {
          intakeWasOffAtFirstStart = !intakeRunningSupplier.getAsBoolean();
          firstStartPose = poseSupplier.get();
        }
        startCount++;
      }
      running = requested;
    }
  }
}
