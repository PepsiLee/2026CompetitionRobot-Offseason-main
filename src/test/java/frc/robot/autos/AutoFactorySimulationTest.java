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
import frc.robot.config.FeederConfiguration;
import frc.robot.config.IntakeConfiguration;
import frc.robot.config.ShooterConfiguration;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutoFactorySimulationTest {
  private static final double LOOP_PERIOD_SECONDS = 0.020;

  private FakeDriveIO driveIO;
  private FakeIntakeIO intakeIO;
  private FakeShooterIO shooterIO;
  private FakeFeederIO feederIO;
  private Drive drive;
  private SuperStructure superStructure;
  private AutoFactory autoFactory;

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
    drive =
        new Drive(
            robotState,
            driveIO,
            new DriveConfiguration(null, null, null, 5.0, 10.0));
    intakeIO = new FakeIntakeIO();
    Intake intake =
        new Intake(
            intakeIO, new IntakeConfiguration("", 9, 10, 11, 7.0, 3.0, 40.0, 80.0));
    shooterIO = new FakeShooterIO();
    ShooterConfiguration shooterConfiguration = shooterConfiguration();
    Shooter shooter = new Shooter(shooterIO, shooterConfiguration);
    feederIO = new FakeFeederIO();
    Feeder feeder = new Feeder(
        feederIO,
        new FeederConfiguration("", 32, -8.0, 8.0, 120.0));
    superStructure = new SuperStructure(drive, robotState, intake, shooter, feeder);
    // Snowflake debounces shooter readiness for 0.20 seconds before feeding.
    autoFactory = new AutoFactory(drive, superStructure, () -> 0.30);
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
  void doNothingStopsWithoutRunningAnyMechanism() {
    Command doNothing = autoFactory.create(AutoMode.DO_NOTHING, Alliance.Blue).command();
    CommandScheduler.getInstance().schedule(doNothing);

    for (int i = 0; i < 5 && doNothing.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(doNothing.isScheduled());
    assertEquals(0, shooterIO.startCount);
    assertFalse(intakeIO.everRanForward);
    assertEquals(0.0, driveIO.maximumTranslationSpeed, 1.0e-9);
  }

  @Test
  void stopAtFullShootStartUsesAlliancePoseAndRunsNoMechanism() {
    AutoRoutine blueStop =
        autoFactory.create(AutoMode.STOP_AT_FULL_SHOOT_START, Alliance.Blue);
    AutoRoutine blueFull = autoFactory.create(AutoMode.BLINE_FULL_SHOOT, Alliance.Blue);
    AutoRoutine redStop =
        autoFactory.create(AutoMode.STOP_AT_FULL_SHOOT_START, Alliance.Red);
    AutoRoutine redFull = autoFactory.create(AutoMode.BLINE_FULL_SHOOT, Alliance.Red);

    assertEquals(blueFull.startingPose(), blueStop.startingPose());
    assertEquals(redFull.startingPose(), redStop.startingPose());

    Command stop = blueStop.command();
    CommandScheduler.getInstance().schedule(stop);
    for (int i = 0; i < 5 && stop.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(stop.isScheduled());
    assertEquals(blueStop.startingPose(), drive.getPose());
    assertEquals(0.0, driveIO.maximumTranslationSpeed, 1.0e-9);
    assertEquals(0, shooterIO.startCount);
    assertFalse(feederIO.everFed);
    assertFalse(intakeIO.everRanForward);
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
    assertEquals(1, shooterIO.startCount, "auto must force one shot after aim timeout");
    assertEquals(0.0, driveIO.maximumTranslationSpeed, 1.0e-9);
  }

  @Test
  void fullBLineAutoKeepsIntakeOffThenAimsAndShoots() {
    Command fullShoot = autoFactory.create(AutoMode.BLINE_FULL_SHOOT, Alliance.Blue).command();
    CommandScheduler.getInstance().schedule(fullShoot);

    for (int i = 0; i < 2500 && fullShoot.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      if (drive.getControlMode() == Drive.ControlMode.PATH_FOLLOWING) {
        assertFalse(feederIO.everFed, "feeder must remain stopped while following the path");
      }
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(fullShoot.isScheduled());
    assertFalse(intakeIO.everRanForward);
    assertFalse(intakeIO.currentlyRunning);
    assertTrue(driveIO.maximumTranslationSpeed > 0.0);
    assertEquals(1, shooterIO.startCount);
    assertTrue(feederIO.everFed);
  }

  @Test
  void fullBLineAutoUsesItsTenSecondForcedShotTimeout() {
    Command fullShoot = autoFactory.create(AutoMode.BLINE_FULL_SHOOT, Alliance.Blue).command();
    CommandScheduler.getInstance().schedule(fullShoot);

    boolean reachedFinalPoint = false;
    boolean pathStarted = false;
    for (int i = 0; i < 2000 && fullShoot.isScheduled() && !reachedFinalPoint; i++) {
      CommandScheduler.getInstance().run();
      pathStarted |= drive.getControlMode() == Drive.ControlMode.PATH_FOLLOWING;
      reachedFinalPoint =
          pathStarted && drive.getControlMode() != Drive.ControlMode.PATH_FOLLOWING;
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertTrue(reachedFinalPoint, "path must finish before testing the aim timeout");
    driveIO.frozen = true;

    for (int i = 0; i < 450; i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }
    assertEquals(0, shooterIO.startCount, "new auto must still be aiming before ten seconds");
    assertFalse(feederIO.everFed);

    for (int i = 0; i < 150 && fullShoot.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(fullShoot.isScheduled());
    assertEquals(1, shooterIO.startCount, "new auto must force exactly one shot after ten seconds");
    assertTrue(feederIO.everFed);
    assertFalse(intakeIO.everRanForward);
    assertFalse(intakeIO.currentlyRunning);
  }

  @Test
  void missingFullShootPathStopsSafelyWithoutShooting() {
    AutoFactory missingPathFactory = new AutoFactory(
        drive,
        superStructure,
        () -> 0.10,
        "shoot",
        "missing-full-shoot-test-path");
    Command fullShoot = missingPathFactory
        .create(AutoMode.BLINE_FULL_SHOOT, Alliance.Blue)
        .command();
    CommandScheduler.getInstance().schedule(fullShoot);

    for (int i = 0; i < 5 && fullShoot.isScheduled(); i++) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
    }

    assertFalse(fullShoot.isScheduled());
    assertEquals(0, shooterIO.startCount);
    assertFalse(feederIO.everFed);
    assertFalse(intakeIO.everRanForward);
    assertEquals(0.0, driveIO.maximumTranslationSpeed, 1.0e-9);
  }

  private static ShooterConfiguration shooterConfiguration() {
    return new ShooterConfiguration(
        "", 30, 31, -7.0, 7.0, 35.0, 60.0, 1.222);
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

    @Override
    public void setRPM(double rpm) {
      boolean requested = Math.abs(rpm) > 0.0;
      if (requested && !running) {
        startCount++;
      }
      running = requested;
    }

    @Override
    public boolean isVelocityWithinTolerance() {
      return running;
    }

    @Override
    public void stop() {
      running = false;
    }
  }

  private static final class FakeFeederIO implements FeederIO {
    private boolean everFed;

    @Override
    public void setVoltage(double volts) {
      everFed |= volts > 0.0;
    }
  }
}
