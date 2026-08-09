package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.config.DriveConfiguration;
import frc.robot.config.IntakeConfiguration;
import frc.robot.config.ShooterConfiguration;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SuperStructureTest {
  private FakeDriveIO driveIO;
  private FakeIntakeIO intakeIO;
  private FakeShooterIO shooterIO;
  private Drive drive;
  private Intake intake;
  private Shooter shooter;
  private SuperStructure superStructure;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    SimHooks.restartTiming();
    SimHooks.pauseTiming();
    RobotState robotState = new RobotState();
    driveIO = new FakeDriveIO();
    driveIO.pose =
        new Pose2d(
            FieldConstants.BLUE_HUB.minus(new Translation2d(2.0, 0.0)),
            Rotation2d.k180deg);
    drive =
        new Drive(
            robotState,
            driveIO,
            new DriveConfiguration(null, null, null, 5.0, 10.0));
    intakeIO = new FakeIntakeIO();
    intake =
        new Intake(
            intakeIO, new IntakeConfiguration("", 9, 10, 7.0, 3.0, 40.0, 80.0));
    shooterIO = new FakeShooterIO();
    shooter = new Shooter(shooterIO, shooterConfiguration());
    superStructure = new SuperStructure(drive, robotState, intake, shooter);
    drive.periodic();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimHooks.resumeTiming();
  }

  @Test
  void flywheelStaysOffUntilAimIsStableForPointOneSeconds() {
    superStructure.setShootRequested(true);

    runMechanismCycle();
    assertEquals(SuperStructure.SystemState.AIMING, superStructure.getSystemState());
    assertEquals(Shooter.WantedState.OFF, shooter.getWantedState());
    assertEquals(Intake.WantedState.OFF, intake.getWantedState());

    SimHooks.stepTiming(0.11);
    drive.periodic();
    runMechanismCycle();

    assertEquals(SuperStructure.SystemState.SHOOTING, superStructure.getSystemState());
    assertEquals(Shooter.WantedState.RUN, shooter.getWantedState());
    assertEquals(Intake.WantedState.OFF, intake.getWantedState());
    assertArrayEquals(new double[] {-7.0, 7.0, 0.0}, shooterIO.volts, 1.0e-9);
  }

  @Test
  void releasingShootImmediatelyStopsBothMechanisms() {
    enterShooting();

    superStructure.setShootRequested(false);
    runMechanismCycle();

    assertEquals(SuperStructure.SystemState.IDLE, superStructure.getSystemState());
    assertArrayEquals(new double[3], shooterIO.volts, 1.0e-9);
    assertEquals(0.0, intakeIO.alwaysOnVolts, 1.0e-9);
    assertEquals(0.0, intakeIO.circleVolts, 1.0e-9);
  }

  @Test
  void sixDegreeHeadingLossStopsFlywheelAndReturnsToAiming() {
    enterShooting();
    driveIO.pose = new Pose2d(driveIO.pose.getTranslation(), Rotation2d.fromDegrees(170.0));
    drive.periodic();

    runMechanismCycle();

    assertEquals(SuperStructure.SystemState.AIMING, superStructure.getSystemState());
    assertArrayEquals(new double[3], shooterIO.volts, 1.0e-9);
    assertEquals(0.0, intakeIO.alwaysOnVolts, 1.0e-9);
    assertEquals(0.0, intakeIO.circleVolts, 1.0e-9);
  }

  @Test
  void directShootStartsMainWheelsImmediatelyAndDelaysShootUpOneSecond() {
    superStructure.setDirectShootRequested(true);

    runMechanismCycle();

    assertEquals(SuperStructure.SystemState.DIRECT_SHOOTING, superStructure.getSystemState());
    assertEquals(Drive.ControlMode.TELEOP, drive.getControlMode());
    assertArrayEquals(new double[] {-7.0, 7.0, 0.0}, shooterIO.volts, 1.0e-9);

    SimHooks.stepTiming(0.99);
    runMechanismCycle();
    assertArrayEquals(new double[] {-7.0, 7.0, 0.0}, shooterIO.volts, 1.0e-9);

    SimHooks.stepTiming(0.02);
    runMechanismCycle();
    assertArrayEquals(new double[] {-7.0, 7.0, 7.0}, shooterIO.volts, 1.0e-9);

    superStructure.setDirectShootRequested(false);
    runMechanismCycle();

    assertEquals(SuperStructure.SystemState.IDLE, superStructure.getSystemState());
    assertArrayEquals(new double[3], shooterIO.volts, 1.0e-9);

    superStructure.setDirectShootRequested(true);
    runMechanismCycle();
    assertArrayEquals(new double[] {-7.0, 7.0, 0.0}, shooterIO.volts, 1.0e-9);
  }

  @Test
  void rearShooterHeadingWrapsAcrossPlusMinus180() {
    Pose2d robot = new Pose2d(2.0, 2.0, Rotation2d.kZero);

    Rotation2d heading =
        SuperStructure.calculateRearShooterHeading(robot, new Translation2d(1.0, 1.99));

    assertEquals(Math.toDegrees(Math.atan2(-0.01, -1.0)) + 180.0, heading.getDegrees(), 1.0e-9);
  }

  private void enterShooting() {
    superStructure.setShootRequested(true);
    runMechanismCycle();
    SimHooks.stepTiming(0.11);
    drive.periodic();
    runMechanismCycle();
    assertEquals(SuperStructure.SystemState.SHOOTING, superStructure.getSystemState());
  }

  private void runMechanismCycle() {
    superStructure.periodic();
    intake.periodic();
    shooter.periodic();
  }

  private static ShooterConfiguration shooterConfiguration() {
    return new ShooterConfiguration(
        "", 30, 31, 32, -7.0, 7.0, 7.0, 1.0, 35.0, 60.0);
  }

  private static final class FakeDriveIO implements DriveIO {
    private Pose2d pose = Pose2d.kZero;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private double timestamp;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
      inputs.pose = pose;
      inputs.gyroYaw = pose.getRotation();
      inputs.measuredRobotRelativeSpeeds = speeds;
      inputs.timestampSeconds = timestamp;
      timestamp += 0.02;
    }

    @Override
    public void runVelocity(ChassisSpeeds robotRelativeSpeeds) {}

    @Override
    public void resetPose(Pose2d pose) {
      this.pose = pose;
    }
  }

  private static final class FakeIntakeIO implements IntakeIO {
    private double alwaysOnVolts;
    private double circleVolts;

    @Override
    public void setVoltages(double alwaysOnVolts, double circleMotorVolts) {
      this.alwaysOnVolts = alwaysOnVolts;
      this.circleVolts = circleMotorVolts;
    }
  }

  private static final class FakeShooterIO implements ShooterIO {
    private final double[] volts = new double[3];

    @Override
    public void setVoltages(double shootOneVolts, double shootTwoVolts, double shootUpVolts) {
      volts[0] = shootOneVolts;
      volts[1] = shootTwoVolts;
      volts[2] = shootUpVolts;
    }
  }
}
