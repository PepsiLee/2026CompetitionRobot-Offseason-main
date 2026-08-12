// package frc.robot.subsystems.intake;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.config.IntakeConfiguration;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeAll;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// class IntakeVoltageTest {
//   private FakeIntakeIO io;
//   private Intake intake;

//   @BeforeAll
//   static void initializeHal() {
//     HAL.initialize(500, 0);
//   }

//   @BeforeEach
//   void setUp() {
//     DriverStationSim.resetData();
//     io = new FakeIntakeIO();
//     intake =
//         new Intake(io, new IntakeConfiguration("", 9, 10, 7.0, 3.0, 40.0, 80.0));
//   }

//   @AfterEach
//   void tearDown() {
//     CommandScheduler.getInstance().cancelAll();
//     CommandScheduler.getInstance().unregisterAllSubsystems();
//     DriverStationSim.resetData();
//     DriverStationSim.notifyNewData();
//   }

//   @Test
//   void teleopEnableRunsCanNineAtSevenVoltsWithoutButtonRequest() {
//     enableTeleop();

//     intake.periodic();

//     assertEquals(7.0, io.alwaysOnVolts, 1.0e-9);
//     assertEquals(0.0, io.circleMotorVolts, 1.0e-9);
//   }

//   @Test
//   void intakeRequestAddsThreeVoltsOnCanTen() {
//     enableTeleop();
//     intake.setWantedState(Intake.WantedState.INTAKE);

//     intake.periodic();

//     assertEquals(7.0, io.alwaysOnVolts, 1.0e-9);
//     assertEquals(3.0, io.circleMotorVolts, 1.0e-9);
//   }

//   @Test
//   void disabledRobotStopsBothMotors() {
//     intake.setWantedState(Intake.WantedState.INTAKE);
//     DriverStationSim.setEnabled(false);
//     DriverStationSim.notifyNewData();

//     intake.periodic();

//     assertEquals(0.0, io.alwaysOnVolts, 1.0e-9);
//     assertEquals(0.0, io.circleMotorVolts, 1.0e-9);
//   }

//   @Test
//   void stoppedStateOverridesAlwaysOnTeleopBehavior() {
//     enableTeleop();
//     intake.setWantedState(Intake.WantedState.STOPPED);

//     intake.periodic();

//     assertEquals(0.0, io.alwaysOnVolts, 1.0e-9);
//     assertEquals(0.0, io.circleMotorVolts, 1.0e-9);
//   }

//   private static void enableTeleop() {
//     DriverStationSim.setAutonomous(false);
//     DriverStationSim.setTest(false);
//     DriverStationSim.setEnabled(true);
//     DriverStationSim.notifyNewData();
//   }

//   private static final class FakeIntakeIO implements IntakeIO {
//     private double alwaysOnVolts;
//     private double circleMotorVolts;

//     @Override
//     public void setVoltages(double alwaysOnVolts, double circleMotorVolts) {
//       this.alwaysOnVolts = alwaysOnVolts;
//       this.circleMotorVolts = circleMotorVolts;
//     }
//   }
// }
