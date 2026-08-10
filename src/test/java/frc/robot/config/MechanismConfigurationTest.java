// package frc.robot.config;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import org.junit.jupiter.api.Test;

// class MechanismConfigurationTest {
//   @Test
//   void competitionConfigurationMatchesProvidedCanIdsAndVoltages() {
//     RobotConfiguration configuration = RobotConfiguration.competitionRobot();

//     assertEquals(9, configuration.intake().alwaysOnMotorCanId());
//     assertEquals(10, configuration.intake().circleMotorCanId());
//     assertEquals(7.0, configuration.intake().alwaysOnVolts(), 1.0e-9);
//     assertEquals(3.0, configuration.intake().circleMotorVolts(), 1.0e-9);

//     assertEquals(30, configuration.shooter().shootOneCanId());
//     assertEquals(31, configuration.shooter().shootTwoCanId());
//     assertEquals(32, configuration.shooter().shootUpCanId());
//     assertEquals(-7.0, configuration.shooter().shootOneVolts(), 1.0e-9);
//     assertEquals(7.0, configuration.shooter().shootTwoVolts(), 1.0e-9);
//     assertEquals(7.0, configuration.shooter().shootUpVolts(), 1.0e-9);
//     assertEquals(1.0, configuration.shooter().shootUpDelaySeconds(), 1.0e-9);
//   }
// }
