package frc.robot.config;

/** Selects all robot-specific constants without leaking hardware details into subsystems. */
public interface RobotConfiguration {
  DriveConfiguration drive();

  IntakeConfiguration intake();

  ShooterConfiguration shooter();

  VisionConfiguration vision();

  static RobotConfiguration competitionRobot() {
    return new CompetitionRobotConfig();
  }
}
