package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Project-wide feature switches. Robot-specific hardware values belong in
 * RobotConfiguration.
 */
public final class Constants {
  private Constants() {
  }

  /**
   * Enables Maple-Sim drivetrain physics and 2026 field collision geometry in
   * simulation.
   */
  public static final boolean useMapleSim = true;

  public static class KrakenX60 {
    public static final AngularVelocity kFreeSpeed = RPM.of(6000);
  }
}
