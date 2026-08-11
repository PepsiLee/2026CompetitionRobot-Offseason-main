package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;


public final class Constants {
  private Constants() {
  }
  //Simulation constants
  public static final boolean useMapleSim = true;

  public static class KrakenX60 {
    public static final AngularVelocity kFreeSpeed = RPM.of(6000);
  }
}
