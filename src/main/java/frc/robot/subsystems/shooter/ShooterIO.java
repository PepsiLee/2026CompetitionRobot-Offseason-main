package frc.robot.subsystems.shooter;

public interface ShooterIO {
  class Inputs {
    public boolean[] connected = new boolean[3];
    public double[] velocityRotationsPerSecond = new double[3];
    public double[] appliedVolts = new double[3];
    public double[] supplyCurrentAmps = new double[3];
    public double[] statorCurrentAmps = new double[3];
    public double[] temperatureCelsius = new double[3];
  }

  default void updateInputs(Inputs inputs) {
  }

  default void setVoltages(double shootOneVolts, double shootTwoVolts, double shootUpVolts) {
  }

  default void setRPM(double rpm) {
  }

  default void setPercentOutput(double percentOutput) {
  }

  default boolean isVelocityWithinTolerance() {
    return false;
  };

  default void stop() {
    setVoltages(0.0, 0.0, 0.0);
  }
}
