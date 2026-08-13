package frc.robot.subsystems.shooter;

public interface ShooterIO {
  class Inputs {
    public boolean[] connected = new boolean[2];
    public double[] velocityRotationsPerSecond = new double[2];
    public double[] appliedVolts = new double[2];
    public double[] supplyCurrentAmps = new double[2];
    public double[] statorCurrentAmps = new double[2];
    public double[] temperatureCelsius = new double[2];
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
