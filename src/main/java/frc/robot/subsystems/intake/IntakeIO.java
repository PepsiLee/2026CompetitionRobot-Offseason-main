package frc.robot.subsystems.intake;

/** Hardware boundary for the two independent intake motors. */
public interface IntakeIO {
  class Inputs {
    public boolean[] connected = new boolean[2];
    public double[] velocityRotationsPerSecond = new double[2];
    public double[] appliedVolts = new double[2];
    public double[] supplyCurrentAmps = new double[2];
    public double[] statorCurrentAmps = new double[2];
    public double[] temperatureCelsius = new double[2];
  }

  default void updateInputs(Inputs inputs) {}

  default void setVoltages(double alwaysOnVolts, double circleMotorVolts) {}

  default void stop() {
    setVoltages(0.0, 0.0);
  }
}
