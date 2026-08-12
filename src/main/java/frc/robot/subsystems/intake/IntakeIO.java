package frc.robot.subsystems.intake;

/** Hardware boundary for the two independent intake motors. */
public interface IntakeIO {
  class Inputs {
    public boolean[] connected = new boolean[3];
    public double[] velocityRotationsPerSecond = new double[3];
    public double[] appliedVolts = new double[3];
    public double[] supplyCurrentAmps = new double[3];
    public double[] statorCurrentAmps = new double[3];
    public double[] temperatureCelsius = new double[3];
  }

  default void updateInputs(Inputs inputs) {}

  default void setVoltages(double alwaysOnVolts, double circleMotorVolts) {}

  default void stop() {
    setVoltages(0.0, 0.0);
  }
}