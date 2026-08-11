package frc.robot.subsystems.intake;


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

  default void setRollerVoltages(double rollerVolts) {}

  default void stop() {
    setRollerVoltages(0.0);
  }

  public default void setPivotPosition(Intake.Position position) {}
}
