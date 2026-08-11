package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;

public interface IntakeIO {
  public static class Inputs {
    // 1. Roller (Intake Roller) 馬達數據
    public boolean rollerConnected = false;
    public double rollerVelocityRotationsPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;

    // 2. Pivot (機構轉軸) 馬達數據
    public boolean pivotConnected = false;
    public double pivotPositionDegrees = 0.0;
    public double pivotVelocityRotationsPerSecond = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotTempCelsius = 0.0;
  }

  public default void updateInputs(Inputs inputs) {}
  public default void setRollerVoltages(double volts) {}
  public default void setPivotPosition(Angle angle) {}
  public default void setPivotVoltage(double volts) {}
  public default void resetPivotEncoder(Angle angle) {}
}