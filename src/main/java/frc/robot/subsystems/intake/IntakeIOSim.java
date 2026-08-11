package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;

/** Lightweight deterministic voltage simulation for intake subsystem. */
public final class IntakeIOSim implements IntakeIO {

  private double rollerCommandedVolts = 0.0;
  private double pivotCommandedVolts = 0.0;

  // 模擬 Pivot 的位置（單位：Degrees）
  private double pivotPositionDegrees = Intake.Position.HOMED.angle().in(edu.wpi.first.units.Units.Degrees);

  @Override
  public void updateInputs(Inputs inputs) {
    // 1. Roller 模擬數據
    inputs.rollerConnected = true;
    inputs.rollerVelocityRotationsPerSecond = rollerCommandedVolts * 7.5;
    inputs.rollerAppliedVolts = rollerCommandedVolts;
    inputs.rollerSupplyCurrentAmps = Math.abs(rollerCommandedVolts) * 1.25;
    inputs.rollerStatorCurrentAmps = Math.abs(rollerCommandedVolts) * 2.0;
    inputs.rollerTempCelsius = 25.0;

    // 2. Pivot 模擬數據 (簡單模擬電壓推動位置變化)
    inputs.pivotConnected = true;
    pivotPositionDegrees += pivotCommandedVolts * 0.1; // 模擬電壓帶動機構移動
    inputs.pivotPositionDegrees = pivotPositionDegrees;
    inputs.pivotVelocityRotationsPerSecond = pivotCommandedVolts * 2.0;
    inputs.pivotAppliedVolts = pivotCommandedVolts;
    
    // 如果給予固定電壓（例如歸零時的 1.2V），模擬擠壓擋塊時電流升高
    if (Math.abs(pivotCommandedVolts) > 1.0 && Math.abs(pivotCommandedVolts) < 2.0) {
      inputs.pivotSupplyCurrentAmps = 8.0; // 模擬撞到硬擋塊觸發過載電流 (> 6.0A)
    } else {
      inputs.pivotSupplyCurrentAmps = Math.abs(pivotCommandedVolts) * 1.5;
    }
    
    inputs.pivotStatorCurrentAmps = Math.abs(pivotCommandedVolts) * 2.5;
    inputs.pivotTempCelsius = 25.0;
  }

  @Override
  public void setRollerVoltages(double rollerVolts) {
    this.rollerCommandedVolts = rollerVolts;
  }

  @Override
  public void setPivotVoltage(double volts) {
    this.pivotCommandedVolts = volts;
  }

  @Override
  public void setPivotPosition(Angle position) {
    this.pivotPositionDegrees = position.in(edu.wpi.first.units.Units.Degrees);
    this.pivotCommandedVolts = 0.0;
  }

  @Override
  public void resetPivotEncoder(Angle angle) {
    this.pivotPositionDegrees = angle.in(edu.wpi.first.units.Units.Degrees);
  }
}