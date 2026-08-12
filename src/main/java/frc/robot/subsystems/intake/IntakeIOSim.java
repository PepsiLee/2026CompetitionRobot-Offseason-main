package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;

/** Lightweight deterministic voltage simulation for intake subsystem. */
public final class IntakeIOSim implements IntakeIO {

  private double rollerCommandedVolts = 0.0;
  private double pivotCommandedVolts = 0.0;

  // 模擬 Pivot 的位置（單位：Degrees）
  private double pivotPositionDegrees = 0.0;

  @Override
  public void updateInputs(Inputs inputs) {
    // 確保陣列已被初始化且長度足夠 (0: Intake, 1: PivotL, 2: PivotR)
    if (inputs.connected == null || inputs.connected.length < 3) {
      inputs.connected = new boolean[3];
      inputs.velocityRotationsPerSecond = new double[3];
      inputs.appliedVolts = new double[3];
      inputs.supplyCurrentAmps = new double[3];
      inputs.statorCurrentAmps = new double[3];
      inputs.temperatureCelsius = new double[3];
    }

    // 1. Roller 馬達模擬數據 (Index 0)
    inputs.connected[0] = true;
    inputs.velocityRotationsPerSecond[0] = rollerCommandedVolts * 7.5;
    inputs.appliedVolts[0] = rollerCommandedVolts;
    inputs.supplyCurrentAmps[0] = Math.abs(rollerCommandedVolts) * 1.25;
    inputs.statorCurrentAmps[0] = Math.abs(rollerCommandedVolts) * 2.0;
    inputs.temperatureCelsius[0] = 25.0;

    // 2. Pivot 運動學與電流計算
    pivotPositionDegrees += pivotCommandedVolts * 0.1;

    double pivotSupplyCurrent;
    if (Math.abs(pivotCommandedVolts) > 1.0 && Math.abs(pivotCommandedVolts) < 2.0) {
      pivotSupplyCurrent = 8.0; // 模擬撞到硬擋塊觸發過載電流 (> 6.0A)
    } else {
      pivotSupplyCurrent = Math.abs(pivotCommandedVolts) * 1.5;
    }
    double pivotStatorCurrent = Math.abs(pivotCommandedVolts) * 2.5;
    double pivotVelRps = pivotCommandedVolts * 2.0;

    // 3. Pivot 左馬達模擬數據 (Index 1 - Follower)
    inputs.connected[1] = true;
    inputs.velocityRotationsPerSecond[1] = pivotVelRps;
    inputs.appliedVolts[1] = pivotCommandedVolts;
    inputs.supplyCurrentAmps[1] = pivotSupplyCurrent;
    inputs.statorCurrentAmps[1] = pivotStatorCurrent;
    inputs.temperatureCelsius[1] = 25.0;

    // 4. Pivot 右馬達模擬數據 (Index 2 - Master)
    inputs.connected[2] = true;
    inputs.velocityRotationsPerSecond[2] = pivotVelRps;
    inputs.appliedVolts[2] = pivotCommandedVolts;
    inputs.supplyCurrentAmps[2] = pivotSupplyCurrent;
    inputs.statorCurrentAmps[2] = pivotStatorCurrent;
    inputs.temperatureCelsius[2] = 25.0;
  }

  @Override
  public void setVoltages(double rollerVolts, double pivotVolts) {
    this.rollerCommandedVolts = rollerVolts;
    this.pivotCommandedVolts = pivotVolts;
  }

}