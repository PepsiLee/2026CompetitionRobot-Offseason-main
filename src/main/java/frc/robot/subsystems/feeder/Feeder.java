package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.FeederConfiguration;
import org.littletonrobotics.junction.Logger;

public final class Feeder extends SubsystemBase {
  public enum WantedState {
    OFF,
    FEED_SHOOTER,  // 推彈進 Shooter 進行射擊
    EJECT,          // 反轉
    TEST_SHOOTER    // 測試 Shooter
  }

  private final FeederIO io;
  private final FeederIO.Inputs inputs = new FeederIO.Inputs();
  private final FeederConfiguration configuration;
  private double voltage = 0.0;

  private WantedState wantedState = WantedState.OFF;

  public Feeder(FeederIO io, FeederConfiguration configuration) {
    this.io = io;
    this.configuration = configuration;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    double commandedVolts = 0.0;

    switch (wantedState) {
      case TEST_SHOOTER -> commandedVolts = voltage;
      case FEED_SHOOTER -> commandedVolts = configuration.feedToShooterVolts();
      case EJECT -> commandedVolts = configuration.ejectVolts();
      case OFF -> commandedVolts = 0.0;
    }

    if (wantedState == WantedState.OFF) {
      io.stop();
    } else {
      io.setVoltage(commandedVolts); // 6000 rpm / 12 volts / 60 sec
    }

    // Logger 紀錄
    Logger.recordOutput("Feeder/WantedState", wantedState);
    Logger.recordOutput("Feeder/CommandedVolts", commandedVolts);
    Logger.recordOutput("Feeder/Connected", inputs.connected);
    Logger.recordOutput("Feeder/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Feeder/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Feeder/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Feeder/TemperatureCelsius", inputs.temperatureCelsius);
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }
}