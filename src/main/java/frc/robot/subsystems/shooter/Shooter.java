package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ShooterConfiguration;
import org.littletonrobotics.junction.Logger;

/** Fixed-angle shooter with three independent, signed voltage outputs. */
public final class Shooter extends SubsystemBase {
  public enum WantedState {
    OFF,
    RUN
  }

  private final ShooterIO io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs();
  private final ShooterConfiguration configuration;
  private WantedState wantedState = WantedState.OFF;
  private boolean wasRunning;
  private double runStartTimestampSeconds;

  public Shooter(ShooterIO io, ShooterConfiguration configuration) {
    this.io = io;
    this.configuration = configuration;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double shootUpVolts = 0.0;
    double runElapsedSeconds = 0.0;
    if (wantedState == WantedState.RUN) {
      if (!wasRunning) {
        runStartTimestampSeconds = Timer.getFPGATimestamp();
      }
      runElapsedSeconds = Timer.getFPGATimestamp() - runStartTimestampSeconds;
      if (runElapsedSeconds >= configuration.shootUpDelaySeconds()) {
        shootUpVolts = configuration.shootUpVolts();
      }
      io.setVoltages(
          configuration.shootOneVolts(),
          configuration.shootTwoVolts(),
          shootUpVolts);
      wasRunning = true;
    } else {
      io.stop();
      wasRunning = false;
    }

    Logger.recordOutput("Shooter/WantedState", wantedState);
    Logger.recordOutput(
        "Shooter/CommandedVolts",
        wantedState == WantedState.RUN
            ? new double[] {
              configuration.shootOneVolts(),
              configuration.shootTwoVolts(),
              shootUpVolts
            }
            : new double[3]);
    Logger.recordOutput("Shooter/RunElapsedSeconds", runElapsedSeconds);
    Logger.recordOutput(
        "Shooter/ShootUpEnabled",
        wantedState == WantedState.RUN
            && runElapsedSeconds >= configuration.shootUpDelaySeconds());
    Logger.recordOutput("Shooter/Connected", inputs.connected);
    Logger.recordOutput("Shooter/VelocityRPS", inputs.velocityRotationsPerSecond);
    Logger.recordOutput("Shooter/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Shooter/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Shooter/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Shooter/TemperatureCelsius", inputs.temperatureCelsius);
    Logger.recordOutput("Shooter/AllConnected", allMotorsConnected());
  }

  public void setWantedState(WantedState state) {
    wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  private boolean allMotorsConnected() {
    for (boolean connected : inputs.connected) {
      if (!connected) {
        return false;
      }
    }
    return true;
  }
}
