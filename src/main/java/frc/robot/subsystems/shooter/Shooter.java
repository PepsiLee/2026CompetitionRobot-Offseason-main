package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.config.ShooterConfiguration;

import org.littletonrobotics.junction.Logger;

public final class Shooter extends SubsystemBase {
  public enum WantedState {
    OFF,
    SHOOTING
  }

  private final ShooterIO io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs();
  private final double maximumMechanismRpm;
  private double rpm = 0;
  private WantedState wantedState = WantedState.OFF;

  public Shooter(ShooterIO io, ShooterConfiguration configuration) {
    this.io = io;
    maximumMechanismRpm =
        KrakenX60.kFreeSpeed.in(RPM) / Math.abs(configuration.gearRatio());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (wantedState == WantedState.SHOOTING) {
      io.setRPM(rpm);
    } else {
      io.stop();
    }

    Logger.recordOutput("Shooter/WantedState", wantedState);
    Logger.recordOutput("Shooter/CommandedRPM", rpm);
    Logger.recordOutput("Shooter/Connected", inputs.connected);
    Logger.recordOutput("Shooter/VelocityRPS", inputs.velocityRotationsPerSecond);
    Logger.recordOutput("Shooter/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Shooter/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Shooter/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Shooter/TemperatureCelsius", inputs.temperatureCelsius);
    Logger.recordOutput("Shooter/AllConnected", allMotorsConnected());
  }

  public boolean isReady(){
    return io.isVelocityWithinTolerance();
  }

  public void setRPM(double rpm){
    this.rpm = rpm;
  }

  public double getMaximumMechanismRpm() {
    return maximumMechanismRpm;
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
