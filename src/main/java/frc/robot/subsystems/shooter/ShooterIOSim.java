package frc.robot.subsystems.shooter;

/** Deterministic first-order voltage simulation for all three shooter motors. */
public final class ShooterIOSim implements ShooterIO {
  private static final double LOOP_PERIOD_SECONDS = 0.020;
  private static final double TIME_CONSTANT_SECONDS = 0.25;

  private final double[] commandedVolts = new double[3];
  private final double[] currentRps = new double[3];

  @Override
  public void updateInputs(Inputs inputs) {
    double alpha = 1.0 - Math.exp(-LOOP_PERIOD_SECONDS / TIME_CONSTANT_SECONDS);
    for (int i = 0; i < 3; i++) {
      double targetRps = commandedVolts[i] * 8.0;
      currentRps[i] += (targetRps - currentRps[i]) * alpha;
      inputs.connected[i] = true;
      inputs.velocityRotationsPerSecond[i] = currentRps[i];
      inputs.appliedVolts[i] = commandedVolts[i];
      inputs.supplyCurrentAmps[i] = Math.abs(targetRps - currentRps[i]) / 4.0;
      inputs.statorCurrentAmps[i] = inputs.supplyCurrentAmps[i] * 1.5;
      inputs.temperatureCelsius[i] = 25.0;
    }
  }

  @Override
  public void setVoltages(double shootOneVolts, double shootTwoVolts, double shootUpVolts) {
    commandedVolts[0] = shootOneVolts;
    commandedVolts[1] = shootTwoVolts;
    commandedVolts[2] = shootUpVolts;
  }
}
