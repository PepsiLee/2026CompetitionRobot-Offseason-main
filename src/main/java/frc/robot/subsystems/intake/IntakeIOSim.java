package frc.robot.subsystems.intake;

/** Lightweight deterministic voltage simulation for both intake motors. */
public final class IntakeIOSim implements IntakeIO {
  private final double[] commandedVolts = new double[2];

  @Override
  public void updateInputs(Inputs inputs) {
    for (int i = 0; i < 2; i++) {
      inputs.connected[i] = true;
      inputs.velocityRotationsPerSecond[i] = commandedVolts[i] * 7.5;
      inputs.appliedVolts[i] = commandedVolts[i];
      inputs.supplyCurrentAmps[i] = Math.abs(commandedVolts[i]) * 1.25;
      inputs.statorCurrentAmps[i] = Math.abs(commandedVolts[i]) * 2.0;
      inputs.temperatureCelsius[i] = 25.0;
    }
  }

  @Override
  public void setVoltages(double alwaysOnVolts, double circleMotorVolts) {
    commandedVolts[0] = alwaysOnVolts;
    commandedVolts[1] = circleMotorVolts;
  }
}
