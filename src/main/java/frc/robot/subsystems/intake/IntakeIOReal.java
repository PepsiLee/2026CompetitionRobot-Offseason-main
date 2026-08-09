package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.IntakeConfiguration;

/** Phoenix 6 VoltageOut implementation for CAN 9 and CAN 10. */
public final class IntakeIOReal implements IntakeIO {
  private final TalonFX alwaysOnMotor;
  private final TalonFX circleMotor;
  private final TalonFX[] motors;
  private final VoltageOut alwaysOnRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut circleRequest = new VoltageOut(0.0).withEnableFOC(true);

  public IntakeIOReal(IntakeConfiguration configuration) {
    CANBus canBus =
        configuration.canBus().isBlank()
            ? CANBus.roboRIO()
            : new CANBus(configuration.canBus());
    alwaysOnMotor = new TalonFX(configuration.alwaysOnMotorCanId(), canBus);
    circleMotor = new TalonFX(configuration.circleMotorCanId(), canBus);
    motors = new TalonFX[] {alwaysOnMotor, circleMotor};
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    for (TalonFX motor : motors) {
      motor.getConfigurator().apply(motorConfiguration);
    }
  }

  @Override
  public void updateInputs(Inputs inputs) {
    for (int i = 0; i < motors.length; i++) {
      TalonFX motor = motors[i];
      inputs.connected[i] = motor.getVersion().getStatus().isOK();
      inputs.velocityRotationsPerSecond[i] = motor.getVelocity().getValueAsDouble();
      inputs.appliedVolts[i] = motor.getMotorVoltage().getValueAsDouble();
      inputs.supplyCurrentAmps[i] = motor.getSupplyCurrent().getValueAsDouble();
      inputs.statorCurrentAmps[i] = motor.getStatorCurrent().getValueAsDouble();
      inputs.temperatureCelsius[i] = motor.getDeviceTemp().getValueAsDouble();
    }
  }

  @Override
  public void setVoltages(double alwaysOnVolts, double circleMotorVolts) {
    alwaysOnMotor.setControl(alwaysOnRequest.withOutput(alwaysOnVolts));
    circleMotor.setControl(circleRequest.withOutput(circleMotorVolts));
  }
}
