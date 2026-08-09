package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.ShooterConfiguration;

/** Phoenix 6 VoltageOut control for three independent shooter motors. */
public final class ShooterIOReal implements ShooterIO {
  private final TalonFX shootOne;
  private final TalonFX shootTwo;
  private final TalonFX shootUp;
  private final TalonFX[] motors;
  private final VoltageOut shootOneRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut shootTwoRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut shootUpRequest = new VoltageOut(0.0).withEnableFOC(true);

  public ShooterIOReal(ShooterConfiguration configuration) {
    CANBus canBus =
        configuration.canBus().isBlank()
            ? CANBus.roboRIO()
            : new CANBus(configuration.canBus());
    shootOne = new TalonFX(configuration.shootOneCanId(), canBus);
    shootTwo = new TalonFX(configuration.shootTwoCanId(), canBus);
    shootUp = new TalonFX(configuration.shootUpCanId(), canBus);
    motors = new TalonFX[] {shootOne, shootTwo, shootUp};

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
  public void setVoltages(double shootOneVolts, double shootTwoVolts, double shootUpVolts) {
    shootOne.setControl(shootOneRequest.withOutput(shootOneVolts));
    shootTwo.setControl(shootTwoRequest.withOutput(shootTwoVolts));
    shootUp.setControl(shootUpRequest.withOutput(shootUpVolts));
  }
}
