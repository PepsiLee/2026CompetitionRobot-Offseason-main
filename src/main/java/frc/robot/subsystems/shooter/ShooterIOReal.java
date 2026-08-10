package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.KrakenX60;
import frc.robot.config.ShooterConfiguration;

public final class ShooterIOReal implements ShooterIO {
  // TODO: Move to the constant File
  private static final AngularVelocity kVelocityTolerance = RPM.of(100);
  private final TalonFX shootOne, shootTwo;
  private final List<TalonFX> motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public ShooterIOReal(ShooterConfiguration configuration) {
    // TODO: Optimizer this reduntant writing
    CANBus canBus = configuration.canBus().isBlank()
        ? CANBus.roboRIO()
        : new CANBus(configuration.canBus());

    shootOne = new TalonFX(configuration.shootOneCanId(), canBus);
    shootTwo = new TalonFX(configuration.shootTwoCanId(), canBus);
    motors = List.of(shootOne, shootTwo);
    ;

    // TODO: Check the motor's Direction
    configureMotor(shootOne, InvertedValue.CounterClockwise_Positive);
    configureMotor(shootTwo, InvertedValue.Clockwise_Positive);
  }

  private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
    final TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(invertDirection)
                .withNeutralMode(NeutralModeValue.Coast))
        .withVoltage(
            new VoltageConfigs()
                .withPeakReverseVoltage(Volts.of(0)))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true))
        .withSlot0(
            // TODO: Tune the PID value
            new Slot0Configs()
                .withKP(0.5)
                .withKI(0)
                .withKD(0)
                .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
        );

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    for (int i = 0; i < motors.size(); i++) {
      TalonFX motor = motors.get(i);
      inputs.connected[i] = motor.getVersion().getStatus().isOK();
      inputs.velocityRotationsPerSecond[i] = motor.getVelocity().getValueAsDouble();
      inputs.appliedVolts[i] = motor.getMotorVoltage().getValueAsDouble();
      inputs.supplyCurrentAmps[i] = motor.getSupplyCurrent().getValueAsDouble();
      inputs.statorCurrentAmps[i] = motor.getStatorCurrent().getValueAsDouble();
      inputs.temperatureCelsius[i] = motor.getDeviceTemp().getValueAsDouble();
    }
  }

  public void setRPM(double rpm) {
    for (final TalonFX motor : motors) {
      motor.setControl(
          velocityRequest
              .withVelocity(RPM.of(rpm)));
    }
  }

  public void setPercentOutput(double percentOutput) {
    for (final TalonFX motor : motors) {
      motor.setControl(
          voltageRequest
              .withOutput(Volts.of(percentOutput * 12.0)));
    }
  }

  public void stop() {
    setPercentOutput(0.0);
  }

  public boolean isVelocityWithinTolerance() {
    return motors.stream().allMatch(motor -> {
      final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
      final AngularVelocity currentVelocity = motor.getVelocity().getValue();
      final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
      return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
    });
  }
}
