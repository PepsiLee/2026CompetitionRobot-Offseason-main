package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.logging.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.KrakenX60;
import frc.robot.config.ShooterConfiguration;

public final class ShooterIOReal implements ShooterIO {
  private static final AngularVelocity kVelocityTolerance = RPM.of(100);
  private final ShooterConfiguration configuration;
  private final TalonFX shootOne, shootTwo;
  private final List<TalonFX> motors;
  private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public ShooterIOReal(ShooterConfiguration configuration) {
    CANBus canBus = configuration.canBus().isBlank()
        ? CANBus.roboRIO()
        : new CANBus(configuration.canBus());

    shootOne = new TalonFX(configuration.shootOneCanId(), canBus);
    shootTwo = new TalonFX(configuration.shootTwoCanId(), canBus);
    motors = List.of(shootOne, shootTwo);

    configureMotor(shootOne, InvertedValue.Clockwise_Positive);
    configureMotor(shootTwo, InvertedValue.CounterClockwise_Positive);
    this.configuration = configuration;
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
            new Slot0Configs()
                .withKP(0.5)
                .withKI(0)
                .withKD(0)
                .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(250) // rps^2
                .withMotionMagicJerk(1500) // rps^3
        );

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    int i = 0;
    for (final TalonFX motor : motors) {
      inputs.connected[i] = motor.getVersion().getStatus().isOK();
      inputs.velocityRotationsPerSecond[i] = motor.getVelocity().getValueAsDouble();
      inputs.appliedVolts[i] = motor.getMotorVoltage().getValueAsDouble();
      inputs.supplyCurrentAmps[i] = motor.getSupplyCurrent().getValueAsDouble();
      inputs.statorCurrentAmps[i] = motor.getStatorCurrent().getValueAsDouble();
      inputs.temperatureCelsius[i] = motor.getDeviceTemp().getValueAsDouble();
      i++;
    }
  }

  public void setRPM(double rpm) {
    rpm = rpm * configuration.gearRatio();
    for (final TalonFX motor : motors) {
      motor.setControl(
          motionMagicVelocityVoltage.withVelocity(RPM.of(rpm)));
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
      final boolean isInVelocityMode = motor.getAppliedControl().equals(motionMagicVelocityVoltage);
      final AngularVelocity currentVelocity = motor.getVelocity().getValue();
      final AngularVelocity targetVelocity = motionMagicVelocityVoltage.getVelocityMeasure();
      SmartDashboard.putBoolean("Shooter/isInVelocityMode", isInVelocityMode);
      SmartDashboard.putString("Shooter/currentVelocity", motor.getVelocity().getValue().toString());
      SmartDashboard.putString("Shooter/targetVelocity", motionMagicVelocityVoltage.getVelocityMeasure().toString());
      return isInVelocityMode && currentVelocity.isNear(targetVelocity, kVelocityTolerance);
    });
  }
}
