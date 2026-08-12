package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.config.IntakeConfiguration;

public final class IntakeIOReal implements IntakeIO {

  private final TalonFX intakeMotor;
  // pivotMotorR 為 Master，pivotMotorL 為 Follower
  private final TalonFX pivotMotorL, pivotMotorR;
  private final TalonFX[] motors;
  private final VoltageOut alwaysOnRequest = new VoltageOut(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0);

  public IntakeIOReal(IntakeConfiguration configuration) {
    CANBus canBus = configuration.canBus().isBlank()
        ? CANBus.roboRIO()
        : new CANBus(configuration.canBus());

    intakeMotor = new TalonFX(configuration.alwaysOnMotorCanId(), canBus);
    pivotMotorL = new TalonFX(configuration.pivotLMotorCanId(), canBus);
    pivotMotorR = new TalonFX(configuration.pivotRMotorCanId(), canBus);
    motors = new TalonFX[] { intakeMotor, pivotMotorL, pivotMotorR };

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    intakeMotor.getConfigurator().apply(motorConfiguration);

    configurePivotMotor();
  }

  private void configurePivotMotor() {
    // 1. 設定 Master (pivotMotorR) 的配置
    final TalonFXConfiguration configR = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(40 / 14.0))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1.5))
                .withMotionMagicAcceleration(RotationsPerSecond.of(1.5).per(Second)))
        .withSlot0(
            new Slot0Configs()
                .withKP(300)
                .withKI(0)
                .withKD(0)
                .withGravityType(GravityTypeValue.Arm_Cosine));

    pivotMotorR.getConfigurator().apply(configR);

    // 2. 設定 Follower (pivotMotorL) 的基本配置
    final TalonFXConfiguration configL = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true));

    pivotMotorL.getConfigurator().apply(configL);

    // 3. 設定 pivotMotorL 跟隨 pivotMotorR
    pivotMotorL.setControl(new Follower(pivotMotorR.getDeviceID(), MotorAlignmentValue.Opposed));
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
  public void setVoltages(double rollerVolts, double pivotMotor) {
    intakeMotor.setControl(alwaysOnRequest.withOutput(rollerVolts));
    pivotMotorR.setControl(
        pivotVoltageRequest
            .withOutput(pivotMotor));
  }
}