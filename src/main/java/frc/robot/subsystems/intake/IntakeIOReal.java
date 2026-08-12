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
  private final VoltageOut alwaysOnRequest = new VoltageOut(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0.0).withSlot(0);
  private static final Angle kPositionTolerance = Degrees.of(3);

  public IntakeIOReal(IntakeConfiguration configuration) {
    CANBus canBus = configuration.canBus().isBlank()
        ? CANBus.roboRIO()
        : new CANBus(configuration.canBus());

    intakeMotor = new TalonFX(configuration.alwaysOnMotorCanId(), canBus);
    pivotMotorL = new TalonFX(configuration.pivotLMotorCanId(), canBus);
    pivotMotorR = new TalonFX(configuration.pivotRMotorCanId(), canBus);

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
    // 1. Roller (Intake Roller) 數據更新
    inputs.rollerConnected = intakeMotor.getVersion().getStatus().isOK();
    inputs.rollerVelocityRotationsPerSecond = intakeMotor.getVelocity().getValueAsDouble();
    inputs.rollerAppliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rollerStatorCurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerTempCelsius = intakeMotor.getDeviceTemp().getValueAsDouble();

    // 2. Pivot 馬達數據更新 (讀取 Master: pivotMotorR)
    inputs.pivotConnected = pivotMotorR.getVersion().getStatus().isOK() && pivotMotorL.getVersion().getStatus().isOK();
    inputs.pivotPositionDegrees = pivotMotorR.getPosition().getValue().in(Degrees);
    inputs.pivotVelocityRotationsPerSecond = pivotMotorR.getVelocity().getValueAsDouble();
    inputs.pivotAppliedVolts = pivotMotorR.getMotorVoltage().getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotMotorR.getSupplyCurrent().getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotMotorR.getStatorCurrent().getValueAsDouble();
    inputs.pivotTempCelsius = pivotMotorR.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setRollerVoltages(double rollerVolts) {
    intakeMotor.setControl(alwaysOnRequest.withOutput(rollerVolts));
  }

  public void setPivotPosition(Angle position) {
    pivotMotorR.setControl(
        pivotMotionMagicRequest
            .withPosition(position));
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotorR.setControl(pivotVoltageRequest.withOutput(volts));
  }

  @Override
  public void resetPivotEncoder(Angle angle) {
    pivotMotorR.setPosition(angle);
    pivotMotorL.setPosition(angle);
  }

  private boolean isPositionWithinTolerance() {
    final Angle currentPosition = pivotMotorR.getPosition().getValue();
    final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, kPositionTolerance);
  }
}