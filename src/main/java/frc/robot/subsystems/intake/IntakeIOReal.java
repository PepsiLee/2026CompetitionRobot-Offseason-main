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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.config.IntakeConfiguration;

public final class IntakeIOReal implements IntakeIO {

  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;
  private final VoltageOut alwaysOnRequest = new VoltageOut(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0.0).withSlot(0);
  private static final Angle kPositionTolerance = Degrees.of(3);

  public IntakeIOReal(IntakeConfiguration configuration) {
    CANBus canBus = configuration.canBus().isBlank()
        ? CANBus.roboRIO()
        : new CANBus(configuration.canBus());

    intakeMotor = new TalonFX(configuration.alwaysOnMotorCanId(), canBus);
    pivotMotor = new TalonFX(configuration.circleMotorCanId(), canBus);
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

  // deploy the intake is positive power
  private void configurePivotMotor() {
    final TalonFXConfiguration config = new TalonFXConfiguration()
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
    pivotMotor.getConfigurator().apply(config);
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

    // 2. Pivot 馬達數據更新
    inputs.pivotConnected = pivotMotor.getVersion().getStatus().isOK();
    inputs.pivotPositionDegrees = pivotMotor.getPosition().getValue().in(Degrees);
    inputs.pivotVelocityRotationsPerSecond = pivotMotor.getVelocity().getValueAsDouble();
    inputs.pivotAppliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.pivotTempCelsius = pivotMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setRollerVoltages(double rollerVolts) {
    intakeMotor.setControl(alwaysOnRequest.withOutput(rollerVolts));
  }

  public void setPivotPosition(Angle position) {
    pivotMotor.setControl(
        pivotMotionMagicRequest
            .withPosition(position));
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(volts));
  }

  @Override
  public void resetPivotEncoder(Angle angle) {
    pivotMotor.setPosition(angle);
  }
  
  private boolean isPositionWithinTolerance() {
    final Angle currentPosition = pivotMotor.getPosition().getValue();
    final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
    return currentPosition.isNear(targetPosition, kPositionTolerance);
  }
}
