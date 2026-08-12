package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants.KrakenX60;
import frc.robot.config.FeederConfiguration;

public class FeederIOReal implements FeederIO {
    private final TalonFX motor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

    public FeederIOReal(FeederConfiguration feederConfiguration) {

        motor = new TalonFX(feederConfiguration.feederMotorCanId(), feederConfiguration.canBus());

        final TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(120))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(50))
                                .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                        new Slot0Configs()
                                .withKP(1)
                                .withKI(0)
                                .withKD(0)
                                .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)))
                .withMotionMagic(
                        new MotionMagicConfigs()
                                .withMotionMagicAcceleration(250) // rps^2
                                .withMotionMagicJerk(1500) // rps^3
                );
        ;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.connected = motor.isConnected();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void set(double rpm) {
        motor.setControl(motionMagicVelocityVoltage.withVelocity(RPM.of(rpm)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}