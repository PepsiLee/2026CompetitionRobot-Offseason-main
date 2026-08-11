package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeConfiguration;
import org.littletonrobotics.junction.Logger;

/**
 * Two voltage-controlled intake motors; gravity performs all downstream
 * transport.
 */
public final class Intake extends SubsystemBase {
  public enum WantedState {
    OFF,
    INTAKE,
    STOPPED,
    TEST_INTAKE
  }

  private final IntakeIO io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs();
  private final IntakeConfiguration configuration;
  private WantedState wantedState = WantedState.OFF;
  private double intakeTestVoltage = 0.0;

  public Intake(IntakeIO io, IntakeConfiguration configuration) {
    this.io = io;
    this.configuration = configuration;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double alwaysOnVolts = 0.0;
    double circleMotorVolts = 0.0;

    switch (wantedState) {
      case INTAKE:
        alwaysOnVolts = configuration.alwaysOnVolts();
        circleMotorVolts = configuration.circleMotorVolts();
        break;

      case OFF:
      case TEST_INTAKE:
        alwaysOnVolts = intakeTestVoltage;
        circleMotorVolts = 0.0;
        break;

      case STOPPED:
      default:
        alwaysOnVolts = 0.0;
        circleMotorVolts = 0.0;
        break;
    }

    io.setVoltages(alwaysOnVolts, circleMotorVolts);

    Logger.recordOutput("Intake/WantedState", wantedState);
    Logger.recordOutput("Intake/CommandedAlwaysOnVolts", alwaysOnVolts);
    Logger.recordOutput("Intake/CommandedCircleMotorVolts", circleMotorVolts);
    Logger.recordOutput("Intake/Connected", inputs.connected);
    Logger.recordOutput("Intake/VelocityRPS", inputs.velocityRotationsPerSecond);
    Logger.recordOutput("Intake/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Intake/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Intake/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Intake/TemperatureCelsius", inputs.temperatureCelsius);
  }

  public void setWantedState(WantedState state) {
    wantedState = state;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public void setIntakeTestVoltage(double voltage) {
    intakeTestVoltage = voltage;
  }
}
