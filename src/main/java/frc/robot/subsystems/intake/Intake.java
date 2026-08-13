package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeConfiguration;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

/**
 * Two voltage-controlled intake motors; gravity performs all downstream
 * transport.
 */
public final class Intake extends SubsystemBase {
  public enum WantedState {
    ONLY_ROLLER,
    INTAKE,
    STOPPED,
    TEST_INTAKE,
    DEPLOY
  }

  private final IntakeIO io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs();
  private final IntakeConfiguration configuration;
  private WantedState wantedState = WantedState.STOPPED;
  private double intakeTestVoltage = 0.0;
  private boolean isDeploy = false;

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

      case ONLY_ROLLER:
        alwaysOnVolts = configuration.alwaysOnVolts();
        circleMotorVolts = 0.0;
        break;

      case TEST_INTAKE:
        alwaysOnVolts = intakeTestVoltage;
        circleMotorVolts = 0.0;
        break;

      case DEPLOY:
        alwaysOnVolts = 0.0;
        circleMotorVolts = 10.0;//intake開機下去的
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
    Logger.recordOutput("Intake/isDeployed", isDeploy);
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

  public Command deployIntake() {
    return Commands.sequence(
        runOnce(() -> setWantedState(WantedState.DEPLOY)),
        Commands.waitTime(Seconds.of(0.4)),
        runOnce(() -> {
          isDeploy = true;
          setWantedState(WantedState.STOPPED);
        }))
        .unless(() -> isDeploy)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}