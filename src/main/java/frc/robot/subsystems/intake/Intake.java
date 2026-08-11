package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeConfiguration;
import org.littletonrobotics.junction.Logger;

public final class Intake extends SubsystemBase {

  public enum Position {
    HOMED(110),
    INTAKE(-4),
    AGITATE(20);

    private final double degrees;

    private Position(double degrees) {
      this.degrees = degrees;
    }

    public Angle angle() {
      return Degrees.of(degrees);
    }
  }

  public enum WantedState {
    OFF(Position.INTAKE, 0.0),
    INTAKE(Position.INTAKE, 12.0), 
    AGITATE(Position.AGITATE, 6.0), 
    HOME(Position.HOMED, 0.0), 
    TEST_INTAKE(Position.INTAKE, 0.0);

    private final Position targetPosition;
    private final double defaultRollerVolts;

    WantedState(Position targetPosition, double defaultRollerVolts) {
      this.targetPosition = targetPosition;
      this.defaultRollerVolts = defaultRollerVolts;
    }

    public Position getTargetPosition() {
      return targetPosition;
    }

    public double getDefaultRollerVolts() {
      return defaultRollerVolts;
    }
  }

  private final IntakeIO io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs();
  private final IntakeConfiguration configuration;

  private WantedState wantedState = WantedState.OFF;
  private Position customPivotPosition = Position.HOMED;
  private boolean overridePivotPosition = false;
  private double intakeTestVoltage = 0.0;

  public Intake(IntakeIO io, IntakeConfiguration configuration) {
    this.io = io;
    this.configuration = configuration;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    double rollerVolts = 0.0;
    Position targetPivotPosition = overridePivotPosition ? customPivotPosition : wantedState.getTargetPosition();

    switch (wantedState) {
      case INTAKE:
        rollerVolts = configuration.alwaysOnVolts();
        break;

      case AGITATE:
        rollerVolts = configuration.alwaysOnVolts() * 0.5;
        break;

      case TEST_INTAKE:
        rollerVolts = intakeTestVoltage;
        break;

      case OFF:
      case HOME:
      default:
        rollerVolts = 0.0;
        break;
    }

    // Update the io
    // io.setRollerVoltages(rollerVolts);
    // io.setPivotPosition(targetPivotPosition);

    // AdvantageKit
    Logger.recordOutput("Intake/WantedState", wantedState.name());
    Logger.recordOutput("Intake/TargetPivotPosition", targetPivotPosition.name());
    Logger.recordOutput("Intake/CommandedAlwaysOnVolts", rollerVolts);
    Logger.recordOutput("Intake/Connected", inputs.connected);
    Logger.recordOutput("Intake/VelocityRPS", inputs.velocityRotationsPerSecond);
    Logger.recordOutput("Intake/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Intake/SupplyCurrentAmps", inputs.supplyCurrentAmps);
    Logger.recordOutput("Intake/StatorCurrentAmps", inputs.statorCurrentAmps);
    Logger.recordOutput("Intake/TemperatureCelsius", inputs.temperatureCelsius);
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
    this.overridePivotPosition = false;
  }

  public void setCustomPivotPosition(Position position) {
    this.customPivotPosition = position;
    this.overridePivotPosition = true;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public void setIntakeTestVoltage(double voltage) {
    this.intakeTestVoltage = voltage;
  }
}