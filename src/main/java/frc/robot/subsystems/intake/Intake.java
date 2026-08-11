package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeConfiguration;
import org.littletonrobotics.junction.Logger;

public final class Intake extends SubsystemBase {

  public enum Position {
    // TODO: Need Test
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
    STOP(Position.INTAKE, 0.0), // 停止
    INTAKE(Position.INTAKE, 12.0), // 吃球狀態
    AGITATE(Position.AGITATE, 6.0), // 攪球狀態
    HOME(Position.HOMED, 0.0), // 歸位狀態（透過 PID 移動到 HOMED 角度）
    HOMING(Position.HOMED, 0.0), // 歸零中（固定電壓撞牆尋找基準點）
    TEST_INTAKE(Position.INTAKE, 0.0), // 測試吃球(包含滾軸)
    TEST_ROLLER(Position.HOMED, 0.0), // 只有滾軸
    TEST(Position.HOMED, 0.0);

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

  private WantedState wantedState = WantedState.STOP;
  private Angle customPivotAngle = Degrees.of(0);
  private boolean overridePivotPosition = false;
  private double intakeTestVoltage = 0.0;

  // 系統歸零狀態標記
  private boolean isHomed = false;

  public Intake(IntakeIO io, IntakeConfiguration configuration) {
    this.io = io;
    this.configuration = configuration;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // 1. 計算滾輪電壓 (Roller Voltage)
    double rollerVolts = calculateRollerVolts();
    io.setRollerVoltages(rollerVolts);

    // 2. 依據狀態機決定 Pivot 控制模式
    Angle targetPivotAngle = overridePivotPosition ? customPivotAngle : wantedState.getTargetPosition().angle();

    switch (wantedState) {
      case HOMING:
      case STOP:
      case TEST_ROLLER:
        // 歸零過程中，Pivot 由 homingCommand() 直接給予固定電壓 (io.setPivotVoltage)，不使用 PID
        break;

      case HOME:
      case INTAKE:
      case AGITATE:
      case TEST_INTAKE:
      case TEST:
      default:
        io.setPivotPosition(targetPivotAngle);
        break;
    }

    // AdvantageKit 數據記錄
    recordOutputs(targetPivotAngle, rollerVolts);
  }

  private double calculateRollerVolts() {
    switch (wantedState) {
      case INTAKE:
        return configuration.alwaysOnVolts();
      case AGITATE:
        return configuration.alwaysOnVolts() * 0.5;
      case TEST_ROLLER:
      case TEST_INTAKE:
      case TEST:
        return intakeTestVoltage;
      case HOMING:
      case HOME:
      case STOP:
      default:
        return 0.0;
    }
  }

  private void recordOutputs(Angle targetPivotAngle, double rollerVolts) {
    Logger.recordOutput("Intake/WantedState", wantedState.name());
    Logger.recordOutput("Intake/TargetPivotPosition", targetPivotAngle);
    Logger.recordOutput("Intake/CommandedAlwaysOnVolts", rollerVolts);
    Logger.recordOutput("Intake/IsHomed", isHomed);

    // Roller 數據 Log
    Logger.recordOutput("Intake/Roller/Connected", inputs.rollerConnected);
    Logger.recordOutput("Intake/Roller/VelocityRPS", inputs.rollerVelocityRotationsPerSecond);
    Logger.recordOutput("Intake/Roller/AppliedVolts", inputs.rollerAppliedVolts);
    Logger.recordOutput("Intake/Roller/SupplyCurrentAmps", inputs.rollerSupplyCurrentAmps);

    // Pivot 數據 Log
    Logger.recordOutput("Intake/Pivot/Connected", inputs.pivotConnected);
    Logger.recordOutput("Intake/Pivot/PositionDegrees", inputs.pivotPositionDegrees);
    Logger.recordOutput("Intake/Pivot/VelocityRPS", inputs.pivotVelocityRotationsPerSecond);
    Logger.recordOutput("Intake/Pivot/AppliedVolts", inputs.pivotAppliedVolts);
    Logger.recordOutput("Intake/Pivot/SupplyCurrentAmps", inputs.pivotSupplyCurrentAmps);
  }

  public void setWantedState(WantedState state) {
    this.wantedState = state;
    this.overridePivotPosition = false;
  }

  public void setCustomPivotPosition(Angle angle) {
    this.customPivotAngle = angle;
    this.overridePivotPosition = true;
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  public boolean isHomed() {
    return isHomed;
  }

  public void setIntakeTestVoltage(double voltage) {
    this.intakeTestVoltage = voltage;
  }

  public Command homingCommand() {
    return Commands.sequence(
        // Step 1: 切換狀態至 HOMING 並給予慢速電壓擠壓 (例: 1.2V)
        runOnce(() -> {
          setWantedState(WantedState.HOMING);
          io.setPivotVoltage(1.2);
        }),
        // Step 2: 監測電流，超過 6A 代表碰到極限擋塊
        Commands.waitUntil(() -> inputs.pivotSupplyCurrentAmps > 6.0),
        // Step 3: 關閉擠壓電壓，重置 Encoder 數值，將狀態切換至 HOME
        runOnce(() -> {
          io.setPivotVoltage(0.0);
          io.resetPivotEncoder(Position.HOMED.angle());
          isHomed = true;
          setWantedState(WantedState.HOME);
        }))
        .until(() -> isHomed) // 已歸零過則自動完成
        .finallyDo((interrupted) -> {
          // 若指令中途被強制取消，停止電壓輸出並退回 OFF 狀態
          if (interrupted && wantedState == WantedState.HOMING) {
            io.setPivotVoltage(0.0);
            setWantedState(WantedState.STOP);
          }
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }
}