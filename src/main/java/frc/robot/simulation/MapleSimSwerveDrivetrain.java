package frc.robot.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Couples Maple-Sim physics to CTRE's simulated TalonFX, CANcoder, and Pigeon devices. */
public final class MapleSimSwerveDrivetrain {
  private final Pigeon2SimState pigeonSim;
  public final SwerveDriveSimulation mapleSimDrive;

  @SafeVarargs
  public MapleSimSwerveDrivetrain(
      Time simulationPeriod,
      Mass robotMassWithBumpers,                         
      Distance bumperLength,
      Distance bumperWidth,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCoefficientOfFriction,
      Translation2d[] moduleLocations,
      Pigeon2 pigeon,
      SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    pigeonSim = pigeon.getSimState();

    DriveTrainSimulationConfig simulationConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMassWithBumpers)
            .withBumperSize(bumperLength, bumperWidth)
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(moduleLocations)
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveMotorModel,
                    steerMotorModel,
                    moduleConstants[0].DriveMotorGearRatio,
                    moduleConstants[0].SteerMotorGearRatio,
                    Volts.of(moduleConstants[0].DriveFrictionVoltage),
                    Volts.of(moduleConstants[0].SteerFrictionVoltage),
                    Meters.of(moduleConstants[0].WheelRadius),
                    KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                    wheelCoefficientOfFriction));

    mapleSimDrive = new SwerveDriveSimulation(simulationConfig, Pose2d.kZero);
    SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
    for (int i = 0; i < modules.length; i++) {
      moduleSimulations[i].useDriveMotorController(
          new TalonFXMotorControllerSim(modules[i].getDriveMotor()));
      moduleSimulations[i].useSteerMotorController(
          new TalonFXMotorControllerWithCANcoderSim(
              modules[i].getSteerMotor(), modules[i].getEncoder()));
    }

    SimulatedArena.overrideSimulationTimings(simulationPeriod, 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
  }

  /** Advances the arena, resolves collisions, and writes physics results into CTRE sensors. */
  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();
    pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(
        RadiansPerSecond.of(
            mapleSimDrive
                .getDriveTrainSimulatedChassisSpeedsRobotRelative()
                .omegaRadiansPerSecond));
  }

  public Pose2d getSimulatedPose() {
    return mapleSimDrive.getSimulatedDriveTrainPose();
  }

  public void resetPose(Pose2d pose) {
    mapleSimDrive.setSimulationWorldPose(pose);
  }

  public static void regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>[] moduleConstants) {
    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
      regulateModuleConstantForSimulation(moduleConstant);
    }
  }

  private static void regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstant) {
    if (RobotBase.isReal()) {
      return;
    }

    moduleConstant
        .withEncoderOffset(0.0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withSteerMotorGains(moduleConstant.SteerMotorGains.withKP(70.0).withKD(4.5))
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.15))
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }

  private static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState talonSim;

    TalonFXMotorControllerSim(TalonFX talonFX) {
      talonSim = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonSim.setRawRotorPosition(encoderAngle);
      talonSim.setRotorVelocity(encoderVelocity);
      talonSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonSim.getMotorVoltageMeasure();
    }
  }

  private static final class TalonFXMotorControllerWithCANcoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState cancoderSim;

    TalonFXMotorControllerWithCANcoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      cancoderSim = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      cancoderSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      cancoderSim.setRawPosition(mechanismAngle);
      cancoderSim.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }
}
// @Override 
/// public Voltage updateControlSiganl(
//int LeftFront canbusID = 1;
