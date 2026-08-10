package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.config.DriveConfiguration;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.util.RobotBumpSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation drive IO supporting either CTRE simulation or Maple-Sim, matching
 * 2910's split.
 */
public final class DriveIOSim extends DriveIOReal {
  private static final double SIMULATION_PERIOD_SECONDS = 0.005;
  private static final double ROBOT_MASS_POUNDS = 150.0;
  private static final double BUMPER_LENGTH_INCHES = 35.625;
  private static final double BUMPER_WIDTH_INCHES = 35.625;
  private static final int BUMP_SIMULATION_SUBTICKS = 5;

  private final SimulatedRobotState simulatedRobotState;
  private final DriveConfiguration configuration;

  public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain;
  private RobotBumpSim robotBumpSim;

  @SuppressWarnings("unused")
  // wpi
  private Notifier simulationNotifier;

  private double lastSimulationTime;

  public DriveIOSim(
      SimulatedRobotState simulatedRobotState, DriveConfiguration configuration) {
    super(regulateForSimulation(configuration));
    this.configuration = configuration;
    this.simulatedRobotState = simulatedRobotState;
    startSimulationThread();
  }

  private void startSimulationThread() {
    if (Constants.useMapleSim) {
      // The stock arena makes the complete BUMP region an impassable rectangle. Keep
      // the HUB
      // collider, but let RobotBumpSim model the BUMP surfaces and traversal instead.
      SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

      mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
          Seconds.of(SIMULATION_PERIOD_SECONDS),
          Pounds.of(ROBOT_MASS_POUNDS),
          Inches.of(BUMPER_LENGTH_INCHES),
          Inches.of(BUMPER_WIDTH_INCHES),
          DCMotor.getKrakenX60(1),
          DCMotor.getKrakenX60(1),
          1.2,
          getModuleLocations(),
          getPigeon2(),
          getModules(),
          configuration.moduleConstants());
      robotBumpSim = new RobotBumpSim(getModuleLocations());
      simulationNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    } else {
      lastSimulationTime = Utils.getCurrentTimeSeconds();
      simulationNotifier = new Notifier(
          () -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimulationTime;
            lastSimulationTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
          });
    }

    simulationNotifier.setName("Drive Simulation");
    simulationNotifier.startPeriodic(SIMULATION_PERIOD_SECONDS);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    super.updateInputs(inputs);

    if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
      Pose2d maplePose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
      var fieldRelativeSpeeds = mapleSimSwerveDrivetrain.mapleSimDrive
          .getDriveTrainSimulatedChassisSpeedsFieldRelative();
      var simulatedPose3d = robotBumpSim.update(maplePose, fieldRelativeSpeeds, BUMP_SIMULATION_SUBTICKS);

      if (robotBumpSim.isOnRamp()) {
        maplePose = robotBumpSim.getSimWorldPose(maplePose);
        mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(maplePose);
      }

      simulatedRobotState.addFieldToRobot(maplePose);

      Logger.recordOutput(
          "FieldSimulation/Fuel",
          SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
      Logger.recordOutput("Drive/Viz/SimPose", simulatedRobotState.getLatestFieldToRobot());
      Logger.recordOutput("Drive/Viz/SimPose3d", simulatedPose3d);
      Logger.recordOutput("Drive/Viz/IsOnBump", robotBumpSim.isOnRamp());
    }
  }

  @Override
  public void resetPose(Pose2d pose) {
    if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
      mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
      robotBumpSim = new RobotBumpSim(getModuleLocations());
      Timer.delay(0.05);
    }
    super.resetPose(pose);
  }

  public MapleSimSwerveDrivetrain getMapleSimDrive() {
    return mapleSimSwerveDrivetrain;
  }

  private static DriveConfiguration regulateForSimulation(DriveConfiguration configuration) {
    if (Constants.useMapleSim) {
      MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(
          configuration.moduleConstants());
    }
    return configuration;
  }
}
