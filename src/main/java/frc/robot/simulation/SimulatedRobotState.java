package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.util.ShooterCalculator;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import org.littletonrobotics.junction.Logger;

/**
 * Stores and manages Maple-Sim ground-truth robot state, following 2910's
 * implementation.
 */
@SuppressWarnings("UnusedVariable")
public class SimulatedRobotState {
  private SwerveDriveSimulation simDrive;
  private final RobotContainer container;
  private double lastTimestamp = 0.0;
  private final Timer launchTimer = new Timer();
  private Pose2d fieldToRobotSimulatedTruth = Pose2d.kZero;

  public SimulatedRobotState(RobotContainer robotContainer) {
    container = robotContainer;
  }

  public void init() {
    simDrive = container.getDriveSubsystem().getMapleSimDrive().mapleSimDrive;
    launchTimer.start();
  }

  public synchronized void addFieldToRobot(Pose2d pose) {
    updateRobotPoseIfNewer(Timer.getFPGATimestamp(), pose);
  }

  public synchronized Pose2d getLatestFieldToRobot() {
    return fieldToRobotSimulatedTruth;
  }

  public void launchFuel() {
    if (launchTimer.hasElapsed(0.2)) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new RebuiltFuelOnFly(
                  simDrive.getSimulatedDriveTrainPose().getTranslation(),
                  new Translation2d(-8.0 * 0.0254, 0),
                  simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  SuperStructure
                      .calculateRearShooterHeading(simDrive.getSimulatedDriveTrainPose(),
                          FieldConstants.hubForAlliance(DriverStation.getAlliance().orElse(Alliance.Blue)))
                      .rotateBy(new Rotation2d(Math.toRadians(180))),
                  Units.Meters.of(0.47),
                  Units.MetersPerSecond.of(ShooterCalculator.calculateRPM(simDrive.getSimulatedDriveTrainPose()
                      .getTranslation().getDistance(FieldConstants.hubForAlliance(
                          DriverStation.getAlliance().orElse(Alliance.Blue))))),
                  Units.Degrees.of(75))
                  .withProjectileTrajectoryDisplayCallBack(
                      (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot",
                          pose3ds.toArray(Pose3d[]::new)),
                      (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot",
                          pose3ds.toArray(Pose3d[]::new))));
      launchTimer.reset();
    } else {
      return;
    }

  }

  public synchronized void updateSim() {
    lastTimestamp = Timer.getFPGATimestamp();
  }

  private void updateRobotPoseIfNewer(double timestamp, Pose2d pose) {
    if (timestamp > lastTimestamp) {
      lastTimestamp = timestamp;
      fieldToRobotSimulatedTruth = pose;
    }
  }
}
