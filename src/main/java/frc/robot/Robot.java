// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private boolean hasEnabled = false;

  public Robot() {
    Logger.recordMetadata("ProjectName", "2026CompetitionRobot-Offseason");
    Logger.recordMetadata("Architecture", "2910-inspired IO layers");

    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    m_robotContainer = new RobotContainer();

    if (RobotBase.isSimulation() && Constants.useMapleSim) {
      m_robotContainer
          .getDriveSubsystem()
          .resetPose(new Pose2d(1.0, 1.0, Rotation2d.kZero));
    }

    if (RobotBase.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (RobotBase.isSimulation() && Constants.useMapleSim) {
      m_robotContainer.getSimulatedRobotState().updateSim();
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.getSuperStructure().stopAll();
    m_robotContainer.getDrive().stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (RobotBase.isSimulation() && Constants.useMapleSim && !hasEnabled) {
      SimulatedArena.getInstance().placeGamePiecesOnField();
    }
    if (!hasEnabled) {
      hasEnabled = true;
    }

    m_robotContainer.getSuperStructure().clearStopped();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (RobotBase.isSimulation() && Constants.useMapleSim && !hasEnabled) {
      SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    hasEnabled = true;
    m_robotContainer.getSuperStructure().clearStopped();
    m_robotContainer.getSuperStructure().setIntakeRequested(false);
    m_robotContainer.getSuperStructure().setShootRequested(false);
    m_robotContainer.getSuperStructure().setDirectShootRequested(false);
    m_robotContainer.getDrive().requestTeleop();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getSuperStructure().stopAll();
    m_robotContainer.getDrive().stop();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    if (RobotBase.isSimulation() && Constants.useMapleSim && !hasEnabled) {
      Logger.recordOutput(
          "FieldSimulation/Fuel",
          SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
  }
}
