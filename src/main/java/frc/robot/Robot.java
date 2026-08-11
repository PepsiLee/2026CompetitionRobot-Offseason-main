// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.intake.Intake.Position;
import frc.robot.util.ShooterCalculator;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

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
  }

  @Override
  public void disabledInit() {
    m_robotContainer.getSuperStructure().stopAll();
    m_robotContainer.getDrive().stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

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
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    // Stop the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Ininitlize the simulation
    if (RobotBase.isSimulation() && Constants.useMapleSim && !hasEnabled) {
      SimulatedArena.getInstance().placeGamePiecesOnField();
    }

    // m_robotContainer.getDrive().resetPose(new Pose2d(0,0,Rotation2d.k180deg));

    hasEnabled = true;
    m_robotContainer.getSuperStructure().clearStopped();
    m_robotContainer.getSuperStructure().setIntakeRequested(false);
    m_robotContainer.getSuperStructure().setShootRequested(false);
    m_robotContainer.getSuperStructure().setDirectShootRequested(false);
    m_robotContainer.getDrive().requestTeleop();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getSuperStructure().stopAll();
    m_robotContainer.getDrive().stop();

    SmartDashboard.setDefaultBoolean("Feeder/On", false);
    SmartDashboard.setDefaultBoolean("Shooter/On", false);    
    SmartDashboard.setDefaultBoolean("Intake/On", false);

    SmartDashboard.setDefaultNumber("Feeder/Target Voltage", 0.0);
    SmartDashboard.setDefaultNumber("Shooter/Target RPM", 0.0);
    SmartDashboard.setDefaultNumber("Intake/Roller Voltage", 0.0);
    SmartDashboard.setDefaultNumber("Intake Pivot/Position", 0.0);

  }

  @Override
  public void testPeriodic() {
    double targetRPM = SmartDashboard.getNumber("Shooter/Target RPM", 0.0);
    double targetVoltage = SmartDashboard.getNumber("Feeder/Target Voltage", 0.0);
    double intakeTargetVoltage = SmartDashboard.getNumber("Intake/Target Voltage", 0.0);
    double intakePivotTargetPosition = SmartDashboard.getNumber("Intake Pivot/Position", 0.0);

    boolean shooter = SmartDashboard.getBoolean("Shooter/On", false);
    boolean feeder = SmartDashboard.getBoolean("Feeder/On", false);
    boolean intake = SmartDashboard.getBoolean("Intake/On", false);

    m_robotContainer.getShooter().setWantedState(shooter ? frc.robot.subsystems.shooter.Shooter.WantedState.SHOOTING
        : frc.robot.subsystems.shooter.Shooter.WantedState.OFF);
    m_robotContainer.getShooter().setRPM(targetRPM);

    m_robotContainer.getFeeder().setWantedState(feeder ? frc.robot.subsystems.feeder.Feeder.WantedState.TEST_SHOOTER
        : frc.robot.subsystems.feeder.Feeder.WantedState.OFF);
    m_robotContainer.getFeeder().setVoltage(targetVoltage);

    m_robotContainer.getIntake().setWantedState(intake ? frc.robot.subsystems.intake.Intake.WantedState.TEST
        : frc.robot.subsystems.intake.Intake.WantedState.STOP);
    m_robotContainer.getIntake().setIntakeTestVoltage(intakeTargetVoltage);
    m_robotContainer.getIntake().setCustomPivotPosition(Degrees.of(intakePivotTargetPosition));

    Pose2d pose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rear").pose;
    double distance = pose.getTranslation()
        .getDistance(FieldConstants.hubForAlliance(DriverStation.getAlliance().orElse(Alliance.Blue)));
    SmartDashboard.putNumber("test/distance", distance);
    SmartDashboard.putNumber("test/calculateRPM", ShooterCalculator.calculateRPM(Math.abs(distance)));
  }

  @Override
  public void testExit() {
    m_robotContainer.getSuperStructure().stopAll();
    m_robotContainer.getDrive().stop();
  }

  @Override
  public void simulationPeriodic() {
    if (Constants.useMapleSim) {
      m_robotContainer.getSimulatedRobotState().updateSim();
      Logger.recordOutput(
          "FieldSimulation/Fuel",
          SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
  }
}
