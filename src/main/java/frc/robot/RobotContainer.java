// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoFactory;
import frc.robot.autos.AutoMode;
import frc.robot.autos.AutoRoutine;
import frc.robot.config.RobotConfiguration;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOReal;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelightHelper;

public class RobotContainer {
  private final RobotConfiguration configuration = RobotConfiguration.competitionRobot();
  private final RobotState robotState = new RobotState();
  private final SimulatedRobotState simulatedRobotState = RobotBase.isSimulation() ? new SimulatedRobotState(this)
      : null;

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Vision vision;
  private final Feeder feeder;
  private final SuperStructure superStructure;

  private final AutoFactory autoFactory;
  private final SendableChooser<AutoMode> autoChooser = new SendableChooser<>();
  private final CommandPS5Controller driverController = new CommandPS5Controller(0);

  public RobotContainer() {
    DriveIO driveIO;
    if (RobotBase.isReal()) {
      driveIO = new DriveIOReal(configuration.drive());
    } else {
      driveIO = new DriveIOSim(simulatedRobotState, configuration.drive());
    }
    drive = new Drive(robotState, driveIO, configuration.drive());
    SmartDashboard.putData("Field", drive.getField());

    if (RobotBase.isReal()) {
      intake = new Intake(new IntakeIOReal(configuration.intake()), configuration.intake());
      shooter = new Shooter(new ShooterIOReal(configuration.shooter()), configuration.shooter());
      feeder = new Feeder(new FeederIOReal(configuration.feeder()), configuration.feeder());
    } else {
      intake = new Intake(new IntakeIOSim(), configuration.intake());
      shooter = new Shooter(new ShooterIOSim(), configuration.shooter());
      feeder = new Feeder(new FeederIOReal(configuration.feeder()), configuration.feeder());
    }

    superStructure = new SuperStructure(drive, robotState, intake, shooter, feeder);
    vision = new Vision(
        robotState,
        drive,
        configuration.vision(),
        new VisionIOLimelightHelper(configuration.vision()));
    autoFactory = new AutoFactory(drive, superStructure);

    if (RobotBase.isSimulation() && Constants.useMapleSim) {
      assert simulatedRobotState != null;
      simulatedRobotState.init();
    }

    configureBindings();
    configureAutoChooser();

  }

  private void configureBindings() {

    // RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
    // .onTrue(intake.deployIntake());

    drive.setDefaultCommand(
        drive.run(
            () -> drive.acceptTeleopInput(
                -driverController.getLeftY(),
                -driverController.getLeftX(),
                -driverController.getRightX()))
            .withName("Drive Field Relative"));

    driverController
        .create()
        .onTrue(Commands.runOnce(drive::resetHeadingForAlliance).ignoringDisable(true));

    // Intake and deploy Pivot
    driverController
        .circle()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> superStructure.setIntakeRequested(true)))
        .onFalse(Commands.runOnce(() -> superStructure.setIntakeRequested(false)));

    // Aim and Shoot
    driverController
        .triangle()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> superStructure.setShootRequested(true)))
        .onFalse(Commands.runOnce(() -> superStructure.setShootRequested(false)));

    // Direct Shoot
    driverController
        .square()
        .and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> superStructure.setDirectShootRequested(true)))
        .onFalse(Commands.runOnce(() -> superStructure.setDirectShootRequested(false)));

    if (RobotBase.isSimulation()) {
      driverController
          .triangle()
          .and(DriverStation::isTeleopEnabled)
          .whileTrue(
              Commands.run(
                  () -> simulatedRobotState.launchFuel()));
    }
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", AutoMode.DO_NOTHING);
    autoChooser.addOption("Shoot Only", AutoMode.SHOOT_ONLY);
    autoChooser.addOption("BLine Shoot (Alliance Flip)", AutoMode.BLINE_SHOOT);
    SmartDashboard.putData("Auto/Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    AutoMode selected = autoChooser.getSelected();
    if (selected == null) {
      selected = AutoMode.DO_NOTHING;
    }
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    AutoRoutine routine = autoFactory.create(selected, alliance);
    return routine.command();
  }

  public Drive getDrive() {
    return drive;
  }

  public Drive getDriveSubsystem() {
    return drive;
  }

  public RobotState getRobotState() {
    return robotState;
  }

  public SuperStructure getSuperStructure() {
    return superStructure;
  }

  public Intake getIntake() {
    return intake;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Vision getVision() {
    return vision;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  public SimulatedRobotState getSimulatedRobotState() {
    return simulatedRobotState;
  }
}
