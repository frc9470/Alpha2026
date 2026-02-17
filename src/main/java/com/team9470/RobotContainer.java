// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.team9470.Constants.OperatorConstants;
import com.team9470.subsystems.vision.Vision;
import com.team9470.autos.CenterRushAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = Math.toRadians(TunerConstants.maxAngularVelocity);

  // Swerve requests
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric autoAimDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(0.0)
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  // Subsystems
  private final Swerve m_swerve = Swerve.getInstance();
  private final Superstructure m_superstructure = Superstructure.getInstance();
  private final Vision m_vision = Vision.getInstance();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  // Auto Chooser
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Debug Y-shot dashboard tuning (only used by Y button command)
  private static final String kDebugYShotRpmKey = "Debug/YShot/RPM";
  private static final String kDebugYShotHoodDegKey = "Debug/YShot/HoodAngleDeg";
  private static final String kDebugYShotRequestedRpmKey = "Debug/YShot/RequestedRPM";
  private static final String kDebugYShotRequestedHoodDegKey = "Debug/YShot/RequestedHoodDeg";
  private static final String kDebugYShotAppliedRpmKey = "Debug/YShot/AppliedRPM";
  private static final String kDebugYShotAppliedHoodDegKey = "Debug/YShot/AppliedHoodDeg";
  private static final String kDebugYShotReadyToFeedKey = "Debug/YShot/ReadyToFeed";
  private static final double kDebugYShotDefaultRpm = 3000.0;
  private static final double kDebugYShotDefaultHoodDeg = 30.0;

  public RobotContainer() {
    // Connect swerve context to superstructure
    m_superstructure.setDriveContext(
        m_swerve::getPose,
        m_swerve::getChassisSpeeds);

    initDebugYShotDashboard();
    configureBindings();

    // Auto Chooser
    m_chooser.setDefaultOption("None", null);
    m_chooser.addOption("Center Rush Auto", new CenterRushAuto());
    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  private void initDebugYShotDashboard() {
    SmartDashboard.putNumber(kDebugYShotRpmKey, kDebugYShotDefaultRpm);
    SmartDashboard.putNumber(kDebugYShotHoodDegKey, kDebugYShotDefaultHoodDeg);
    SmartDashboard.putNumber(kDebugYShotRequestedRpmKey, kDebugYShotDefaultRpm);
    SmartDashboard.putNumber(kDebugYShotRequestedHoodDegKey, kDebugYShotDefaultHoodDeg);
  }

  private final SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();

  private void configureBindings() {
    // ==================== TRIGGERS ====================

    // Left Trigger: Intake (deploy arm + run rollers while held)
    m_driverController.leftTrigger().whileTrue(m_superstructure.intakeCommand());

    // Right Trigger: Auto-Aim & Shoot/Feed
    m_driverController.rightTrigger().whileTrue(
        new RunCommand(() -> {
          var aim = m_superstructure.getAimResult();

          // Dynamic Swerve Limiting - prioritize rotation
          final double kDriveRadius = 0.45;
          double rotCost = Math.abs(aim.rotationCommand()) * kDriveRadius;
          double transLimit = Math.max(0.0, (MaxSpeed - rotCost) * 0.80);

          double vX = -m_driverController.getLeftY() * MaxSpeed;
          double vY = -m_driverController.getLeftX() * MaxSpeed;

          // Desaturate translation
          double vMag = Math.hypot(vX, vY);
          if (vMag > transLimit) {
            double scale = transLimit / vMag;
            vX *= scale;
            vY *= scale;
          }

          SmartDashboard.putBoolean("Drive/AutoAimActive", true);
          SmartDashboard.putNumber("Drive/RotCmd", aim.rotationCommand());
          SmartDashboard.putNumber("Drive/TransLimit", transLimit);

          m_swerve.setControl(autoAimDrive
              .withVelocityX(vX)
              .withVelocityY(vY)
              .withRotationalRate(aim.rotationCommand()));

        }, m_swerve)
            .alongWith(m_superstructure.aimAndShootCommand())
            .finallyDo(() -> SmartDashboard.putBoolean("Drive/AutoAimActive", false)));

    // ==================== BUMPERS ====================

    // Left Bumper: Toggle intake deployed/retracted
    m_driverController.leftBumper().onTrue(m_superstructure.toggleIntakeCommand());

    // Right Bumper: Outtake (reverse rollers while held)
    m_driverController.rightBumper().whileTrue(m_superstructure.outtakeCommand());

    // ==================== FACE BUTTONS ====================

    // X: X-lock wheels (defensive stance)
    m_driverController.x().whileTrue(m_swerve.applyRequest(() -> xLock));

    // B: Zero swerve heading (reset field-centric forward)
    m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldCentric()));

    // A: Debug - Run hopper while held
    m_driverController.a().whileTrue(m_superstructure.getHopper().runCommand());

    // Y: Debug - Spin up shooter + set hood from dashboard + feed hopper while held
    m_driverController.y().whileTrue(
        Commands.runEnd(
            () -> {
              double requestedRpm = SmartDashboard.getNumber(kDebugYShotRpmKey, kDebugYShotDefaultRpm);
              double requestedHoodDeg = SmartDashboard.getNumber(kDebugYShotHoodDegKey, kDebugYShotDefaultHoodDeg);
              double rpm = Math.max(0.0, requestedRpm);
              double hoodDeg = requestedHoodDeg;
              double clampedHoodDeg = Math.max(
                  ShooterConstants.kMinHoodAngle.in(Degrees),
                  Math.min(ShooterConstants.kMaxHoodAngle.in(Degrees), hoodDeg));
              double targetRps = rpm / 60.0;

              m_superstructure.getShooter().setFlywheelSpeed(targetRps);
              m_superstructure.getShooter().setHoodAngle(
                  ShooterConstants.launchRadToMechanismRotations(Math.toRadians(clampedHoodDeg)));
              boolean readyToFeed = m_superstructure.getShooter().isAtSetpoint();
              m_superstructure.getShooter().setFiring(readyToFeed);
              m_superstructure.getHopper().setRunning(readyToFeed);

              SmartDashboard.putNumber(kDebugYShotRequestedRpmKey, requestedRpm);
              SmartDashboard.putNumber(kDebugYShotRequestedHoodDegKey, requestedHoodDeg);
              SmartDashboard.putNumber(kDebugYShotAppliedRpmKey, rpm);
              SmartDashboard.putNumber(kDebugYShotAppliedHoodDegKey, clampedHoodDeg);
              SmartDashboard.putBoolean(kDebugYShotReadyToFeedKey, readyToFeed);
            },
            () -> {
              m_superstructure.getShooter().setFlywheelSpeed(0);
              m_superstructure.getShooter().setFiring(false);
              m_superstructure.getHopper().stop();
              SmartDashboard.putBoolean(kDebugYShotReadyToFeedKey, false);
            }));

    // ==================== DEFAULT COMMANDS ====================
    m_swerve.setDefaultCommand(
        m_swerve.applyRequest(() -> drive
            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
