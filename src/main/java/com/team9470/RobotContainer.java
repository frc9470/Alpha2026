// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.Superstructure;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.team9470.Constants.OperatorConstants;
import com.team9470.subsystems.vision.Vision;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = TunerConstants.maxAngularVelocity;

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

  // Acceleration Tracking for Predictive Shooting
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();
  private static final double LOOP_PERIOD = 0.02; // 20ms standard FRC loop time

  public RobotContainer() {
    MaxAngularRate = Math.toRadians(TunerConstants.maxAngularVelocity);

    // Connect swerve context to superstructure (with acceleration supplier)
    m_superstructure.setDriveContext(
        m_swerve::getPose,
        m_swerve::getChassisSpeeds,
        this::calculateAcceleration);

    configureBindings();
  }

  /**
   * Calculate chassis acceleration for predictive shooting.
   */
  private ChassisSpeeds calculateAcceleration() {
    ChassisSpeeds currentSpeeds = m_swerve.getChassisSpeeds();
    ChassisSpeeds acceleration = new ChassisSpeeds(
        (currentSpeeds.vxMetersPerSecond - m_lastChassisSpeeds.vxMetersPerSecond) / LOOP_PERIOD,
        (currentSpeeds.vyMetersPerSecond - m_lastChassisSpeeds.vyMetersPerSecond) / LOOP_PERIOD,
        (currentSpeeds.omegaRadiansPerSecond - m_lastChassisSpeeds.omegaRadiansPerSecond) / LOOP_PERIOD);
    m_lastChassisSpeeds = currentSpeeds;
    return acceleration;
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

    // A/Y: Climber (placeholder for future implementation)
    // m_driverController.a().whileTrue(climberOutCommand);
    // m_driverController.y().whileTrue(climberInCommand);

    // ==================== DEFAULT COMMANDS ====================
    m_swerve.setDefaultCommand(
        m_swerve.applyRequest(() -> drive
            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
