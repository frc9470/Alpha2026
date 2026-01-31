// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.Superstructure;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.team9470.Constants.OperatorConstants;
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

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    MaxAngularRate = Math.toRadians(TunerConstants.maxAngularVelocity);

    // Connect swerve context to superstructure
    m_superstructure.setDriveContext(m_swerve::getPose, m_swerve::getChassisSpeeds);

    configureBindings();
  }

  private void configureBindings() {
    // Right Bumper: Auto-Aim & Shoot
    m_driverController.rightBumper().whileTrue(
        new RunCommand(() -> {
          // Get aim result from superstructure
          var aim = m_superstructure.getAimResult();

          // Dynamic Swerve Limiting - prioritize rotation
          final double kDriveRadius = 0.45; // meters
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

          // Telemetry
          SmartDashboard.putBoolean("Drive/AutoAimActive", true);
          SmartDashboard.putNumber("Drive/RotCmd", aim.rotationCommand());
          SmartDashboard.putNumber("Drive/TransLimit", transLimit);

          // Drive with auto-aim rotation
          m_swerve.setControl(autoAimDrive
              .withVelocityX(vX)
              .withVelocityY(vY)
              .withRotationalRate(aim.rotationCommand()));

        }, m_swerve)
            .alongWith(m_superstructure.aimAndShootCommand())
            .finallyDo(() -> SmartDashboard.putBoolean("Drive/AutoAimActive", false)));

    // Left Trigger: Intake
    m_driverController.leftTrigger().whileTrue(m_superstructure.intakeCommand());

    // Default Drive (Manual)
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
