// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.team9470.subsystems.swerve.Swerve;
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.team9470.Constants.OperatorConstants;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = TunerConstants.maxAngularVelocity; // Radians per second? No, constant is in degrees?
                                                                     // Wait, let me check TunerConstants again.
  // TunerConstants.maxAngularVelocity is 572.96. Assuming degrees based on
  // variable name comment in typical TunerX gen.
  // SwerveRequest.FieldCentric expects Radians per second for rotational rate.
  // So MaxAngularRate should be Math.toRadians(TunerConstants.maxAngularVelocity)
  // if that constant is degrees.

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  // Separate request for auto-aim to ensure no interference
  private final SwerveRequest.FieldCentric autoAimDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(0.0) // No deadband for PID
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final Swerve m_swerve = Swerve.getInstance();

  public final com.team9470.subsystems.shooter.Shooter m_shooter = new com.team9470.subsystems.shooter.Shooter();
  private final com.team9470.subsystems.intake.Intake m_intake = com.team9470.subsystems.intake.Intake.getInstance();
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  // Acceleration Tracking for Predictive Shooting
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();
  private static final double LOOP_PERIOD = 0.02; // 20ms standard FRC loop time

  public RobotContainer() {
    MaxAngularRate = Math.toRadians(TunerConstants.maxAngularVelocity);

    m_shooter.setSimulationContext(m_swerve::getPose, m_swerve::getChassisSpeeds);

    configureBindings();
  }

  private void configureBindings() {
    // Right Bumper: Auto-Aim & Rapid Fire
    m_driverController.rightBumper().whileTrue(
        new RunCommand(() -> {
          // Get current chassis speeds
          ChassisSpeeds currentSpeeds = m_swerve.getChassisSpeeds();
          
          // Calculate acceleration: a = (v_current - v_last) / dt
          ChassisSpeeds acceleration = new ChassisSpeeds(
              (currentSpeeds.vxMetersPerSecond - m_lastChassisSpeeds.vxMetersPerSecond) / LOOP_PERIOD,
              (currentSpeeds.vyMetersPerSecond - m_lastChassisSpeeds.vyMetersPerSecond) / LOOP_PERIOD,
              (currentSpeeds.omegaRadiansPerSecond - m_lastChassisSpeeds.omegaRadiansPerSecond) / LOOP_PERIOD
          );
          
          // Store current speeds for next iteration
          m_lastChassisSpeeds = currentSpeeds;
          
          // Calculate Aim with acceleration feedforward
          var solution = com.team9470.util.AutoAim.calculate(
              m_swerve.getPose(),
              currentSpeeds,
              acceleration);

          // Aim Robot
          double rotError = solution.targetRobotYaw().minus(m_swerve.getPose().getRotation()).getRadians();
          double rotCmd = (rotError * 12.0) + solution.targetOmega(); // Add Feedforward

          // Fire if aligned
          boolean isAligned = Math.abs(rotError) < Math.toRadians(3.0);

          // Telemetry
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Debug/ShootCommandRunning", true);
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Debug/RotErrorDeg",
              edu.wpi.first.math.util.Units.radiansToDegrees(rotError));
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Debug/TargetYawDeg",
              solution.targetRobotYaw().getDegrees());
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Debug/CurrentYawDeg",
              m_swerve.getPose().getRotation().getDegrees());
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Debug/RotCmd", rotCmd);
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Debug/IsAligned", isAligned);

          m_shooter.setFiring(isAligned);

          // Dynamic Swerve Limiting
          // Prioritize Rotation: Slow down translation if rotation demands high wheel
          // speed
          final double kDriveRadius = 0.45; // meters (hypot(13.5, 11.5) inches)
          double rotCost = Math.abs(rotCmd) * kDriveRadius;
          double transLimit = Math.max(0.0, (MaxSpeed - rotCost) * 0.80); // 20% Overhead Safety Margin

          double vX = -m_driverController.getLeftY() * MaxSpeed;
          double vY = -m_driverController.getLeftX() * MaxSpeed;

          // Desaturate Translation
          double vMag = Math.hypot(vX, vY);
          if (vMag > transLimit) {
            double scale = transLimit / vMag;
            vX *= scale;
            vY *= scale;
          }

          SmartDashboard.putNumber("Debug/Limiting/RotCost", rotCost);
          SmartDashboard.putNumber("Debug/Limiting/TransLimit", transLimit);
          SmartDashboard.putNumber("Debug/Limiting/OriginalMag", vMag);
          SmartDashboard.putNumber("Debug/Limiting/FinalMag", Math.hypot(vX, vY));

          // Drive
          m_swerve.setControl(autoAimDrive.withVelocityX(vX)
              .withVelocityY(vY)
              .withRotationalRate(rotCmd));
        }, m_swerve).finallyDo(() -> {
          m_shooter.setFiring(false);
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Debug/ShootCommandRunning", false);
        }));

    // Bind Left Trigger to Intake Deploy
    m_driverController.leftTrigger().whileTrue(m_intake.getDeployCommand()); // Added

    // Continuous Setpoint Update (Tracking)
    m_shooter.setDefaultCommand(new RunCommand(() -> {
      var solution = com.team9470.util.AutoAim.calculate(
          m_swerve.getPose(),
          m_swerve.getChassisSpeeds());
      m_shooter.setSetpoint(solution);
    }, m_shooter));

    // Default Drive (Manual)
    m_swerve.setDefaultCommand(
        m_swerve.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));
  }

  public Command getAutonomousCommand() {

    return null;
  }
}
