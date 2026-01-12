// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team9470;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ShooterConstants {
    // Hardware IDs
    public static final int kFlywheelMasterId = 50;
    public static final int kFlywheelSlaveId = 51;
    public static final int kHoodMotorId = 52;

    // Physical Constants
    public static final double kFlywheelGearRatio = 1.5;
    public static final double kFlywheelEfficiency = 0.8;
    // 9 lb*in^2 = 9 * 0.0002926397 ~= 0.002634 kg*m^2
    public static final double kFlywheelMOI = 0.002634;

    public static final double kHoodGearRatio = 50.0;
    public static final double kHoodMOI = 0.05; // kg*m^2 (approx)
    public static final double kHoodLength = 0.2; // m
    public static final double kHoodMass = 2.0; // kg

    public static final double kMinHoodAngle = 0.0; // Radians
    public static final double kMaxHoodAngle = Math.toRadians(85.0); // Radians

    // Field Geometry
    public static final double kShooterOffsetX = 0.2; // Meters (Forward from robot center)
    public static final double kShooterOffsetZ = 0.5; // Meters (Up from center)

    // PID / FF Gains
    // Flywheel (VelocityVol m_targetSpeedRPS = Units.radiansToRotations(radsPerSec)
    // * ShooterConstants.kFlywheelGearRatio;ond (RPS)
    public static final double kFlywheelVkP = 0.5;
    public static final double kFlywheelVkV = 0.12; // ~12V / 100 RPS

    // Hood (PositionVoltage): Units are Rotations
    public static final double kHoodVkP = 40.0;
    public static final double kHoodVkG = 0.2; // Gravity hold

  }

  public static class IntakeConstants {
    // --- Intake ---
    public static final int kIntakePivotId = 60;
    public static final int kIntakeRollerId = 61;

    public static final double kIntakePivotGearRatio = 20.0; // Estimate
    public static final double kIntakePivotKp = 40.0;
    public static final double kIntakePivotKv = 0.5; // V/(rad/s)
    public static final double kIntakePivotKa = 0.1;
    public static final double kIntakePivotKg = 0.5; // Gravity Hold

    public static final double kIntakeMotionMagicCruiseVel = 4.0; // rad/s
    public static final double kIntakeMotionMagicAccel = 8.0; // rad/s^2

    public static final double kIntakeDeployAngle = Math.toRadians(-25.0); // Down/Floor
    public static final double kIntakeRetractAngle = Math.toRadians(90.0); // Up/Stowed
    public static final double kIntakeRollerVoltage = 8.0;

    // Simulation
    public static final double kIntakeLength = 0.3; // meters
    public static final double kIntakeMass = 4.0; // kg
  }
}
