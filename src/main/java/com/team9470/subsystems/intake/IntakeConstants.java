package com.team9470.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    // NOTE: CAN IDs are in Ports.java (INTAKE_PIVOT, INTAKE_ROLLER)

    // Physical Constants
    public static final double kPivotGearRatio = 20.0;

    // Setpoints
    public static final double kDeployAngle = Math.toRadians(-25.0); // Down/Floor
    public static final double kRetractAngle = Math.toRadians(90.0); // Up/Stowed
    public static final double kRollerVoltage = 8.0;

    // Simulation
    public static final double kIntakeLength = 0.3; // meters
    public static final double kIntakeMass = 4.0; // kg

    // Motor Configs
    public static final TalonFXConfiguration kPivotConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRollerConfig = new TalonFXConfiguration();

    static {
        // Pivot Config
        kPivotConfig.Slot0.kP = 40.0;
        kPivotConfig.Slot0.kI = 0.0;
        kPivotConfig.Slot0.kD = 0.0;
        kPivotConfig.Slot0.kV = 0.5;
        kPivotConfig.Slot0.kA = 0.1;
        kPivotConfig.Slot0.kG = 0.5; // Gravity hold
        kPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kPivotConfig.Feedback.SensorToMechanismRatio = kPivotGearRatio;
        kPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 4.0; // rad/s
        kPivotConfig.MotionMagic.MotionMagicAcceleration = 8.0; // rad/s^2
        kPivotConfig.MotionMagic.MotionMagicJerk = 0;
        kPivotConfig.CurrentLimits.withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);

        // Roller Config
        kRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kRollerConfig.CurrentLimits.withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);
    }
}
