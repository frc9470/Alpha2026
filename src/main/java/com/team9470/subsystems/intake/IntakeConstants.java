package com.team9470.subsystems.intake;

public class IntakeConstants {
    // NOTE: CAN IDs are in Ports.java (INTAKE_PIVOT, INTAKE_ROLLER)

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
