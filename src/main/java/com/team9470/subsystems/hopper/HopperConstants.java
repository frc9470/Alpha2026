package com.team9470.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperConstants {
    // NOTE: CAN IDs are in Ports.java (HOPPER_MOTOR_1, HOPPER_MOTOR_2)

    // Control
    public static final double kFeedVoltage = 8.0; // Voltage when feeding to shooter

    // Motor Configs
    public static final TalonFXConfiguration kMotor1Config = new TalonFXConfiguration();
    public static final TalonFXConfiguration kMotor2Config = new TalonFXConfiguration();

    static {
        // Motor 1 Config
        kMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMotor1Config.CurrentLimits.withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);

        // Motor 2 Config (inverted for opposing roller direction)
        kMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMotor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMotor2Config.CurrentLimits.withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);
    }
}
