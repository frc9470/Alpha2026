package com.team9470.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperConstants {
        // NOTE: CAN IDs are in Ports.java (HOPPER_LEFT, HOPPER_RIGHT, HOPPER_TOP)

        // Control
        public static final double kFeedVoltage = -12.0; // Voltage when feeding to shooter
        public static final double kRollerSupplyCurrentLimit = 25.0;
        public static final double kTopStatorCurrentLimit = 60.0;

        // Motor Configs
        public static final TalonFXConfiguration kLeftConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kRightConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kTopConfig = new TalonFXConfiguration();

        static {
                // Left Motor Config
                kLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                kRightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                kLeftConfig.CurrentLimits
                                .withSupplyCurrentLimit(kRollerSupplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true);

                // Right Motor Config (inverted for opposing roller direction)
                kRightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                kRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                kRightConfig.CurrentLimits
                                .withSupplyCurrentLimit(kRollerSupplyCurrentLimit)
                                .withSupplyCurrentLimitEnable(true);

                // Top Motor Config (opposite orientation to left motor)
                kTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                kTopConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                kTopConfig.CurrentLimits
                                .withSupplyCurrentLimitEnable(false)
                                .withStatorCurrentLimit(kTopStatorCurrentLimit)
                                .withStatorCurrentLimitEnable(true);
        }
}
