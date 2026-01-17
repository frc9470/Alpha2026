package com.team9470.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConstants {
        // Hardware IDs
        public static final int kFlywheelMasterId = 50;
        public static final int kFlywheelSlaveId = 51;
        public static final int kFlywheelSlave2Id = 53;
        public static final int kFlywheelSlave3Id = 54;
        public static final int kHoodMotorId = 52;

        // Physical Constants
        public static final double kFlywheelGearRatio = 1.5;
        public static final double kFlywheelEfficiency = 0.8;
        public static final double kMaxMotorSpeed = 100.0; // RPS (~6000 RPM Kraken)

        // 9 lb*in^2 = 9 * 0.0002926397 ~= 0.002634 kg*m^2
        public static final MomentOfInertia kFlywheelMOI = KilogramSquareMeters.of(0.002634);

        public static final double kHoodGearRatio = 50.0;
        public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.05);
        public static final Distance kHoodLength = Meters.of(0.2);
        public static final Mass kHoodMass = Kilograms.of(2.0);

        public static final Angle kMinHoodAngle = Radians.of(0.0);
        public static final Angle kMaxHoodAngle = Degrees.of(85.0);

        // Field Geometry
        public static final Distance kShooterOffsetX = Meters.of(0.2); // Forward from robot center
        public static final Distance kShooterOffsetZ = Meters.of(0.5); // Up from center

        // PID / FF Gains
        public static final TalonFXConfiguration kFlywheelConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kHoodConfig = new TalonFXConfiguration();

        static {
                // Flywheel Config
                kFlywheelConfig.Slot0.kP = 0.25;
                kFlywheelConfig.Slot0.kI = 0.0;
                kFlywheelConfig.Slot0.kD = 0.01;
                kFlywheelConfig.Slot0.kV = 0.125;
                kFlywheelConfig.Slot0.kS = 0.0;
                kFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                kFlywheelConfig.CurrentLimits.withSupplyCurrentLimit(80.0).withSupplyCurrentLimitEnable(true);

                // Hood Config
                kHoodConfig.Slot0.kP = 40.0;
                kHoodConfig.Slot0.kI = 0.0;
                kHoodConfig.Slot0.kD = 0.0;
                kHoodConfig.Slot0.kV = 0.0;
                kHoodConfig.Slot0.kG = 0.2; // Gravity hold (Volts)
                kHoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                kHoodConfig.Feedback.SensorToMechanismRatio = kHoodGearRatio;
                kHoodConfig.CurrentLimits.withSupplyCurrentLimit(40.0).withSupplyCurrentLimitEnable(true);
        }
}
