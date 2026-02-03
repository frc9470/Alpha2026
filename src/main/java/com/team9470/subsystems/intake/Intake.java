package com.team9470.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.IntakeSimulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem - controls pivot arm and roller.
 * This class contains only the logic that runs on both real robot and
 * simulation.
 * All physics simulation is handled by IntakeSimulation.
 */
public class Intake extends SubsystemBase {

    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    // Hardware
    private final TalonFX pivot = new TalonFX(Ports.INTAKE_PIVOT.getDeviceNumber());
    private final TalonFX roller = new TalonFX(Ports.INTAKE_ROLLER.getDeviceNumber());

    // Controls
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltRequest = new VoltageOut(0);

    // State
    private boolean deployed = false;

    private Intake() {
        // --- Pivot Config ---
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = IntakeConstants.kIntakePivotKp;
        pivotConfig.Slot0.kV = IntakeConstants.kIntakePivotKv;
        pivotConfig.Slot0.kA = IntakeConstants.kIntakePivotKa;
        pivotConfig.Slot0.kG = IntakeConstants.kIntakePivotKg;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Units
                .radiansToRotations(IntakeConstants.kIntakeMotionMagicCruiseVel);
        pivotConfig.MotionMagic.MotionMagicAcceleration = Units
                .radiansToRotations(IntakeConstants.kIntakeMotionMagicAccel);
        pivotConfig.MotionMagic.MotionMagicJerk = 0;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kIntakePivotGearRatio;

        pivot.getConfigurator().apply(pivotConfig);
        pivot.setPosition(0);

        // --- Roller Config ---
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        roller.getConfigurator().apply(rollerConfig);
    }

    // --- Commands ---

    public void setDeployed(boolean deployed) {
        this.deployed = deployed;
    }

    public Command getDeployCommand() {
        return this.startEnd(
                () -> setDeployed(true),
                () -> setDeployed(false))
                .withName("Intake Deploy");
    }

    @Override
    public void periodic() {
        // Pivot control
        double targetAngle = deployed ? IntakeConstants.kIntakeDeployAngle : IntakeConstants.kIntakeRetractAngle;
        double targetRot = Units.radiansToRotations(targetAngle);
        pivot.setControl(mmRequest.withPosition(targetRot));

        // Roller control
        if (deployed) {
            roller.setControl(voltRequest.withOutput(IntakeConstants.kIntakeRollerVoltage));
        } else {
            roller.setControl(voltRequest.withOutput(0));
        }

        // Telemetry
        SmartDashboard.putNumber("Intake/AngleDeg", Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()));
        SmartDashboard.putBoolean("Intake/Deployed", deployed);

        // Update visualization (works in both real and sim)
        if (Robot.isSimulation()) {
            IntakeSimulation.getInstance().updateVisualization(pivot.getPosition().getValueAsDouble());
        }
    }

    @Override
    public void simulationPeriodic() {
        IntakeSimulation.getInstance().update(pivot, roller, deployed);
    }

    // --- Accessors for physics ---

    public boolean isRunning() {
        return deployed;
    }

    public double getPivotAngle() {
        if (Robot.isSimulation()) {
            return IntakeSimulation.getInstance().getAngleRad();
        }
        return Units.rotationsToRadians(pivot.getPosition().getValueAsDouble());
    }
}
