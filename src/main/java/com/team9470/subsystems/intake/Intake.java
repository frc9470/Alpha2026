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
        // Apply configs from IntakeConstants
        pivot.getConfigurator().apply(IntakeConstants.kPivotConfig);
        pivot.setPosition(0);

        roller.getConfigurator().apply(IntakeConstants.kRollerConfig);
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
        double targetAngle = deployed ? IntakeConstants.kDeployAngle : IntakeConstants.kRetractAngle;
        double targetRot = IntakeConstants.pivotRadiansToMechanismRotations(targetAngle);
        pivot.setControl(mmRequest.withPosition(targetRot));

        // Roller control
        if (deployed) {
            roller.setControl(voltRequest.withOutput(IntakeConstants.kRollerVoltage));
        } else {
            roller.setControl(voltRequest.withOutput(0));
        }

        // Telemetry
        SmartDashboard.putNumber("Intake/AngleDeg",
                Math.toDegrees(IntakeConstants.pivotMechanismRotationsToRadians(pivot.getPosition().getValueAsDouble())));
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
        return IntakeConstants.pivotMechanismRotationsToRadians(pivot.getPosition().getValueAsDouble());
    }

    // ==================== HOMING ====================

    private boolean isHomed = false;
    private final VoltageOut homingVoltage = new VoltageOut(0);

    /**
     * Returns whether the pivot has been homed.
     */
    public boolean isHomed() {
        return isHomed;
    }

    /**
     * Command to home the pivot to hardstop (retract position).
     * Drives pivot at homing voltage until stall is detected, then zeros encoder.
     */
    public Command homePivotCommand() {
        return new edu.wpi.first.wpilibj2.command.FunctionalCommand(
                // Init
                () -> {
                    isHomed = false;
                },
                // Execute
                () -> {
                    pivot.setControl(homingVoltage.withOutput(IntakeConstants.kHomingVoltage));
                },
                // End
                (interrupted) -> {
                    pivot.setControl(homingVoltage.withOutput(0));
                    if (!interrupted) {
                        pivot.setPosition(IntakeConstants.kHomePosition);
                        isHomed = true;
                    }
                },
                // IsFinished
                () -> {
                    double current = pivot.getSupplyCurrent().getValueAsDouble();
                    return current > IntakeConstants.kStallCurrentThreshold;
                },
                this).withTimeout(3.0).withName("Intake Homing");
    }
}
