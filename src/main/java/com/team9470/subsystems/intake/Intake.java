package com.team9470.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.IntakeSimulation;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.IntakeSnapshot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

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
    private final TalonFX pivot = TalonFXFactory.createDefaultTalon(Ports.INTAKE_PIVOT);
    private final TalonFX roller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_ROLLER);

    // Controls
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltRequest = new VoltageOut(0);

    // State
    private boolean deployed = false;
    private boolean agitating = false;
    private boolean needsHoming = true;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    private static final int STATE_HOMING = 0;
    private static final int STATE_RETRACTED = 1;
    private static final int STATE_DEPLOYED = 2;
    private static final int STATE_AGITATE_DEPLOY = 3;
    private static final int STATE_AGITATE_MID = 4;

    private Intake() {
        // Apply configurations
        TalonUtil.applyAndCheckConfiguration(pivot, IntakeConstants.kPivotConfig);
        pivot.setPosition(0);

        TalonUtil.applyAndCheckConfiguration(roller, IntakeConstants.kRollerConfig);
    }

    // --- Commands ---

    public void setDeployed(boolean deployed) {
        this.deployed = deployed;
    }

    public void setAgitating(boolean agitating) {
        this.agitating = agitating;
    }

    /** Toggle deploy/retract arm position. */
    public Command getToggleCommand() {
        return this.runOnce(() -> setDeployed(!deployed))
                .withName("Intake Toggle");
    }

    /** Deploy + run rollers while held, retract on release. */
    public Command getIntakeCommand() {
        return this.startEnd(
                () -> setDeployed(true),
                () -> setDeployed(false))
                .withName("Intake Run");
    }

    /** Reverse rollers while held (for clearing jams). */
    public Command getOuttakeCommand() {
        return this.startEnd(
                () -> roller.setControl(voltRequest.withOutput(-IntakeConstants.kRollerVoltage)),
                () -> roller.setControl(voltRequest.withOutput(0)))
                .withName("Intake Outtake");
    }

    @Override
    public void periodic() {
        if (needsHoming) {
            // Drive toward retract hardstop
            pivot.setControl(voltRequest.withOutput(IntakeConstants.kHomingVoltage));
            roller.setControl(voltRequest.withOutput(0));

            // Check for stall (hit hardstop): high current AND low velocity
            double current = pivot.getSupplyCurrent().getValueAsDouble();
            double velocity = Math.abs(pivot.getVelocity().getValueAsDouble());
            if (current > IntakeConstants.kStallCurrentThreshold && velocity < 0.5) {
                pivot.setControl(voltRequest.withOutput(0));
                pivot.setPosition(IntakeConstants.pivotAngleToMechanismRotations(IntakeConstants.kHomePosition));
                needsHoming = false;
            }

            telemetry.publishIntakeState(new IntakeSnapshot(
                    true,
                    deployed,
                    agitating,
                    STATE_HOMING,
                    0.0,
                    0.0,
                    IntakeConstants.pivotMechanismRotationsToAngle(pivot.getPosition().getValueAsDouble()).in(Radians),
                    0.0,
                    velocity * 2.0 * Math.PI,
                    current,
                    IntakeConstants.kHomingVoltage,
                    0.0,
                    roller.getSupplyCurrent().getValueAsDouble()));
            return;
        }

        // --- Normal operation ---

        // Pivot control
        boolean agitateAtDeploy = false;
        if (agitating) {
            double t = Timer.getFPGATimestamp();
            agitateAtDeploy = Math.sin(2.0 * Math.PI * IntakeConstants.kAgitateFrequencyHz * t) >= 0.0;
        }
        Angle targetAngle;
        if (agitating) {
            targetAngle = agitateAtDeploy ? IntakeConstants.kDeployAngle : IntakeConstants.kAgitateMiddleAngle;
        } else {
            targetAngle = deployed ? IntakeConstants.kDeployAngle : IntakeConstants.kRetractAngle;
        }
        double targetRot = IntakeConstants.pivotAngleToMechanismRotations(targetAngle);
        pivot.setControl(mmRequest.withPosition(targetRot));

        // Roller control
        double rollerVolts = (deployed || agitating) ? IntakeConstants.kRollerVoltage : 0.0;
        roller.setControl(voltRequest.withOutput(rollerVolts));

        // --- Telemetry ---
        double currentPositionRot = pivot.getPosition().getValueAsDouble();
        double currentPositionRad = IntakeConstants.pivotMechanismRotationsToAngle(currentPositionRot)
                .in(Radians);
        double goalRad = targetAngle.in(Radians);
        double setpointRad = IntakeConstants.pivotMechanismRotationsToAngle(targetRot).in(Radians);

        int stateCode;
        if (agitating) {
            stateCode = agitateAtDeploy ? STATE_AGITATE_DEPLOY : STATE_AGITATE_MID;
        } else {
            stateCode = deployed ? STATE_DEPLOYED : STATE_RETRACTED;
        }
        telemetry.publishIntakeState(new IntakeSnapshot(
                false,
                deployed,
                agitating,
                stateCode,
                goalRad,
                setpointRad,
                currentPositionRad,
                setpointRad - currentPositionRad,
                pivot.getVelocity().getValueAsDouble() * 2.0 * Math.PI,
                pivot.getSupplyCurrent().getValueAsDouble(),
                pivot.getMotorVoltage().getValueAsDouble(),
                rollerVolts,
                roller.getSupplyCurrent().getValueAsDouble()));

        // Update visualization (works in both real and sim)
        if (Robot.isSimulation()) {
            IntakeSimulation.getInstance().updateVisualization(currentPositionRot);
        }
    }

    @Override
    public void simulationPeriodic() {
        IntakeSimulation.getInstance().update(pivot, roller, deployed || agitating);
    }

    // --- Accessors for physics ---

    public boolean isRunning() {
        return deployed || agitating;
    }

    public boolean isDeployed() {
        return deployed;
    }

    public boolean isAgitating() {
        return agitating;
    }

    public double getPivotAngle() {
        if (Robot.isSimulation()) {
            return IntakeSimulation.getInstance().getAngleRad();
        }
        return IntakeConstants.pivotMechanismRotationsToAngle(pivot.getPosition().getValueAsDouble())
                .in(Radians);
    }

    /**
     * Returns whether homing is complete.
     */
    public boolean isHomed() {
        return !needsHoming;
    }
}
