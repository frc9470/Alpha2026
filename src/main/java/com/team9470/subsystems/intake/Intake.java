package com.team9470.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.IntakeSimulation;
import com.team9470.util.TelemetryUtil;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
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
    private boolean needsHoming = true;

    // ==================== TELEMETRY ====================
    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("Intake");

    // State publishers
    private final BooleanPublisher ntHoming = nt.getBooleanTopic("Homing").publish();
    private final BooleanPublisher ntDeployed = nt.getBooleanTopic("Deployed").publish();
    private final StringPublisher ntState = nt.getStringTopic("State").publish();

    // Pivot position (degrees)
    private final DoublePublisher ntGoalDeg = TelemetryUtil.publishDouble(nt, "Pivot/GoalDeg", "deg");
    private final DoublePublisher ntSetpointDeg = TelemetryUtil.publishDouble(nt, "Pivot/SetpointDeg", "deg");
    private final DoublePublisher ntPositionDeg = TelemetryUtil.publishDouble(nt, "Pivot/PositionDeg", "deg");
    private final DoublePublisher ntErrorDeg = TelemetryUtil.publishDouble(nt, "Pivot/ErrorDeg", "deg");

    // Pivot motor signals
    private final DoublePublisher ntPivotVelocity = TelemetryUtil.publishDouble(nt, "Pivot/VelocityDegPerSec", "deg/s");
    private final DoublePublisher ntPivotCurrent = TelemetryUtil.publishDouble(nt, "Pivot/SupplyCurrentAmps", "A");
    private final DoublePublisher ntPivotVoltage = TelemetryUtil.publishDouble(nt, "Pivot/AppliedVolts", "V");

    // Roller signals
    private final DoublePublisher ntRollerVoltage = TelemetryUtil.publishDouble(nt, "Roller/AppliedVolts", "V");
    private final DoublePublisher ntRollerCurrent = TelemetryUtil.publishDouble(nt, "Roller/SupplyCurrentAmps", "A");

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

            // Homing telemetry
            ntHoming.set(true);
            ntDeployed.set(deployed);
            ntState.set("HOMING");
            ntPivotCurrent.set(current);
            ntPivotVelocity.set(velocity * 360.0); // rot/s -> deg/s
            ntPivotVoltage.set(IntakeConstants.kHomingVoltage);
            return;
        }

        // --- Normal operation ---

        // Pivot control
        Angle targetAngle = deployed ? IntakeConstants.kDeployAngle : IntakeConstants.kRetractAngle;
        double targetRot = IntakeConstants.pivotAngleToMechanismRotations(targetAngle);
        pivot.setControl(mmRequest.withPosition(targetRot));

        // Roller control
        double rollerVolts = deployed ? IntakeConstants.kRollerVoltage : 0.0;
        roller.setControl(voltRequest.withOutput(rollerVolts));

        // --- Telemetry ---
        double currentPositionRot = pivot.getPosition().getValueAsDouble();
        double currentPositionDeg = IntakeConstants.pivotMechanismRotationsToAngle(currentPositionRot)
                .in(Degrees);
        double goalDeg = targetAngle.in(Degrees);
        double setpointDeg = IntakeConstants.pivotMechanismRotationsToAngle(targetRot).in(Degrees);

        ntHoming.set(false);
        ntDeployed.set(deployed);
        ntState.set(deployed ? "DEPLOYED" : "RETRACTED");

        ntGoalDeg.set(goalDeg);
        ntSetpointDeg.set(setpointDeg);
        ntPositionDeg.set(currentPositionDeg);
        ntErrorDeg.set(setpointDeg - currentPositionDeg);

        ntPivotVelocity.set(pivot.getVelocity().getValueAsDouble() * 360.0);
        ntPivotCurrent.set(pivot.getSupplyCurrent().getValueAsDouble());
        ntPivotVoltage.set(pivot.getMotorVoltage().getValueAsDouble());

        ntRollerVoltage.set(rollerVolts);
        ntRollerCurrent.set(roller.getSupplyCurrent().getValueAsDouble());

        // Update visualization (works in both real and sim)
        if (Robot.isSimulation()) {
            IntakeSimulation.getInstance().updateVisualization(currentPositionRot);
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
