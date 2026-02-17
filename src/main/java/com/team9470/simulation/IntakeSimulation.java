package com.team9470.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team9470.subsystems.intake.IntakeConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Handles all intake simulation - arm physics and visualization.
 * Separate from Intake subsystem to keep sim logic out of robot code.
 */
public class IntakeSimulation {

    private static IntakeSimulation instance;

    public static IntakeSimulation getInstance() {
        if (instance == null) {
            instance = new IntakeSimulation();
        }
        return instance;
    }

    // Physics
    private final SingleJointedArmSim pivotSim;

    // Visualization
    private final Mechanism2d mech;
    private final MechanismLigament2d arm;

    private IntakeSimulation() {
        // Arm physics simulation
        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                IntakeConstants.kPivotGearRatio,
                SingleJointedArmSim.estimateMOI(IntakeConstants.kIntakeLength, IntakeConstants.kIntakeMass),
                IntakeConstants.kIntakeLength,
                0.0, // Min Angle
                Math.PI, // Max Angle
                true, // Simulate gravity
                0.0); // Start angle

        // Mechanism2d visualization
        mech = new Mechanism2d(1.0, 1.0);
        MechanismRoot2d root = mech.getRoot("Pivot", 0.35, 0.2);
        arm = root.append(new MechanismLigament2d("Arm", IntakeConstants.kIntakeLength, 90));
        arm.setColor(new Color8Bit(Color.kYellow));
    }

    /**
     * Update simulation - call from Intake.simulationPeriodic().
     */
    public void update(TalonFX pivotMotor, TalonFX rollerMotor, boolean deployed) {
        // Feed voltage to sim
        double voltage = pivotMotor.getSimState().getMotorVoltage();
        pivotSim.setInput(voltage);
        pivotSim.update(0.020);

        // Update motor sim state
        pivotMotor.getSimState().setRawRotorPosition(
                Units.radiansToRotations(pivotSim.getAngleRads()) * IntakeConstants.kPivotGearRatio);
        pivotMotor.getSimState().setRotorVelocity(
                Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * IntakeConstants.kPivotGearRatio);

        // Update roller sim state (basic)
        rollerMotor.getSimState().setSupplyVoltage(12.0);
        rollerMotor.getSimState().setRotorVelocity(deployed ? 60.0 : 0.0); // 60 RPS ~ 3600 RPM

        // Update visualization
        arm.setAngle(Math.toDegrees(pivotSim.getAngleRads()));
    }

    /**
     * Update visualization only (for periodic, not sim).
     */
    public void updateVisualization(double angleRotations) {
        arm.setAngle(Units.rotationsToDegrees(angleRotations));
    }

    // Accessors
    public double getAngleRad() {
        return pivotSim.getAngleRads();
    }

    public double getVelocityRadPerSec() {
        return pivotSim.getVelocityRadPerSec();
    }
}
