package com.team9470.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team9470.Constants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake m_instance;

    private final TalonFX m_pivot = new TalonFX(Constants.IntakeConstants.kIntakePivotId);
    private final TalonFX m_roller = new TalonFX(Constants.IntakeConstants.kIntakeRollerId);

    private final SingleJointedArmSim m_pivotSim;

    // Controls
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut m_voltReq = new VoltageOut(0);

    // State
    private boolean m_deployed = false;

    private Intake() {
        // --- Pivot Config ---
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = Constants.IntakeConstants.kIntakePivotKp;
        pivotConfig.Slot0.kV = Constants.IntakeConstants.kIntakePivotKv;
        pivotConfig.Slot0.kA = Constants.IntakeConstants.kIntakePivotKa;
        pivotConfig.Slot0.kG = Constants.IntakeConstants.kIntakePivotKg; // Gravity compensation

        // Wait, Phoenix 6 PID is in Rotations. Constants are likely Rads logic usually.
        // Let's assume Constants values are appropriate directly or convert.
        // If Constant is 4 rad/s -> ~0.63 rot/s. Phoenix expects RPS.
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Units
                .radiansToRotations(Constants.IntakeConstants.kIntakeMotionMagicCruiseVel);
        pivotConfig.MotionMagic.MotionMagicAcceleration = Units
                .radiansToRotations(Constants.IntakeConstants.kIntakeMotionMagicAccel);
        pivotConfig.MotionMagic.MotionMagicJerk = 0; // Disable Jerk (infinite)

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.kIntakePivotGearRatio;

        m_pivot.getConfigurator().apply(pivotConfig);
        m_pivot.setPosition(0); // Start retracted

        // --- Roller Config ---
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_roller.getConfigurator().apply(rollerConfig);

        // --- Sim ---
        m_pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                Constants.IntakeConstants.kIntakePivotGearRatio,
                SingleJointedArmSim.estimateMOI(Constants.IntakeConstants.kIntakeLength,
                        Constants.IntakeConstants.kIntakeMass),
                Constants.IntakeConstants.kIntakeLength,
                0.0, // Min Angle
                Math.PI, // Max Angle
                true, // Simulate gravity
                0.0 // Start angle
        );
    }

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake();
        }
        return m_instance;
    }

    // --- Commands ---
    public void setDeployed(boolean deployed) {
        m_deployed = deployed;
    }

    public Command getDeployCommand() {
        return this.startEnd(
                () -> setDeployed(true),
                () -> setDeployed(false));
    }

    @Override
    public void periodic() {
        // Control Logic
        double targetAngle = m_deployed ? Constants.IntakeConstants.kIntakeDeployAngle
                : Constants.IntakeConstants.kIntakeRetractAngle;
        double targetRot = Units.radiansToRotations(targetAngle);

        m_pivot.setControl(m_mmReq.withPosition(targetRot));

        // Roller Logic
        if (m_deployed) {
            m_roller.setControl(m_voltReq.withOutput(Constants.IntakeConstants.kIntakeRollerVoltage));
        } else {
            m_roller.setControl(m_voltReq.withOutput(0));
        }

        // Telemetry
        SmartDashboard.putNumber("Intake/AngleDeg", Units.rotationsToDegrees(m_pivot.getPosition().getValueAsDouble()));
        SmartDashboard.putBoolean("Intake/Deployed", m_deployed);

        // Update Visualization
        m_arm.setAngle(Units.rotationsToDegrees(m_pivot.getPosition().getValueAsDouble()));
    }

    @Override
    public void simulationPeriodic() {
        // Feed voltage to Sim
        double voltage = m_pivot.getSimState().getMotorVoltage();
        m_pivotSim.setInput(voltage);
        m_pivotSim.update(0.020);

        // Update Motor Sim State
        m_pivot.getSimState().setRawRotorPosition(
                Units.radiansToRotations(m_pivotSim.getAngleRads()) * Constants.IntakeConstants.kIntakePivotGearRatio);
        m_pivot.getSimState().setRotorVelocity(
                Units.radiansToRotations(m_pivotSim.getVelocityRadPerSec())
                        * Constants.IntakeConstants.kIntakePivotGearRatio);

        // Update Roller Sim Speed (Basic)
        m_roller.getSimState().setSupplyVoltage(12.0);
        m_roller.getSimState().setRotorVelocity(m_deployed ? 60.0 : 0.0); // 60 RPS ~ 3600 RPM
    }

    // --- Visualization ---
    private final edu.wpi.first.wpilibj.smartdashboard.Mechanism2d m_mech = new edu.wpi.first.wpilibj.smartdashboard.Mechanism2d(
            1.0, 1.0);
    private final edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d m_root = m_mech.getRoot("Pivot", 0.35, 0.2); // x=0.35,
                                                                                                                    // z=0.2
                                                                                                                    // based
                                                                                                                    // on
                                                                                                                    // sim
    private final edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d m_arm = m_root.append(
            new edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d("Arm", Constants.IntakeConstants.kIntakeLength,
                    90));

    {
        SmartDashboard.putData("Intake/Mechanism2d", m_mech);
        m_arm.setColor(new edu.wpi.first.wpilibj.util.Color8Bit(edu.wpi.first.wpilibj.util.Color.kYellow));
    }

    // --- Physics Interface ---
    public boolean isRunning() {
        return m_deployed; // Simplified: "Running" if deployed
    }

    public double getPivotAngle() {
        return Units.rotationsToRadians(m_pivot.getPosition().getValueAsDouble());
    }
}
