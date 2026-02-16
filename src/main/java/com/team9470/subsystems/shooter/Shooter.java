package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.ProjectileSimulation;
import com.team9470.util.AutoAim.ShootingSolution;
import com.team9470.util.TelemetryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

/**
 * Shooter subsystem - controls flywheel velocity and hood angle.
 * This class contains only the logic that runs on both real robot and
 * simulation.
 * All physics simulation is handled by ProjectileSimulation.
 */
public class Shooter extends SubsystemBase {

    // Hardware
    private final TalonFX flywheel1 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_1);
    private final TalonFX flywheel2 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_2);
    private final TalonFX flywheel3 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_3);
    private final TalonFX flywheel4 = TalonFXFactory.createDefaultTalon(Ports.FLYWHEEL_4);
    private final TalonFX hoodMotor = TalonFXFactory.createDefaultTalon(Ports.HOOD_MOTOR);

    // Controls
    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0);
    private final PositionVoltage hoodRequest = new PositionVoltage(0);
    private final VoltageOut voltRequest = new VoltageOut(0);

    // State
    private double targetSpeedRPS = 0.0;
    private double targetHoodAngleRotations = ShooterConstants.launchRadToMechanismRotations(
            ShooterConstants.kHoodHomePosition.in(edu.wpi.first.units.Units.Radians)); // Launch angle (mechanism
                                                                                       // rotations)
    private boolean isFiring = false;
    private boolean needsHoming = true;

    // Simulation context
    private Supplier<Pose2d> poseSupplier = Pose2d::new;
    private Supplier<ChassisSpeeds> speedsSupplier = ChassisSpeeds::new;

    // ==================== TELEMETRY ====================
    private final NetworkTable nt = NetworkTableInstance.getDefault().getTable("Shooter");

    // State
    private final BooleanPublisher ntHoming = nt.getBooleanTopic("Homing").publish();
    private final BooleanPublisher ntFiring = nt.getBooleanTopic("Firing").publish();
    private final BooleanPublisher ntAtSetpoint = nt.getBooleanTopic("AtSetpoint").publish();
    private final StringPublisher ntState = nt.getStringTopic("State").publish();

    // Hood position (degrees)
    private final DoublePublisher ntHoodGoalDeg = TelemetryUtil.publishDouble(nt, "Hood/GoalDeg", "deg");
    private final DoublePublisher ntHoodSetpointDeg = TelemetryUtil.publishDouble(nt, "Hood/SetpointDeg", "deg");
    private final DoublePublisher ntHoodPositionDeg = TelemetryUtil.publishDouble(nt, "Hood/PositionDeg", "deg");
    private final DoublePublisher ntHoodErrorDeg = TelemetryUtil.publishDouble(nt, "Hood/ErrorDeg", "deg");

    // Hood motor signals
    private final DoublePublisher ntHoodVelocity = TelemetryUtil.publishDouble(nt, "Hood/VelocityDegPerSec", "deg/s");
    private final DoublePublisher ntHoodStatorCurrent = TelemetryUtil.publishDouble(nt, "Hood/StatorCurrentAmps", "A");
    private final DoublePublisher ntHoodCurrent = TelemetryUtil.publishDouble(nt, "Hood/SupplyCurrentAmps", "A");
    private final DoublePublisher ntHoodVoltage = TelemetryUtil.publishDouble(nt, "Hood/AppliedVolts", "V");

    // Flywheel
    private final DoublePublisher ntFlywheelTargetRPS = TelemetryUtil.publishDouble(nt, "Flywheel/TargetRPS", "rps");
    private final DoublePublisher ntFlywheelCurrentRPS = TelemetryUtil.publishDouble(nt, "Flywheel/CurrentRPS", "rps");
    private final DoublePublisher ntFlywheelErrorRPS = TelemetryUtil.publishDouble(nt, "Flywheel/ErrorRPS", "rps");
    private final DoublePublisher ntFlywheelVoltage = TelemetryUtil.publishDouble(nt, "Flywheel/AppliedVolts", "V");
    private final DoublePublisher ntFlywheelSupplyCurrent = TelemetryUtil.publishDouble(nt,
            "Flywheel/SupplyCurrentAmps", "A");
    private final DoublePublisher ntFlywheelStatorCurrent = TelemetryUtil.publishDouble(nt,
            "Flywheel/StatorCurrentAmps",
            "A");
    private final DoublePublisher ntSimMeasuredBPS = TelemetryUtil.publishDouble(nt, "Sim/MeasuredBPS", "bps");

    /** Clamp hood rotations to valid launch-angle range. */
    private static double clampHoodRotations(double rotations) {
        double minRot = ShooterConstants.launchRadToMechanismRotations(
                ShooterConstants.kMinHoodAngle.in(edu.wpi.first.units.Units.Radians));
        double maxRot = ShooterConstants.launchRadToMechanismRotations(
                ShooterConstants.kMaxHoodAngle.in(edu.wpi.first.units.Units.Radians));
        return Math.max(minRot, Math.min(maxRot, rotations));
    }

    public Shooter() {
        // Configure motors
        TalonUtil.applyAndCheckConfiguration(flywheel1, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel2, ShooterConstants.kFlywheelInvertedConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel3, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel4, ShooterConstants.kFlywheelInvertedConfig);
        TalonUtil.applyAndCheckConfiguration(hoodMotor, ShooterConstants.kHoodConfig);

        // Initialize simulation if needed
        if (Robot.isSimulation()) {
            initSimulation();
        }
    }

    private void initSimulation() {
        ProjectileSimulation.getInstance().setContext(
                poseSupplier,
                speedsSupplier,
                () -> flywheel1.getSimState().getMotorVoltage(),
                () -> hoodMotor.getSimState().getMotorVoltage());
    }

    /**
     * Set robot context for aiming calculations.
     */
    public void setSimulationContext(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        this.poseSupplier = pose;
        this.speedsSupplier = speeds;
        if (Robot.isSimulation()) {
            initSimulation();
        }
    }

    /**
     * Set target shooting solution from AutoAim.
     */
    public void setSetpoint(ShootingSolution solution) {
        // Flywheel uses direct map RPM command.
        this.targetSpeedRPS = solution.flywheelRpm() / 60.0;

        // Hood map uses commanded hood plane angle in degrees.
        double launchRad = Math.toRadians(solution.hoodCommandDeg());
        this.targetHoodAngleRotations = clampHoodRotations(
                ShooterConstants.launchRadToMechanismRotations(launchRad));
    }

    /**
     * Set flywheel speed directly (RPS).
     */
    public void setFlywheelSpeed(double rps) {
        this.targetSpeedRPS = rps;
    }

    /**
     * Set hood angle directly (rotations).
     */
    public void setHoodAngle(double rotations) {
        this.targetHoodAngleRotations = clampHoodRotations(rotations);
    }

    /**
     * Enable/disable firing mode.
     */
    public void setFiring(boolean firing) {
        this.isFiring = firing;
    }

    public boolean isFiring() {
        return isFiring;
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        targetSpeedRPS = 0.0;
        isFiring = false;
    }

    /**
     * Check if shooter is at setpoint (ready to fire).
     */
    public boolean isAtSetpoint() {
        double currentRPS = getCurrentFlywheelRPS();
        double rpsError = Math.abs(currentRPS - targetSpeedRPS);

        double currentHoodRot = getCurrentHoodRotations();
        double hoodError = Math.abs(currentHoodRot - targetHoodAngleRotations);

        boolean flywheelReady = rpsError < 1; // 60 RPM
        boolean hoodReady = hoodError < 0.01; // ~3.6 degrees

        return flywheelReady && hoodReady;
    }

    public double getCurrentFlywheelRPS() {
        if (Robot.isSimulation()) {
            return ProjectileSimulation.getInstance().getFlywheelVelocityRPS();
        }
        return flywheel1.getVelocity().getValueAsDouble();
    }

    public double getCurrentHoodRotations() {
        if (Robot.isSimulation()) {
            // Sim returns launch angle in rad.
            double launchRad = ProjectileSimulation.getInstance().getHoodAngleRad();
            return ShooterConstants.launchRadToMechanismRotations(launchRad);
        }
        return hoodMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // === Flywheel control (always runs, independent of homing) ===
        flywheel1.setControl(flywheelRequest.withVelocity(targetSpeedRPS));
        flywheel2.setControl(flywheelRequest.withVelocity(targetSpeedRPS));
        flywheel3.setControl(flywheelRequest.withVelocity(targetSpeedRPS));
        flywheel4.setControl(flywheelRequest.withVelocity(targetSpeedRPS));

        // === Hood control (gated by homing) ===
        if (needsHoming) {
            // Drive hood toward min-angle hardstop
            hoodMotor.setControl(voltRequest.withOutput(ShooterConstants.kHoodHomingVoltage));

            // Check for stall (hit hardstop): high current AND low velocity
            double current = hoodMotor.getStatorCurrent().getValueAsDouble();
            double velocity = Math.abs(hoodMotor.getVelocity().getValueAsDouble());
            if (current > ShooterConstants.kHoodStallCurrentThreshold && velocity < 0.5) {
                hoodMotor.setControl(voltRequest.withOutput(0));
                hoodMotor.setPosition(ShooterConstants.launchRadToMechanismRotations(
                        ShooterConstants.kHoodHomePosition.in(edu.wpi.first.units.Units.Radians)));
                needsHoming = false;
            }
        } else {
            // Normal hood control
            hoodMotor.setControl(hoodRequest.withPosition(targetHoodAngleRotations));
        }

        // === Telemetry (always runs) ===
        double currentHoodRot = getCurrentHoodRotations();
        double currentLaunchRad = ShooterConstants.mechanismRotationsToLaunchRad(currentHoodRot);
        double targetLaunchRad = ShooterConstants.mechanismRotationsToLaunchRad(targetHoodAngleRotations);
        double currentHoodDeg = Math.toDegrees(currentLaunchRad);
        double targetHoodDeg = Math.toDegrees(targetLaunchRad);
        double currentRPS = getCurrentFlywheelRPS();

        // State
        ntHoming.set(needsHoming);
        ntFiring.set(isFiring);
        ntAtSetpoint.set(!needsHoming && isAtSetpoint());
        ntState.set(needsHoming ? "HOMING" : (isFiring ? "FIRING" : (targetSpeedRPS > 0 ? "SPINNING_UP" : "IDLE")));

        // Hood
        ntHoodGoalDeg.set(targetHoodDeg);
        ntHoodSetpointDeg.set(targetHoodDeg);
        ntHoodPositionDeg.set(currentHoodDeg);
        ntHoodErrorDeg.set(targetHoodDeg - currentHoodDeg);
        ntHoodVelocity.set(hoodMotor.getVelocity().getValueAsDouble() * 360.0);
        ntHoodStatorCurrent.set(hoodMotor.getStatorCurrent().getValueAsDouble());
        ntHoodCurrent.set(hoodMotor.getSupplyCurrent().getValueAsDouble());
        ntHoodVoltage.set(hoodMotor.getMotorVoltage().getValueAsDouble());

        // Flywheel
        double avgFlywheelVoltage = (flywheel1.getMotorVoltage().getValueAsDouble()
                + flywheel2.getMotorVoltage().getValueAsDouble()
                + flywheel3.getMotorVoltage().getValueAsDouble()
                + flywheel4.getMotorVoltage().getValueAsDouble()) / 4.0;
        double avgFlywheelSupplyCurrent = (flywheel1.getSupplyCurrent().getValueAsDouble()
                + flywheel2.getSupplyCurrent().getValueAsDouble()
                + flywheel3.getSupplyCurrent().getValueAsDouble()
                + flywheel4.getSupplyCurrent().getValueAsDouble()) / 4.0;
        double avgFlywheelStatorCurrent = (flywheel1.getStatorCurrent().getValueAsDouble()
                + flywheel2.getStatorCurrent().getValueAsDouble()
                + flywheel3.getStatorCurrent().getValueAsDouble()
                + flywheel4.getStatorCurrent().getValueAsDouble()) / 4.0;
        ntFlywheelTargetRPS.set(targetSpeedRPS);
        ntFlywheelCurrentRPS.set(currentRPS);
        ntFlywheelErrorRPS.set(targetSpeedRPS - currentRPS);
        ntFlywheelVoltage.set(avgFlywheelVoltage);
        ntFlywheelSupplyCurrent.set(avgFlywheelSupplyCurrent);
        ntFlywheelStatorCurrent.set(avgFlywheelStatorCurrent);
    }

    @Override
    public void simulationPeriodic() {
        // Update projectile simulation
        ProjectileSimulation sim = ProjectileSimulation.getInstance();
        sim.update(0.02, isFiring, targetSpeedRPS, targetHoodAngleRotations);

        // Feed simulated values back to motor sim states
        double flywheelRPS = sim.getFlywheelVelocityRPS();
        // Sim API needs rotor (motor) velocity, so we convert back to motor space
        flywheel1.getSimState().setRotorVelocity(flywheelRPS * ShooterConstants.kFlywheelGearRatio);

        // Sim returns launch angle in rad -> convert to motor rotor position
        double launchAngleRad = sim.getHoodAngleRad();
        double mechanismRot = ShooterConstants.launchRadToMechanismRotations(launchAngleRad);
        hoodMotor.getSimState().setRawRotorPosition(mechanismRot * ShooterConstants.kHoodGearRatio);

        double launchVelRadPerSec = sim.getHoodVelocityRadPerSec();
        double launchVelRot = Units.radiansToRotations(launchVelRadPerSec);
        hoodMotor.getSimState().setRotorVelocity(launchVelRot * ShooterConstants.kHoodGearRatio);

        // Sim telemetry
        ntSimMeasuredBPS.set(sim.getMeasuredBPS());
    }

    /**
     * Debug: Seed field with game pieces.
     */
    public void seedField() {
        if (Robot.isSimulation()) {
            ProjectileSimulation.getInstance().seedField();
        }
    }

    /**
     * Returns whether homing is complete.
     */
    public boolean isHomed() {
        return !needsHoming;
    }
}
