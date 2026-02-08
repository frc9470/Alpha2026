package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;
import com.team9470.Robot;
import com.team9470.simulation.ProjectileSimulation;
import com.team9470.util.AutoAim.ShootingSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // Status signals
    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Angle> hoodPosition;

    // State
    private double targetSpeedRPS = 0.0;
    private double targetHoodAngleRotations = 0.0;
    private boolean isFiring = false;
    private boolean needsHoming = true;

    // Simulation context
    private Supplier<Pose2d> poseSupplier = Pose2d::new;
    private Supplier<ChassisSpeeds> speedsSupplier = ChassisSpeeds::new;

    public Shooter() {
        // Configure motors
        TalonUtil.applyAndCheckConfiguration(flywheel1, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel2, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel3, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(flywheel4, ShooterConstants.kFlywheelConfig);
        TalonUtil.applyAndCheckConfiguration(hoodMotor, ShooterConstants.kHoodConfig);

        // Get status signals
        flywheelVelocity = flywheel1.getVelocity();
        hoodPosition = hoodMotor.getPosition();

        // Optimize CAN bus
        BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelVelocity, hoodPosition);
        flywheel1.optimizeBusUtilization();
        flywheel2.optimizeBusUtilization();
        flywheel3.optimizeBusUtilization();
        flywheel4.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();

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
        // Convert solution to motor setpoints
        // Exit velocity -> Flywheel RPS
        double exitSpeed = solution.speed();
        double wheelRadius = Units.inchesToMeters(2.0);
        double efficiency = ShooterConstants.kFlywheelEfficiency;
        this.targetSpeedRPS = exitSpeed / (efficiency * 2.0 * Math.PI * wheelRadius);

        // Pitch angle -> Hood rotations
        this.targetHoodAngleRotations = ShooterConstants.hoodRadiansToMechanismRotations(solution.pitch().getRadians());
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
        this.targetHoodAngleRotations = rotations;
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

        boolean flywheelReady = rpsError < 0.5;
        boolean hoodReady = hoodError < 0.01; // ~3.6 degrees

        return flywheelReady && hoodReady;
    }

    public double getCurrentFlywheelRPS() {
        if (Robot.isSimulation()) {
            return ProjectileSimulation.getInstance().getFlywheelVelocityRPS();
        }
        return ShooterConstants.flywheelMotorRpsToMechanismRps(flywheelVelocity.getValueAsDouble());
    }

    public double getCurrentHoodRotations() {
        if (Robot.isSimulation()) {
            return ShooterConstants
                    .hoodRadiansToMechanismRotations(ProjectileSimulation.getInstance().getHoodAngleRad());
        }
        return hoodPosition.getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (needsHoming) {
            // Drive hood toward min-angle hardstop
            hoodMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(ShooterConstants.kHoodHomingVoltage));

            // Keep flywheels stopped during homing
            flywheel1.setControl(flywheelRequest.withVelocity(0));
            flywheel2.setControl(flywheelRequest.withVelocity(0));
            flywheel3.setControl(flywheelRequest.withVelocity(0));
            flywheel4.setControl(flywheelRequest.withVelocity(0));

            // Check for stall (hit hardstop): high current AND low velocity
            double current = hoodMotor.getSupplyCurrent().getValueAsDouble();
            double velocity = Math.abs(hoodMotor.getVelocity().getValueAsDouble());
            if (current > ShooterConstants.kHoodStallCurrentThreshold && velocity < 0.5) {
                hoodMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0));
                hoodMotor.setPosition(ShooterConstants.hoodRadiansToMechanismRotations(
                        ShooterConstants.kHoodHomePosition.in(edu.wpi.first.units.Units.Radians)));
                needsHoming = false;
            }

            SmartDashboard.putBoolean("Shooter/Homing", true);
            return;
        }

        // --- Normal operation ---

        // Apply motor commands
        double motorRPS = ShooterConstants.flywheelMechanismRpsToMotorRps(targetSpeedRPS);
        flywheel1.setControl(flywheelRequest.withVelocity(motorRPS));
        flywheel2.setControl(flywheelRequest.withVelocity(motorRPS));
        flywheel3.setControl(flywheelRequest.withVelocity(motorRPS));
        flywheel4.setControl(flywheelRequest.withVelocity(motorRPS));

        hoodMotor.setControl(hoodRequest.withPosition(targetHoodAngleRotations));

        // Telemetry
        SmartDashboard.putBoolean("Shooter/Homing", false);
        SmartDashboard.putNumber("Shooter/TargetRPS", targetSpeedRPS);
        SmartDashboard.putNumber("Shooter/CurrentRPS", getCurrentFlywheelRPS());
        SmartDashboard.putNumber("Shooter/TargetHoodRot", targetHoodAngleRotations);
        SmartDashboard.putNumber("Shooter/CurrentHoodRot", getCurrentHoodRotations());
        SmartDashboard.putBoolean("Shooter/AtSetpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Shooter/Firing", isFiring);
    }

    @Override
    public void simulationPeriodic() {
        // Update projectile simulation
        ProjectileSimulation sim = ProjectileSimulation.getInstance();
        sim.update(0.02, isFiring, targetSpeedRPS, targetHoodAngleRotations);

        // Feed simulated values back to motor sim states
        double flywheelRPS = sim.getFlywheelVelocityRPS();
        flywheel1.getSimState().setRotorVelocity(ShooterConstants.flywheelMechanismRpsToMotorRps(flywheelRPS));

        double hoodAngleRad = sim.getHoodAngleRad();
        double hoodVelRadPerSec = sim.getHoodVelocityRadPerSec();
        hoodMotor.getSimState().setRawRotorPosition(
                ShooterConstants.hoodMechanismRotationsToMotorRotations(
                        ShooterConstants.hoodRadiansToMechanismRotations(hoodAngleRad)));
        hoodMotor.getSimState().setRotorVelocity(
                ShooterConstants.hoodMechanismRpsToMotorRps(
                        ShooterConstants.hoodRadiansToMechanismRotations(hoodVelRadPerSec)));

        // Telemetry
        SmartDashboard.putNumber("Shooter/MeasuredBPS", sim.getMeasuredBPS());
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
