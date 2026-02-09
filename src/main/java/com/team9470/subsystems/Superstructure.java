package com.team9470.subsystems;

import com.team9470.subsystems.hopper.Hopper;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.util.AutoAim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

/**
 * Superstructure manages all mechanisms (shooter, intake, hopper) and provides
 * high-level commands for coordinated behavior.
 */
public class Superstructure extends SubsystemBase {

    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    // Subsystems
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;

    // Context suppliers (set by RobotContainer)
    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<ChassisSpeeds> speedsSupplier = () -> new ChassisSpeeds();
    private Supplier<ChassisSpeeds> accelerationSupplier = () -> new ChassisSpeeds();

    private Superstructure() {
        shooter = new Shooter();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
    }

    /**
     * Set the pose and chassis speeds suppliers for aiming calculations.
     * Called by RobotContainer to connect swerve data.
     */
    public void setDriveContext(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        setDriveContext(pose, speeds, () -> new ChassisSpeeds());
    }

    /**
     * Set the pose, chassis speeds, and acceleration suppliers for aiming
     * calculations.
     * Acceleration enables predictive shooting.
     */
    public void setDriveContext(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds,
            Supplier<ChassisSpeeds> acceleration) {
        this.poseSupplier = pose;
        this.speedsSupplier = speeds;
        this.accelerationSupplier = acceleration;
        shooter.setSimulationContext(pose, speeds);
    }

    @Override
    public void periodic() {
        // No unconditional setSetpoint here - AutoAim setpoints are applied
        // only by shooting commands (shootCommand, aimAndShootCommand).
        // This prevents overriding manual flywheel/hood commands (e.g. debug Y button).
    }

    // ==================== HIGH-LEVEL COMMANDS ====================

    /**
     * Intake command - deploys intake and runs rollers while held.
     */
    public Command intakeCommand() {
        return intake.getIntakeCommand()
                .withName("Superstructure Intake");
    }

    /**
     * Toggle intake arm deployed/retracted.
     */
    public Command toggleIntakeCommand() {
        return intake.getToggleCommand()
                .withName("Superstructure Toggle Intake");
    }

    /**
     * Outtake command - reverses rollers while held.
     */
    public Command outtakeCommand() {
        return intake.getOuttakeCommand()
                .withName("Superstructure Outtake");
    }

    /**
     * Shoot command - fires when aligned.
     * Returns the target rotation rate for swerve to apply.
     */
    public Command shootCommand(Supplier<Double> rotationRateConsumer) {
        return Commands.run(() -> {
            var solution = AutoAim.calculate(poseSupplier.get(), speedsSupplier.get());
            shooter.setSetpoint(solution);
            double rotError = solution.targetRobotYaw()
                    .minus(poseSupplier.get().getRotation())
                    .getRadians();
            double rotCmd = (rotError * 12.0) + solution.targetOmega();

            // Check alignment
            boolean isAligned = Math.abs(rotError) < Math.toRadians(3.0);
            boolean canFire = isAligned && solution.isValid() && shooter.isAtSetpoint();

            // Control shooter and hopper
            shooter.setFiring(canFire);
            hopper.setRunning(canFire);

            // Telemetry
            SmartDashboard.putBoolean("Superstructure/Aligned", isAligned);
            SmartDashboard.putBoolean("Superstructure/CanFire", canFire);
            SmartDashboard.putNumber("Superstructure/RotError", Math.toDegrees(rotError));
            SmartDashboard.putNumber("Superstructure/RotCmd", rotCmd);

        }, this).finallyDo(() -> {
            shooter.setFiring(false);
            hopper.stop();
        }).withName("Superstructure Shoot");
    }

    /**
     * Aim and shoot command - calculates aim and returns rotation command for
     * swerve.
     * Use with swerve to get the rotation rate.
     */
    public AimResult getAimResult() {
        var solution = AutoAim.calculate(poseSupplier.get(), speedsSupplier.get(), accelerationSupplier.get());

        double rotError = solution.targetRobotYaw()
                .minus(poseSupplier.get().getRotation())
                .getRadians();
        double rotCmd = (rotError * 12.0) + solution.targetOmega();
        boolean isAligned = Math.abs(rotError) < Math.toRadians(3.0);

        return new AimResult(rotCmd, isAligned, solution);
    }

    /**
     * Result of aiming calculation for swerve to use.
     */
    public record AimResult(
            double rotationCommand,
            boolean isAligned,
            AutoAim.ShootingSolution solution) {
    }

    /**
     * Command that handles shooting coordination.
     * Call getAimResult() to get rotation command for swerve.
     */
    public Command aimAndShootCommand() {
        return Commands.run(() -> {
            var result = getAimResult();
            shooter.setSetpoint(result.solution());
            boolean canFire = result.isAligned() && result.solution().isValid() && shooter.isAtSetpoint();

            // Control shooter and hopper based on alignment
            shooter.setFiring(canFire);
            hopper.setRunning(canFire);

            // Telemetry
            SmartDashboard.putBoolean("Superstructure/Aligned", result.isAligned());
            SmartDashboard.putBoolean("Superstructure/CanFire", canFire);
            SmartDashboard.putNumber("Superstructure/RotCmd", result.rotationCommand());

        }, this).finallyDo(() -> {
            shooter.setFiring(false);
            hopper.stop();
        }).withName("Superstructure AimAndShoot");
    }

    /**
     * Idle command - stops all mechanisms.
     */
    public Command idleCommand() {
        return Commands.runOnce(() -> {
            shooter.setFiring(false);
            hopper.stop();
        }, this).withName("Superstructure Idle");
    }

    // ==================== ACCESSORS ====================

    public Shooter getShooter() {
        return shooter;
    }

    public Intake getIntake() {
        return intake;
    }

    public Hopper getHopper() {
        return hopper;
    }
}
