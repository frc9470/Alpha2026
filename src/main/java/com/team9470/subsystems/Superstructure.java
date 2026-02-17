package com.team9470.subsystems;

import com.team9470.subsystems.hopper.Hopper;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.util.AutoAim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    // Context suppliers (set by RobotContainer)
    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<ChassisSpeeds> speedsSupplier = () -> new ChassisSpeeds();

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
        this.poseSupplier = pose;
        this.speedsSupplier = speeds;
        shooter.setSimulationContext(pose, speeds);
    }

    @Override
    public void periodic() {
        AutoAim.publishModeTelemetry(poseSupplier.get());
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
     * Toggle intake arm to deploy-high/retracted.
     */
    public Command toggleIntakeHighCommand() {
        return intake.getDeployHighToggleCommand()
                .withName("Superstructure Toggle Intake High");
    }

    /**
     * Agitate intake while held.
     */
    public Command agitateIntakeCommand() {
        return intake.getAgitateCommand()
                .withName("Superstructure Agitate Intake");
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
            intake.setShooting(false);
            intake.setAgitating(true);
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
            publishTelemetry(isAligned, canFire, rotCmd, rotError);

        }, this).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            intake.setShooting(false);
            intake.setAgitating(false);
        }).withName("Superstructure Shoot");
    }

    /**
     * Aim and shoot command - calculates aim and returns rotation command for
     * swerve.
     * Use with swerve to get the rotation rate.
     */
    public AimResult getAimResult() {
        var solution = AutoAim.calculate(poseSupplier.get(), speedsSupplier.get());

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
            intake.setShooting(false);
            intake.setAgitating(true);
            boolean canFire = result.isAligned() && result.solution().isValid() && shooter.isAtSetpoint();
            double rotError = result.solution().targetRobotYaw()
                    .minus(poseSupplier.get().getRotation())
                    .getRadians();

            // Control shooter and hopper based on alignment
            shooter.setFiring(canFire);
            hopper.setRunning(canFire);

            // Telemetry
            publishTelemetry(result.isAligned(), canFire, result.rotationCommand(), rotError);

        }, this).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            intake.setShooting(false);
            intake.setAgitating(false);
        }).withName("Superstructure AimAndShoot");
    }

    /**
     * Idle command - stops all mechanisms.
     */
    public Command idleCommand() {
        return Commands.runOnce(() -> {
            shooter.stop();
            hopper.stop();
            intake.setShooting(false);
            intake.setAgitating(false);
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

    private void publishTelemetry(boolean isAligned, boolean canFire, double rotCmd, double rotErrorRad) {
        telemetry.publishSuperstructureState(new SuperstructureSnapshot(isAligned, canFire, rotCmd, rotErrorRad));
    }
}
