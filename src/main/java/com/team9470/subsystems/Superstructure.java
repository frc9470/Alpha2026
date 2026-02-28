package com.team9470.subsystems;

import com.team9470.subsystems.hopper.Hopper;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.SuperstructureSnapshot;
import com.team9470.util.AutoAim;

import com.team9470.subsystems.swerve.Swerve;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.atomic.AtomicBoolean;
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
     * Feed command - spins shooter at 500 RPM with max hood angle and feeds game
     * pieces.
     * Intended for passing/feeding to a partner robot.
     */
    public Command feedCommand() {
        final double feedRps = 500.0 / 60.0; // 500 RPM -> RPS
        final double maxHoodRotations = com.team9470.subsystems.shooter.ShooterConstants
                .launchRadToMechanismRotations(
                        com.team9470.subsystems.shooter.ShooterConstants.kMaxHoodAngle.in(
                                edu.wpi.first.units.Units.Radians));
        return Commands.runEnd(
                () -> {
                    shooter.setFlywheelSpeed(feedRps);
                    shooter.setHoodAngle(maxHoodRotations);
                    shooter.setFiring(true);
                    hopper.setRunning(true);
                    intake.setShooting(true);
                    intake.setAgitating(true);
                },
                () -> {
                    shooter.stop();
                    hopper.stop();
                    intake.setShooting(false);
                    intake.setAgitating(false);
                }).withName("Superstructure Feed");
    }

    /**
     * Re-home both intake pivot and shooter hood.
     */
    public Command homeIntakeAndHoodCommand() {
        return Commands.runOnce(() -> {
            intake.requestHome();
            shooter.requestHome();
        }, intake, shooter).withName("Superstructure Home Intake+Hood");
    }

    /**
     * Aim and shoot command - calculates aim and returns rotation command for
     * swerve.
     * Use with swerve to get the rotation rate.
     */
    public AimResult getAimResult() {
        return getAimResult(false);
    }

    public AimResult getAimResult(boolean useRobotSideForFeedTarget) {
        var solution = AutoAim.calculate(poseSupplier.get(), speedsSupplier.get(), useRobotSideForFeedTarget);

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
     * Aim, rotate, and shoot — auto version (robot stationary, only rotates to
     * aim).
     */
    public Command aimAndShootCommand() {
        return aimAndShootCommand(() -> 0.0, () -> 0.0, true);
    }

    /**
     * Aim, rotate, and shoot — teleop version (driver controls translation).
     * Handles swerve rotation automatically via auto-aim.
     *
     * @param vxSupplier field-relative X velocity (m/s)
     * @param vySupplier field-relative Y velocity (m/s)
     */
    public Command aimAndShootCommand(Supplier<Double> vxSupplier, Supplier<Double> vySupplier) {
        return aimAndShootCommand(vxSupplier, vySupplier, true);
    }

    /**
     * Aim, rotate, and shoot — full version.
     * Handles swerve rotation automatically via auto-aim.
     *
     * @param vxSupplier field-relative X velocity (m/s)
     * @param vySupplier field-relative Y velocity (m/s)
     * @param agitate    whether to agitate the intake during shooting
     */
    public Command aimAndShootCommand(Supplier<Double> vxSupplier, Supplier<Double> vySupplier, boolean agitate) {
        Swerve swerve = Swerve.getInstance();
        AtomicBoolean shooterReadyLatched = new AtomicBoolean(false);
        boolean useRobotSideForFeedTarget = !agitate;
        // Use closed-loop velocity so commanding 0 m/s actively brakes (no drift)
        SwerveRequest.FieldCentric aimDrive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
        return Commands.run(() -> {
            var result = getAimResult(useRobotSideForFeedTarget);
            shooter.setSetpoint(result.solution());
            intake.setShooting(true);
            intake.setAgitating(agitate);

            boolean shooterAtSetpoint = shooter.isAtSetpoint();
            if (shooterAtSetpoint) {
                shooterReadyLatched.set(true);
            }

            boolean canFire = result.isAligned() && result.solution().isValid() && shooterAtSetpoint;
            double rotError = result.solution().targetRobotYaw()
                    .minus(poseSupplier.get().getRotation())
                    .getRadians();

            // Wait for initial shooter readiness, then feed continuously.
            shooter.setFiring(canFire);
            hopper.setRunning(shooterReadyLatched.get());

            // Drive: pass through translation, auto-aim rotation (closed-loop velocity)
            swerve.setControl(aimDrive
                    .withVelocityX(vxSupplier.get())
                    .withVelocityY(vySupplier.get())
                    .withRotationalRate(result.rotationCommand()));

            // Telemetry
            publishTelemetry(result.isAligned(), canFire, result.rotationCommand(), rotError);

        }, this, swerve).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            intake.setShooting(false);
            intake.setAgitating(false);
            shooterReadyLatched.set(false);
            swerve.setControl(aimDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }).beforeStarting(() -> shooterReadyLatched.set(false))
                .withName("Superstructure AimAndShoot");
    }

    /**
     * Shoot without auto-aligning — uses AutoAim to set flywheel/hood but does
     * NOT control swerve rotation. The driver retains full manual drivetrain
     * control.
     */
    public Command shootNoAlignCommand() {
        AtomicBoolean shooterReadyLatched = new AtomicBoolean(false);
        return Commands.run(() -> {
            var result = getAimResult();
            shooter.setSetpoint(result.solution());
            intake.setShooting(true);
            intake.setAgitating(true);

            boolean shooterAtSetpoint = shooter.isAtSetpoint();
            if (shooterAtSetpoint) {
                shooterReadyLatched.set(true);
            }

            boolean canFire = result.solution().isValid() && shooterAtSetpoint;

            shooter.setFiring(canFire);
            hopper.setRunning(shooterReadyLatched.get());

            publishTelemetry(false, canFire, 0.0, 0.0);
        }, this).finallyDo(() -> {
            shooter.stop();
            hopper.stop();
            intake.setShooting(false);
            intake.setAgitating(false);
            shooterReadyLatched.set(false);
        }).beforeStarting(() -> shooterReadyLatched.set(false))
                .withName("Superstructure ShootNoAlign");
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
