package com.team9470.util;

import com.team9470.FieldConstants;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.AutoAimSolverSnapshot;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.team9470.subsystems.shooter.ShooterInterpolationMaps;
import com.team9470.subsystems.shooter.ShotParameter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.*;

import java.util.Optional;

public class AutoAim {
    private static final TelemetryManager telemetry = TelemetryManager.getInstance();

    // Field Geometry (from FieldConstants)
    private static final double BALL_RADIUS_METERS = FieldConstants.GamePiece.ballRadius;
    private static final double GOAL_Z = FieldConstants.Hub.height + BALL_RADIUS_METERS; // Target Z
    private static final double FEED_MODE_BLUE_X_THRESHOLD_M = 4.11; // was 3.5, increased by 2 ft

    // Field Center Target (Hub center point + ball clearance)
    private static final Translation3d BASE_HUB_TARGET = new Translation3d(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.topCenterPoint.getY(),
            GOAL_Z);
    private static final Translation3d BASE_FEED_TARGET = new Translation3d(1.8, 6.7, GOAL_Z);

    // Shooter Configuration
    // Exit point relative to robot center
    public static final Translation3d SHOOTER_OFFSET = new Translation3d(
            ShooterConstants.kShooterOffsetX,
            Meters.of(0.0),
            ShooterConstants.kShooterOffsetZ);

    private enum AimMode {
        HUB,
        FEED
    }

    // --- Result Record ---
    public record ShootingSolution(
            Rotation2d targetRobotYaw,
            double hoodCommandDeg,
            double flywheelRpm,
            double targetOmega,
            boolean isValid) {
    }

    /**
     * Returns the target position (Field-Relative), flipped based on alliance.
     */
    public static Translation3d getTarget() {
        return AllianceFlipUtil.apply(BASE_HUB_TARGET);
    }

    /**
     * Returns the target position (Field-Relative), flipped based on alliance and
     * selected auto mode.
     */
    public static Translation3d getTarget(Pose2d robotPose) {
        if (robotPose == null) {
            return getTarget();
        }
        AimMode mode = getAimMode(robotPose);
        Translation3d baseTarget = (mode == AimMode.FEED) ? BASE_FEED_TARGET : BASE_HUB_TARGET;
        return AllianceFlipUtil.apply(baseTarget);
    }

    /**
     * Returns true when feed mode is active for the provided pose.
     */
    public static boolean isFeedModeActive(Pose2d robotPose) {
        if (robotPose == null) {
            return false;
        }
        return getAimMode(robotPose) == AimMode.FEED;
    }

    /**
     * Returns the robot X position converted to the blue reference frame.
     */
    public static double getRobotXBlueMeters(Pose2d robotPose) {
        if (robotPose == null) {
            return 0.0;
        }
        return AllianceFlipUtil.applyX(robotPose.getX());
    }

    /**
     * Publishes mode telemetry independent of active shooting state.
     */
    public static void publishModeTelemetry(Pose2d robotPose) {
        double robotXBlueMeters = getRobotXBlueMeters(robotPose);
        AimMode mode = isFeedModeActive(robotPose) ? AimMode.FEED : AimMode.HUB;
        telemetry.publishAutoAimSolver(new AutoAimSolverSnapshot(
                mode == AimMode.FEED ? 1 : 0,
                mode == AimMode.FEED,
                robotXBlueMeters,
                Double.NaN,
                false,
                Double.NaN,
                Double.NaN));
    }

    /**
     * Calculates the shooting solution from static distance interpolation maps.
     */
    public static ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        if (robotPose == null || robotSpeeds == null) {
            publishMapTelemetry(0.0, null, AimMode.HUB, false, 0.0);
            return new ShootingSolution(new Rotation2d(), 0.0, 0.0, 0.0, false);
        }

        double robotXBlueMeters = getRobotXBlueMeters(robotPose);
        AimMode mode = getAimMode(robotPose);
        boolean feedModeActive = mode == AimMode.FEED;
        Translation3d target = getTarget(robotPose);
        Translation2d targetXY = target.toTranslation2d();

        Translation2d shooterOffsetXY = new Translation2d(SHOOTER_OFFSET.getX(), SHOOTER_OFFSET.getY())
                .rotateBy(robotPose.getRotation());
        Translation2d shooterExitXY = robotPose.getTranslation().plus(shooterOffsetXY);

        double distanceMeters = shooterExitXY.getDistance(targetXY);
        Optional<ShotParameter> shotParameter = mode == AimMode.FEED
                ? ShooterInterpolationMaps.getFeed(distanceMeters)
                : ShooterInterpolationMaps.getHub(distanceMeters);
        publishMapTelemetry(distanceMeters, shotParameter.orElse(null), mode, feedModeActive, robotXBlueMeters);

        double dx = targetXY.getX() - robotPose.getX();
        double dy = targetXY.getY() - robotPose.getY();
        Rotation2d targetRobotYaw = new Rotation2d(dx, dy);

        if (shotParameter.isEmpty() || !shotParameter.get().isValid()) {
            return new ShootingSolution(targetRobotYaw, 0.0, 0.0, 0.0, false);
        }

        ShotParameter shot = shotParameter.get();
        return new ShootingSolution(
                targetRobotYaw,
                shot.hoodCommandDeg(),
                shot.flywheelRpm(),
                0.0,
                true);
    }

    private static AimMode getAimMode(Pose2d robotPose) {
        double robotXBlueMeters = AllianceFlipUtil.applyX(robotPose.getX());
        return robotXBlueMeters > FEED_MODE_BLUE_X_THRESHOLD_M ? AimMode.FEED : AimMode.HUB;
    }

    private static void publishMapTelemetry(
            double distanceMeters,
            ShotParameter shot,
            AimMode mode,
            boolean feedModeActive,
            double robotXBlueMeters) {
        boolean valid = shot != null && shot.isValid();
        telemetry.publishAutoAimSolver(new AutoAimSolverSnapshot(
                mode == AimMode.FEED ? 1 : 0,
                feedModeActive,
                robotXBlueMeters,
                distanceMeters,
                valid,
                valid ? Math.toRadians(shot.hoodCommandDeg()) : 0.0,
                valid ? shot.flywheelRpm() / 60.0 : 0.0));
    }
}
