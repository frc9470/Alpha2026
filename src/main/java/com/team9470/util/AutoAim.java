package com.team9470.util;

import com.team9470.FieldConstants;
import com.team9470.subsystems.shooter.ShooterConstants;
import com.team9470.subsystems.shooter.ShooterInterpolationMaps;
import com.team9470.subsystems.shooter.ShotParameter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoAim {

    // Field Geometry (from FieldConstants)
    private static final double BALL_RADIUS_METERS = FieldConstants.GamePiece.ballRadius;
    private static final double GOAL_Z = FieldConstants.Hub.height + BALL_RADIUS_METERS; // Target Z

    // Field Center Target (Hub center point + ball clearance)
    private static final Translation3d BASE_TARGET = new Translation3d(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.topCenterPoint.getY(),
            GOAL_Z);

    // Shooter Configuration
    // Exit point relative to robot center
    public static final Translation3d SHOOTER_OFFSET = new Translation3d(
            ShooterConstants.kShooterOffsetX,
            Meters.of(0.0),
            ShooterConstants.kShooterOffsetZ);

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
        return AllianceFlipUtil.apply(BASE_TARGET);
    }

    /**
     * Calculates the shooting solution from static distance interpolation maps.
     */
    public static ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        if (robotPose == null || robotSpeeds == null) {
            publishMapTelemetry(0.0, null);
            return new ShootingSolution(new Rotation2d(), 0.0, 0.0, 0.0, false);
        }

        Translation3d target = getTarget();
        Translation2d targetXY = target.toTranslation2d();

        Translation2d shooterOffsetXY = new Translation2d(SHOOTER_OFFSET.getX(), SHOOTER_OFFSET.getY())
                .rotateBy(robotPose.getRotation());
        Translation2d shooterExitXY = robotPose.getTranslation().plus(shooterOffsetXY);

        double distanceMeters = shooterExitXY.getDistance(targetXY);
        Optional<ShotParameter> shotParameter = ShooterInterpolationMaps.getSpeaker(distanceMeters);
        publishMapTelemetry(distanceMeters, shotParameter.orElse(null));

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

    private static void publishMapTelemetry(double distanceMeters, ShotParameter shot) {
        SmartDashboard.putNumber("Shooter/Solver/Map/DistanceMeters", distanceMeters);
        SmartDashboard.putBoolean("Shooter/Solver/Map/Valid", shot != null && shot.isValid());
        SmartDashboard.putNumber("Shooter/Solver/Map/HoodCommandDeg", shot != null ? shot.hoodCommandDeg() : 0.0);
        SmartDashboard.putNumber("Shooter/Solver/Map/FlywheelRPM", shot != null ? shot.flywheelRpm() : 0.0);
    }
}
