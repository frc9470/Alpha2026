package com.team9470.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class AutoAim {

    // Constants
    private static final double GRAVITY = 9.81; // m/s^2

    // Geometry Constants (Metric)
    private static final double SHOOTER_HEIGHT_METERS = Units.inchesToMeters(24.0);
    private static final double RIM_HEIGHT_METERS = Units.inchesToMeters(72.0);
    private static final double BALL_RADIUS_METERS = Units.inchesToMeters(2.75);

    // Derived Targets
    // Delta Y Clear = (Rim + Ball Radius) - Shooter Height
    private static final double DELTA_Y_CLEARANCE = (RIM_HEIGHT_METERS + BALL_RADIUS_METERS) - SHOOTER_HEIGHT_METERS;

    // Target Entry Angle (Descending -35 deg)
    private static final double GAMMA_TARGET_RAD = Units.degreesToRadians(-35.0);

    // Field Center Target (Reef Center)
    private static final Translation3d BASE_TARGET = new Translation3d(Units.inchesToMeters(182.111250),
            Units.inchesToMeters(158.843750), RIM_HEIGHT_METERS);

    // Turret Offset relative to robot center (20cm forward, 0.5m up) -> Wait, user
    // said Shooter Exit Height is 24in (~0.6m).
    // Previous code had 0.5m. Let's update Turret Z to SHOOTER_HEIGHT_METERS to be
    // consistent with physics model.
    public static final Translation3d TURRET_OFFSET = new Translation3d(0.2, 0.0, SHOOTER_HEIGHT_METERS);

    public record ShootingSolution(Rotation2d yaw, Rotation2d pitch, double speed, double timeOfFlight) {
    }

    /**
     * Returns the target position, flipped based on the current alliance color.
     */
    public static Translation3d getTarget() {
        return AllianceFlipUtil.apply(BASE_TARGET);
    }

    /**
     * Calculates the shooting solution to hit a target from the current robot pose.
     * 
     * @param robotPose The current pose of the robot on the field.
     * @param targetPos The target position in 3D space (Field-Relative).
     * @return A ShootingSolution containing the necessary turret yaw, pitch,
     *         shot speed, and time of flight.
     */
    public static ShootingSolution calculate(Pose2d robotPose, Translation3d targetPos) {
        if (robotPose == null)
            return new ShootingSolution(new Rotation2d(), Rotation2d.fromDegrees(45), 0.0, 0.0);

        // 1. Calculate Turret Position in 3D Space (Field Relative)
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();

        // Rotate the XY part of the offset by the robot's rotation
        Translation2d offsetXY = new Translation2d(TURRET_OFFSET.getX(), TURRET_OFFSET.getY()).rotateBy(robotRotation);

        Translation3d turretPos = new Translation3d(
                robotTranslation.getX() + offsetXY.getX(),
                robotTranslation.getY() + offsetXY.getY(),
                TURRET_OFFSET.getZ());

        // 2. Calculate Vector to Target (Horizontal Distance d)
        // Physics model assumes target height is implicitly handled by
        // DELTA_Y_CLEARANCE relative to Shooter Height.
        // But we should verify targetPos.Z matches or we use the horizontal distance to
        // the target XY column.

        double d = turretPos.toTranslation2d().getDistance(targetPos.toTranslation2d());

        // 3. Calculate Yaw (Field Relative Angle to Target)
        Translation3d toTarget = targetPos.minus(turretPos);
        Rotation2d angleFieldRelative = new Rotation2d(toTarget.getX(), toTarget.getY());
        Rotation2d turretYaw = angleFieldRelative.minus(robotRotation);

        // 4. Calculate Pitch (Theta) and Speed (v)
        // Formula: tan(theta) = (2 * DeltaY / d) - tan(gamma)
        double tanTheta = (2 * DELTA_Y_CLEARANCE / d) - Math.tan(GAMMA_TARGET_RAD);
        double thetaRad = Math.atan(tanTheta);
        Rotation2d pitch = new Rotation2d(thetaRad);

        // Formula: v = sqrt( (g * d^2) / (2 * cos^2(theta) * (DeltaY - d * tan(gamma)))
        // )
        double cosTheta = Math.cos(thetaRad);
        double term = DELTA_Y_CLEARANCE - (d * Math.tan(GAMMA_TARGET_RAD)); // Note: tan(-35) is negative, so this adds
                                                                            // to DeltaY

        // Safety check for complex roots (shouldn't happen with these constraints but
        // good practice)
        if (term <= 0 || cosTheta == 0) {
            return new ShootingSolution(turretYaw, Rotation2d.fromDegrees(45), 0.0, 0.0); // Unreachable
        }

        double vSquared = (GRAVITY * d * d) / (2 * cosTheta * cosTheta * term);
        double speed = Math.sqrt(vSquared);

        // 5. Calculate Time of Flight (t)
        // d = v_xy * t <=> t = d / v_xy = d / (v * cos(theta))
        double timeOfFlight = d / (speed * cosTheta);

        return new ShootingSolution(turretYaw, pitch, speed, timeOfFlight);
    }

    /**
     * Default calculation aiming at Field Center.
     */
    public static ShootingSolution calculate(Pose2d robotPose) {
        return calculate(robotPose, getTarget());
    }

    /**
     * Calculates shooting solution with Lead Compensation for a moving robot.
     * 
     * @param robotPose   Current Robot Pose
     * @param robotSpeeds Current Robot Velocity (ChassisSpeeds, usually Robot
     *                    Relative)
     * @return ShootingSolution aimed at a Lead Point
     */
    public static ShootingSolution calculate(Pose2d robotPose,
            edu.wpi.first.math.kinematics.ChassisSpeeds robotSpeeds) {
        if (robotPose == null || robotSpeeds == null)
            return calculate(robotPose);

        Translation3d target = getTarget();

        // 1. Initial Solution (get TOF)
        ShootingSolution initialSol = calculate(robotPose, target);
        double t = initialSol.timeOfFlight();

        if (t <= 0)
            return initialSol;

        // 2. Robot Velocity in Field Frame
        // Rotate robot-relative speeds by robot heading
        Translation2d robotVelField = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                .rotateBy(robotPose.getRotation());

        // 3. Predict Robot Displacement: Delta P = V_robot * t
        Translation2d deltaP = robotVelField.times(t);

        // 4. Calculate Lead Point: P_lead = P_target - Delta P
        // (We subtract because if we move TO target, we effectively need to shoot
        // "shorter" relative to us?
        // Reuse math:
        // Ball_Pos_Global = Ball_Pos_Rel + Robot_Pos_t
        // Ball_Pos_Global = (V_launch_field * t) + Robot_Pos_0
        // Ball_Pos_Global = (V_launch_rel + V_robot) * t + Robot_Pos_0
        // Target = (V_launch_rel * t) + (V_robot * t) + Robot_Pos_0
        // (V_launch_rel * t) = Target - Robot_Pos_0 - (V_robot * t)
        // The term (Target - V_robot * t) acts as the effective target for the
        // relative/launcher physics.
        // Yes, P_lead = Target - (V_robot * t).

        Translation3d leadPoint = new Translation3d(
                target.getX() - deltaP.getX(),
                target.getY() - deltaP.getY(),
                target.getZ());

        // 5. Recompute Solution for Lead Point
        ShootingSolution leadSol = calculate(robotPose, leadPoint);

        // Optional: One more iteration? The user says "One iteration is usually
        // sufficient".

        return leadSol;
    }
}
