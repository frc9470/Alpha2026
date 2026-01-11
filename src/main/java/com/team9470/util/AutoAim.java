package com.team9470.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAim {

    // --- Constants ---
    private static final double GRAVITY = 9.81; // m/s^2

    // Field Geometry
    private static final double RIM_HEIGHT_METERS = Units.inchesToMeters(72.0);
    private static final double BALL_RADIUS_METERS = Units.inchesToMeters(2.75);
    private static final double GOAL_Z = RIM_HEIGHT_METERS + BALL_RADIUS_METERS; // Target Z

    // Field Center Target (Reef Center)
    private static final Translation3d BASE_TARGET = new Translation3d(
            Units.inchesToMeters(182.111250),
            Units.inchesToMeters(158.843750),
            GOAL_Z);

    // Shooter Configuration
    // Exit point relative to robot center
    public static final Translation3d SHOOTER_OFFSET = new Translation3d(0.2, 0.0, Units.inchesToMeters(24.0));

    // Solver Tunables
    private static final double RELEASE_DELAY = 0.0; // seconds (Command Latency + Mechanism Delay)
    // Limits
    private static final double T_MIN = 0.25;
    private static final double T_MAX = 1.2;
    private static final double T_STEP = 0.05; // 50ms steps
    private static final double GAMMA_MAX_RAD = Units.degreesToRadians(-30.0); // Entry angle must be steeper (more
                                                                               // negative) than this

    // --- Result Record ---
    public record ShootingSolution(
            Rotation2d yaw,
            Rotation2d pitch,
            double speed,
            double timeOfFlight,
            double entryAngle,
            boolean isValid) {
    }

    /**
     * Returns the target position (Field-Relative), flipped based on alliance.
     */
    public static Translation3d getTarget() {
        return AllianceFlipUtil.apply(BASE_TARGET);
    }

    /**
     * Calculates the shooting solution using the Kinematic Solver.
     * Assumes zero acceleration if not provided.
     */
    public static ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        // Assume zero acceleration
        return solve(robotPose, robotSpeeds, new ChassisSpeeds());
    }

    /**
     * Step 1-6: Full Kinematic Solver
     */
    public static ShootingSolution solve(Pose2d p_r, ChassisSpeeds v_r, ChassisSpeeds a_r) {
        if (p_r == null || v_r == null) {
            return new ShootingSolution(new Rotation2d(), Rotation2d.fromDegrees(45), 0.0, 0.0, 0.0, false);
        }

        Translation3d target = getTarget();

        // --- Step 1: Predict Robot State at Release ---
        double dt = RELEASE_DELAY;

        // Predict Heading: psi' = psi + omega*dt
        Rotation2d psi = p_r.getRotation();
        double omega = v_r.omegaRadiansPerSecond;
        Rotation2d psi_prime = psi.plus(new Rotation2d(omega * dt)); // Ignore alpha for now

        // Predict Velocity: v' = v + a*dt (Field Relative)
        // Convert Robot-Relative inputs to Field-Relative vectors
        Translation2d v_robot_vec = new Translation2d(v_r.vxMetersPerSecond, v_r.vyMetersPerSecond).rotateBy(psi);
        Translation2d a_robot_vec = new Translation2d(a_r.vxMetersPerSecond, a_r.vyMetersPerSecond).rotateBy(psi);
        Translation2d v_prime = v_robot_vec.plus(a_robot_vec.times(dt));

        // Predict Position: p' = p + v*dt + 0.5*a*dt^2
        Translation2d p_prime_xy = p_r.getTranslation()
                .plus(v_robot_vec.times(dt))
                .plus(a_robot_vec.times(0.5 * dt * dt));

        // Predict Shooter Exit Point in Field Frame
        // r_s' = R(psi') * r_s
        Translation2d shooterOffsetXY = new Translation2d(SHOOTER_OFFSET.getX(), SHOOTER_OFFSET.getY())
                .rotateBy(psi_prime);
        Translation3d p0 = new Translation3d(
                p_prime_xy.getX() + shooterOffsetXY.getX(),
                p_prime_xy.getY() + shooterOffsetXY.getY(),
                SHOOTER_OFFSET.getZ() // Assume flat floor for now (z=0 + offset)
        );

        // Predict Shooter Velocity Contribution from Rotation
        // v_rot = omega' x r_s'
        // r_s' (3D) = (x, y, z). Omega is around Z.
        // v_rot = (-omega * y, omega * x, 0)
        // Note: shooterOffsetXY is (x, y) relative to robot center, rotated to field
        // frame.
        // Wait, "r_s' = R_z(psi') * r_s".
        // Cross product is (0,0,w) x (rx, ry, rz) = (-w*ry, w*rx, 0).
        // Correct.
        Translation3d v0_rot = new Translation3d(
                -omega * shooterOffsetXY.getY(),
                omega * shooterOffsetXY.getX(),
                0.0);
        Translation3d v0_robot = new Translation3d(v_prime.getX(), v_prime.getY(), 0.0).plus(v0_rot);

        // --- Step 2, 3, 4: Search over TOF ---
        ShootingSolution bestSol = null;
        double bestCost = Double.MAX_VALUE;

        // Iterate Time of Flight
        for (double tf = T_MIN; tf <= T_MAX; tf += T_STEP) {

            // A. Solve for Required World Velocity
            // P_t = P_0 + V_w * t + 0.5 * G * t^2. (G = [0,0,-9.81])
            // V_w = (P_t - P_0 - 0.5 * G * t^2) / t

            // Gravity Term: 0.5 * G * t^2 = (0, 0, -0.5 * 9.81 * t^2)
            // Subtracting it means ADDING positive Z.
            // Numerator = Target - P0 - (GravityVector)
            // NumeratorZ = TargetZ - P0Z - (-0.5*g*t*t) = TargetZ - P0Z + 0.5*g*t*t

            double vx_world = (target.getX() - p0.getX()) / tf;
            double vy_world = (target.getY() - p0.getY()) / tf;
            double vz_world = (target.getZ() - p0.getZ() + (0.5 * GRAVITY * tf * tf)) / tf;

            Translation3d v_world = new Translation3d(vx_world, vy_world, vz_world);

            // B. Check Impact Angle gamma
            // v_impact = v_world + G*tf
            double vz_impact = vz_world - (GRAVITY * tf);
            double vxy_impact = Math.hypot(vx_world, vy_world);
            double gamma = Math.atan2(vz_impact, vxy_impact); // Angle vs Horizon

            // Reject if too shallow (gamma > -30 deg implies -10, 0, +10. We want < -30
            // e.g. -45)
            // -45 < -30 is TRUE.
            if (gamma > GAMMA_MAX_RAD)
                continue;

            // --- Step 3: Convert to Shooter Relative ---
            // v_rel = v_world - v0_robot
            Translation3d v_rel = v_world.minus(v0_robot);

            // Compute Commands
            double speed = v_rel.getNorm();
            double pitchRad = Math.atan2(v_rel.getZ(), Math.hypot(v_rel.getX(), v_rel.getY()));
            Rotation2d pitch = new Rotation2d(pitchRad);
            Rotation2d yawField = new Rotation2d(v_rel.getX(), v_rel.getY()); // Absolute direction of ball launch in XY

            // Convert Yaw to Robot Relative
            // Turret Yaw = YawField - RobotHeading(psi_prime)
            Rotation2d turretYaw = yawField.minus(psi_prime);

            // --- Step 4: Cost Function ---
            // Minimize Speed (Energy)
            double cost = speed;

            // Optimization: Track best
            if (cost < bestCost) {
                bestCost = cost;
                bestSol = new ShootingSolution(turretYaw, pitch, speed, tf, gamma, true);
            }
        }

        if (bestSol == null) {
            // No valid solution found within constraints
            return new ShootingSolution(new Rotation2d(), Rotation2d.fromDegrees(45), 0.0, 0.0, 0.0, false);
        }

        // Telemetry
        LogUtil.recordPose3d("Shooter/Target/Static", new Pose3d(target, new Rotation3d()));
        // Predict Release Point
        LogUtil.recordPose3d("Shooter/PredictedRelease",
                new Pose3d(new Translation3d(p0.getX(), p0.getY(), p0.getZ()), new Rotation3d()));
        SmartDashboard.putNumber("Shooter/EntryAngleDeg", Units.radiansToDegrees(bestSol.entryAngle));
        SmartDashboard.putNumber("Shooter/Speed", bestSol.speed);

        return bestSol;
    }
}
