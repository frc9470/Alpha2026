package com.team9470.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team9470.subsystems.shooter.ShooterConstants;

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
    public static final Translation3d SHOOTER_OFFSET = new Translation3d(
            ShooterConstants.kShooterOffsetX,
            Meters.of(0.0),
            ShooterConstants.kShooterOffsetZ);

    // Solver Tunables - Distance-Based Variable Latency
    // Close range needs more lead time due to high angular velocity sensitivity
    private static final double CLOSE_RANGE_DELAY = 0.14; // seconds (for distances < 2m)
    private static final double FAR_RANGE_DELAY = 0.10;   // seconds (for distances > 5m)
    private static final double CLOSE_RANGE_DIST = 2.0;   // meters
    private static final double FAR_RANGE_DIST = 5.0;     // meters
    // Limits
    private static final double T_MIN = 0.25;
    private static final double T_MAX = 1.2;
    private static final double T_STEP = 0.05; // 50ms steps
    private static final double GAMMA_MAX_RAD = Units.degreesToRadians(-30.0); // Entry angle must be steeper (more
                                                                               // negative) than this

    // --- Result Record ---
    public record ShootingSolution(
            Rotation2d targetRobotYaw,
            Rotation2d pitch,
            double speed,
            double timeOfFlight,
            double entryAngle,
            double targetOmega,
            boolean isValid) {
    }

    /**
     * Returns the target position (Field-Relative), flipped based on alliance.
     */
    public static Translation3d getTarget() {
        return AllianceFlipUtil.apply(BASE_TARGET);
    }

    // Low-pass filter state for acceleration smoothing
    private static ChassisSpeeds filteredAcceleration = new ChassisSpeeds();
    private static final double ACCEL_FILTER_ALPHA = 0.3; // Lower = smoother, Higher = more responsive

    /**
     * Calculates the shooting solution using the Kinematic Solver.
     * Assumes zero acceleration if not provided.
     */
    public static ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        // Assume zero acceleration
        return solve(robotPose, robotSpeeds, new ChassisSpeeds());
    }

    /**
     * Calculates the shooting solution with acceleration feedforward.
     * Use this overload when tracking robot acceleration for better prediction.
     * @param robotPose Current robot pose
     * @param robotSpeeds Current robot velocity (robot-relative)
     * @param robotAcceleration Current robot acceleration (robot-relative)
     */
    public static ShootingSolution calculate(Pose2d robotPose, ChassisSpeeds robotSpeeds, ChassisSpeeds robotAcceleration) {
        // Apply low-pass filter to acceleration to reduce jitter
        filteredAcceleration = new ChassisSpeeds(
            filteredAcceleration.vxMetersPerSecond * (1 - ACCEL_FILTER_ALPHA) + robotAcceleration.vxMetersPerSecond * ACCEL_FILTER_ALPHA,
            filteredAcceleration.vyMetersPerSecond * (1 - ACCEL_FILTER_ALPHA) + robotAcceleration.vyMetersPerSecond * ACCEL_FILTER_ALPHA,
            filteredAcceleration.omegaRadiansPerSecond * (1 - ACCEL_FILTER_ALPHA) + robotAcceleration.omegaRadiansPerSecond * ACCEL_FILTER_ALPHA
        );
        return solve(robotPose, robotSpeeds, filteredAcceleration);
    }

    /**
     * Step 1-6: Full Kinematic Solver
     */
    public static ShootingSolution solve(Pose2d p_r, ChassisSpeeds v_r, ChassisSpeeds a_r) {
        if (p_r == null || v_r == null) {
            return new ShootingSolution(new Rotation2d(), Rotation2d.fromDegrees(45), 0.0, 0.0, 0.0, 0.0, false);
        }

        Translation3d target = getTarget();

        // --- Step 1: Calculate Distance-Based Variable Latency ---
        double distanceToTarget = p_r.getTranslation().getDistance(target.toTranslation2d());
        double dt = MathUtil.interpolate(
            CLOSE_RANGE_DELAY, 
            FAR_RANGE_DELAY, 
            MathUtil.clamp((distanceToTarget - CLOSE_RANGE_DIST) / (FAR_RANGE_DIST - CLOSE_RANGE_DIST), 0.0, 1.0)
        );
        
        // Log the effective delay for tuning
        SmartDashboard.putNumber("Shooter/Solver/EffectiveDelay", dt);
        SmartDashboard.putNumber("Shooter/Solver/DistanceToTarget", distanceToTarget);

        // --- Step 2: Predict Robot State at Release using Arc-Based Motion ---
        Rotation2d psi = p_r.getRotation();
        double omega = v_r.omegaRadiansPerSecond;
        
        // Predict Heading: psi' = psi + omega*dt
        Rotation2d psi_prime = psi.plus(new Rotation2d(omega * dt));

        // Arc-Based Motion Prediction (Twist Exponential / Discretization)
        // When rotating, the robot moves in an arc, not a straight line.
        // Use sinc functions to properly integrate the curved path.
        double vx_robot = v_r.vxMetersPerSecond;
        double vy_robot = v_r.vyMetersPerSecond;
        double ax_robot = a_r.vxMetersPerSecond;
        double ay_robot = a_r.vyMetersPerSecond;
        
        // Discretized velocity components (accounts for rotation during motion)
        double vx_discrete, vy_discrete;
        if (Math.abs(omega) < 1e-9) {
            // No rotation - use linear approximation
            vx_discrete = vx_robot;
            vy_discrete = vy_robot;
        } else {
            // Arc motion - use twist exponential formulas
            // These formulas compute the average velocity direction over the arc
            double halfAngle = omega * dt / 2.0;
            double sinc = Math.sin(halfAngle) / halfAngle; // sinc(theta/2)
            double cosc = (1.0 - Math.cos(halfAngle)) / halfAngle; // (1-cos(theta/2))/(theta/2)
            
            // Discretized velocities that account for the arc
            vx_discrete = vx_robot * sinc - vy_robot * cosc;
            vy_discrete = vx_robot * cosc + vy_robot * sinc;
        }
        
        // Convert discretized robot-relative velocity to field-relative
        Translation2d v_discrete_field = new Translation2d(vx_discrete, vy_discrete).rotateBy(psi);
        Translation2d a_robot_vec = new Translation2d(ax_robot, ay_robot).rotateBy(psi);
        
        // Predict velocity at release time (for shooter velocity compensation)
        Translation2d v_robot_vec = new Translation2d(vx_robot, vy_robot).rotateBy(psi);
        Translation2d v_prime = v_robot_vec.plus(a_robot_vec.times(dt));

        // Predict Position using arc-corrected velocity: p' = p + v_discrete*dt + 0.5*a*dt^2
        Translation2d p_prime_xy = p_r.getTranslation()
                .plus(v_discrete_field.times(dt))
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
        // Predict Shooter Velocity Contribution from Rotation
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
            double vx_world = (target.getX() - p0.getX()) / tf;
            double vy_world = (target.getY() - p0.getY()) / tf;
            double vz_world = (target.getZ() - p0.getZ() + (0.5 * GRAVITY * tf * tf)) / tf;

            Translation3d v_world = new Translation3d(vx_world, vy_world, vz_world);

            // B. Check Impact Angle gamma
            double vz_impact = vz_world - (GRAVITY * tf);
            double vxy_impact = Math.hypot(vx_world, vy_world);
            double gamma = Math.atan2(vz_impact, vxy_impact);

            // Reject if too shallow
            if (gamma > GAMMA_MAX_RAD)
                continue;

            // --- Step 3: Convert to Shooter Relative ---
            Translation3d v_rel = v_world.minus(v0_robot);

            // Compute Commands
            double speed = v_rel.getNorm();
            double pitchRad = Math.atan2(v_rel.getZ(), Math.hypot(v_rel.getX(), v_rel.getY()));

            Rotation2d pitch = new Rotation2d(pitchRad);
            Rotation2d yawField = new Rotation2d(v_rel.getX(), v_rel.getY());

            // Convert Yaw to Robot Relative
            // Robot Should Face the Shot Direction (YawField)
            Rotation2d targetRobotYaw = yawField;

            // --- Step 4: Cost Function ---
            // Minimize Speed (Energy)
            double cost = speed;

            // Optimization: Track best
            if (cost < bestCost) {
                // Calculate Heading Feedforward (Angular Velocity required to track target)
                // Omega_ff = (r x v_rel) / r^2
                // r = Vector to Target (dx, dy)
                // v_rel = Velocity of Target relative to Robot = -v_robot (Field Relative)
                // Numerator = dx * (-vy) - dy * (-vx) = dy * vx - dx * vy
                double dx_ff = target.getX() - p_r.getX();
                double dy_ff = target.getY() - p_r.getY();
                double distSq = dx_ff * dx_ff + dy_ff * dy_ff;
                double vX_robot = v_robot_vec.getX();
                double vY_robot = v_robot_vec.getY();

                double targetOmega = 0.0;
                if (distSq > 1e-6) {
                    targetOmega = (dy_ff * vX_robot - dx_ff * vY_robot) / distSq;
                }

                bestCost = cost;
                bestSol = new ShootingSolution(targetRobotYaw, pitch, speed, tf, gamma, targetOmega, true);
            }
        }

        if (bestSol == null) {
            // No valid solution found within constraints
            return new ShootingSolution(new Rotation2d(), Rotation2d.fromDegrees(45), 0.0, 0.0, 0.0, 0.0, false);
        }

        // Telemetry
        LogUtil.recordPose3d("Shooter/Target/Static", new Pose3d(target, new Rotation3d()));
        // Predict Release Point
        LogUtil.recordPose3d("Shooter/PredictedRelease",
                new Pose3d(new Translation3d(p0.getX(), p0.getY(), p0.getZ()), new Rotation3d()));
        SmartDashboard.putNumber("Shooter/EntryAngleDeg", Units.radiansToDegrees(bestSol.entryAngle));
        SmartDashboard.putNumber("Shooter/Speed", bestSol.speed);

        // Enhanced Diagnostics
        SmartDashboard.putNumber("Shooter/Solver/TimeOfFlight", bestSol.timeOfFlight);
        SmartDashboard.putNumber("Shooter/Solver/PitchDeg", bestSol.pitch.getDegrees());
        SmartDashboard.putNumber("Shooter/Solver/TargetYawDeg", bestSol.targetRobotYaw.getDegrees());

        // Kinematic Compensation Diagnostics
        // Compare "Static" speed (v_world magnitude) vs "Dynamic" speed (v_rel
        // magnitude)
        // We can re-calculate v_world magnitude from the last iteration's best solution
        // components if needed,
        // or just accept we log the result.
        // Let's log the Robot Velocity input to confirm it's non-zero
        SmartDashboard.putNumber("Shooter/Solver/InputRobotVelX", v_r.vxMetersPerSecond);
        SmartDashboard.putNumber("Shooter/Solver/InputRobotVelY", v_r.vyMetersPerSecond);
        SmartDashboard.putNumber("Shooter/Solver/InputRobotOmega", v_r.omegaRadiansPerSecond);

        // Heading Diagnostics
        // Calculate what the yaw would be if we were stationary (Static Look-At)
        double dx = target.getX() - p_r.getX();
        double dy = target.getY() - p_r.getY();
        Rotation2d staticYaw = new Rotation2d(dx, dy);

        SmartDashboard.putNumber("Shooter/Solver/StaticYawDeg", staticYaw.getDegrees());
        SmartDashboard.putNumber("Shooter/Solver/CompensatedYawDeg", bestSol.targetRobotYaw.getDegrees());
        SmartDashboard.putNumber("Shooter/Solver/YawCompensationErr",
                bestSol.targetRobotYaw.minus(staticYaw).getDegrees());

        return bestSol;
    }
}
