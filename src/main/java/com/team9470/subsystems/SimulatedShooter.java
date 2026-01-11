package com.team9470.subsystems;

import com.team9470.simulation.PhysicsSim;
import com.team9470.util.AutoAim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.Supplier;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SimulatedShooter extends SubsystemBase {
    // Virtual Turret State
    private Rotation2d turretYaw = new Rotation2d();
    private Rotation2d turretPitch = Rotation2d.fromDegrees(45); // Default 45 deg launch
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotSpeedsSupplier;

    // Physics Constants
    private static final double FLOOR_Z = 0.0;

    // Projectile Management
    private final List<PhysicsSim.Projectile> projectiles = new ArrayList<>();

    // Visualization
    private final StructArrayPublisher<Pose3d> ballPublisher;
    private final StructPublisher<Pose3d> turretPublisher;
    private final StructPublisher<Pose3d> targetPublisher;

    // Auto-Aim State
    private double lastCalculatedSpeed = 10.0;

    public SimulatedShooter(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedsSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.robotSpeedsSupplier = robotSpeedsSupplier;
        var table = NetworkTableInstance.getDefault().getTable("Sim");
        ballPublisher = table.getStructArrayTopic("BallPose", Pose3d.struct).publish();
        turretPublisher = table.getStructTopic("TurretPose", Pose3d.struct).publish();
        targetPublisher = table.getStructTopic("TargetPose", Pose3d.struct).publish();
    }

    /* --- Controls --- */

    public void setTurretYaw(Rotation2d yaw) {
        this.turretYaw = yaw;
    }

    public void setPitch(Rotation2d pitch) {
        this.turretPitch = pitch;
    }

    public void launch() {
        launch(lastCalculatedSpeed);
    }

    /**
     * Launches a projectile from the current turret position and angle.
     * 
     * @param speedMetersPerSecond Initial speed of the ball.
     */
    public void launch(double speedMetersPerSecond) {
        // Calculate initial velocity vector based on Yaw and Pitch
        // Yaw rotates around Z, Pitch rotates up from XY plane
        double pitchRad = turretPitch.getRadians();
        double yawRad = turretYaw.getRadians();

        // v_z = speed * sin(pitch)
        // v_xy = speed * cos(pitch)
        // v_x = v_xy * cos(yaw)
        // v_y = v_xy * sin(yaw)

        double vxy = speedMetersPerSecond * Math.cos(pitchRad);
        double vz = speedMetersPerSecond * Math.sin(pitchRad);
        double vx = vxy * Math.cos(yawRad);
        double vy = vxy * Math.sin(yawRad);

        // Add Robot Velocity (Field Relative)
        // ChassisSpeeds are usually Robot-Relative.
        // We need to rotate them to Field-Relative.
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds robotSpeeds = robotSpeedsSupplier.get();

        if (robotPose != null && robotSpeeds != null) {
            // Rotate velocity by robot heading
            Translation2d fieldVel = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                    .rotateBy(robotPose.getRotation());

            vx += fieldVel.getX();
            vy += fieldVel.getY();
            // Ignore robot Z velocity (pitch/roll) for now
        }

        // Calculate launch position (Turret Pose)
        Pose3d turretPose = getTurretPose();

        projectiles.add(new PhysicsSim.Projectile(turretPose.getTranslation(), new Translation3d(vx, vy, vz)));
    }

    @Override
    public void periodic() {
        // Auto-Aim Logic
        Pose2d robotPose = robotPoseSupplier.get();
        // Use supplied speeds if available, else empty
        ChassisSpeeds speeds = (robotSpeedsSupplier != null) ? robotSpeedsSupplier.get() : new ChassisSpeeds();

        if (robotPose != null) {
            var solution = AutoAim.calculate(robotPose, speeds);
            this.turretYaw = solution.yaw();
            this.turretPitch = solution.pitch();
            this.lastCalculatedSpeed = solution.speed();
        }

        // Update Physics
        double dt = 0.02; // Standard loop time

        Iterator<PhysicsSim.Projectile> iter = projectiles.iterator();
        while (iter.hasNext()) {
            PhysicsSim.Projectile p = iter.next();
            boolean scored = p.update(dt, AutoAim.getTarget());

            if (scored) {
                iter.remove();
                continue;
            }

            // Remove if it hits the floor
            // Note: PhysicsSim handles bounce check, but if it eventually settles or
            // purely rolls, we might want to kill it after some time.
            // For now, if Z is very low and V is low, maybe remove?
            // Or just let it roll.
            // But we wanted to verify floor collision working, so keeping it around is
            // good.
            // But to save memory/cpu, remove if "stopped" or "too far"?
            // Let's remove if Z <= 0 and vz < 0 (should be handled by physics sim
            // bounciness?)

            // Just let them exist for now. Or add max lifetime.
        }

        // Ball-Ball Collisions
        for (int i = 0; i < projectiles.size(); i++) {
            for (int j = i + 1; j < projectiles.size(); j++) {
                PhysicsSim.resolveCollision(projectiles.get(i), projectiles.get(j));
            }
        }

        // Publish Visuals
        publishVisuals();

        // Publish Target
        targetPublisher.set(new Pose3d(AutoAim.getTarget(), new Rotation3d()));
    }

    private Pose3d getTurretPose() {
        Pose2d robotPose = robotPoseSupplier.get();
        if (robotPose == null)
            return new Pose3d();

        // Robot Pose 3D
        Pose3d robotPose3d = new Pose3d(robotPose);

        // Turret offset is robot-relative.
        // We need to rotate the offset by the robot's rotation.
        // Actually Pose3d.transformBy() does this.

        return robotPose3d.transformBy(new edu.wpi.first.math.geometry.Transform3d(
                AutoAim.TURRET_OFFSET,
                new Rotation3d(0, 0, turretYaw.getRadians()) // The turret's rotation relative to robot
        ));
    }

    private void publishVisuals() {
        // Publish Turret Pose
        Pose3d turretPose = getTurretPose();

        // For visualization of the "Barrel" pitch, we might want to include pitch in
        // the rotation
        // But the getTurretPose() above returns the base of the turret.
        // Let's add the pitch to the rotation for the visualizer.
        Pose3d turretVisualPose = new Pose3d(
                turretPose.getTranslation(),
                turretPose.getRotation().plus(new Rotation3d(0, turretPitch.getRadians(), 0)));

        turretPublisher.set(turretVisualPose);

        // Publish Projectiles
        Pose3d[] ballPoses = projectiles.stream()
                .map(p -> new Pose3d(p.position, new Rotation3d()))
                .toArray(Pose3d[]::new);
        ballPublisher.set(ballPoses);
    }

    /* --- Physics Engine Class --- */
    // Moved to com.team9470.simulation.PhysicsSim
}
