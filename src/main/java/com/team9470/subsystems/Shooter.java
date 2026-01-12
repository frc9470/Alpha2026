package com.team9470.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team9470.Constants.ShooterConstants;
import com.team9470.simulation.PhysicsSim;
import com.team9470.util.AutoAim.ShootingSolution;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // helper function to create config
    private static TalonFXConfiguration createFlywheelConfig() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kFlywheelVkP;
        config.Slot0.kV = ShooterConstants.kFlywheelVkV;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current Limits (Supply) - Protect Breaker, allow burst for recovery
        config.CurrentLimits.withSupplyCurrentLimit(80.0) // Flat 80A limit
                .withSupplyCurrentLimitEnable(true);

        return config;
    }

    private static TalonFXConfiguration createHoodConfig() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kHoodVkP;
        config.Slot0.kG = ShooterConstants.kHoodVkG; // Gravity Feedforward
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.kHoodGearRatio;

        // Current Limits (Supply) - Safety for mechanism
        config.CurrentLimits.withSupplyCurrentLimit(40.0)
                .withSupplyCurrentLimitEnable(true);

        return config;
    }

    // Hardware
    private final TalonFX m_flywheelMaster = new TalonFX(ShooterConstants.kFlywheelMasterId);
    private final TalonFX m_flywheelSlave = new TalonFX(ShooterConstants.kFlywheelSlaveId);
    private final TalonFX m_hoodMotor = new TalonFX(ShooterConstants.kHoodMotorId);

    // Controls
    private final VelocityVoltage m_flywheelRequest = new VelocityVoltage(0);
    private final PositionVoltage m_hoodRequest = new PositionVoltage(0);

    // State
    private double m_targetSpeedRPS = 0.0;
    private double m_targetHoodAngleRotations = 0.0;
    private double m_targetExitSpeed = 0.0;

    // Simulation Context
    private Supplier<Pose2d> m_poseSupplier = null;
    private Supplier<ChassisSpeeds> m_speedsSupplier = null;

    // Firing Logic
    private boolean m_isFiring = false;
    private double m_lastLaunchTime = 0.0;
    private double m_measuredBPS = 0.0;

    private final List<PhysicsSim.Projectile> m_projectiles = new ArrayList<>();
    private final StructArrayPublisher<Pose3d> m_ballPublisher;
    // private final StructPublisher<Pose3d> m_turretPublisher; // No Turret
    private final StructPublisher<Pose3d> m_targetPublisher;

    // Simulation
    private final FlywheelSim m_flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(2),
                    ShooterConstants.kFlywheelMOI,
                    ShooterConstants.kFlywheelGearRatio),
            DCMotor.getKrakenX60(2),
            ShooterConstants.kFlywheelGearRatio);

    private final SingleJointedArmSim m_hoodSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.kHoodGearRatio,
            ShooterConstants.kHoodMOI,
            ShooterConstants.kHoodLength,
            ShooterConstants.kMinHoodAngle,
            ShooterConstants.kMaxHoodAngle,
            true, // Simulate Gravity
            0.0 // Starting Angle
    );

    public Shooter() {
        // Init Hardware
        m_flywheelMaster.getConfigurator().apply(createFlywheelConfig());
        m_flywheelSlave.getConfigurator().apply(createFlywheelConfig());
        m_hoodMotor.getConfigurator().apply(createHoodConfig());

        // Slave follows Master via simple duplicate control for now (Follower API
        // issue)
        // m_flywheelSlave.setControl(new Follower(m_flywheelMaster.getDeviceID(),
        // true));

        var table = NetworkTableInstance.getDefault().getTable("Sim");
        m_ballPublisher = table.getStructArrayTopic("BallPose", Pose3d.struct).publish();
        m_targetPublisher = table.getStructTopic("TargetPose", Pose3d.struct).publish();
    }

    public void setSimulationContext(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        this.m_poseSupplier = poseSupplier;
        this.m_speedsSupplier = speedsSupplier;
    }

    /**
     * Set the target shooting solution.
     * 
     * @param solution The calculated AutoAim solution.
     */
    public void setSetpoint(ShootingSolution solution) {
        if (!solution.isValid())
            return;

        m_targetExitSpeed = solution.speed();

        // Convert Speed (m/s) to RPM -> RPS
        // Speed = Radius * Omega -> Omega = Speed / Radius
        // Radius = 2 inches (~0.05m)? Need wheel radius.
        // Let's assume 4 inch wheel (0.05m radius).
        double wheelRadius = Units.inchesToMeters(2.0);
        // Compensate for efficiency loss
        double surfaceSpeed = solution.speed() / ShooterConstants.kFlywheelEfficiency;
        double radsPerSec = surfaceSpeed / wheelRadius;
        m_targetSpeedRPS = Units.radiansToRotations(radsPerSec) * ShooterConstants.kFlywheelGearRatio;

        // Convert Pitch (Radians from horizontal) to Hood Rotations
        // Hood 0 = Horizontal? Or Vertical?
        // Typically Hood 0 = Bottom stop.
        // Let's assume direct mapping for now.
        double hoodRadians = solution.pitch().getRadians();
        // Clamp to limits
        hoodRadians = Math.max(ShooterConstants.kMinHoodAngle, Math.min(ShooterConstants.kMaxHoodAngle, hoodRadians));
        m_targetHoodAngleRotations = Units.radiansToRotations(hoodRadians);
    }

    /**
     * Manual Launch command (just sets full speed for test)
     * In simulation, triggers a projectile spawn if at speed.
     */
    public void launch() {
        m_targetSpeedRPS = 80.0; // ~4800 RPM
        m_targetHoodAngleRotations = Units.degreesToRotations(45.0);
        m_targetExitSpeed = 30.0; // Approximate manual speed

        // Sim Launch Logic
        if (m_poseSupplier != null) {
            spawnProjectile();
        }
    }

    private void spawnProjectile() {
        Pose2d robotPose = m_poseSupplier.get();
        if (robotPose == null)
            return;

        // Calculate Exit State
        // Hood Angle (Pitch)
        // Assuming Motor Position 0 = Horizontal?
        // Or using Constants kMinHoodAngle?
        // Let's use Sim State for accuracy
        double hoodRotations = m_hoodMotor.getPosition().getValueAsDouble();
        double hoodRad = Units.rotationsToRadians(hoodRotations); // Assuming 0-based
        if (m_hoodSim != null)
            hoodRad = m_hoodSim.getAngleRads();

        // Flywheel Speed
        double flywheelRPS = m_flywheelMaster.getVelocity().getValueAsDouble();
        if (m_flywheelSim != null)
            flywheelRPS = m_flywheelSim.getAngularVelocityRPM() / 60.0;

        // This method is used when manual firing without a target, so flywheelRPS here
        // is
        // derived from actual physical state. If it was derived from motor, we should
        // divide by gear ratio.
        // But `m_flywheelSim.getAngularVelocityRPM()` IS the flywheel velocity, not
        // motor.
        // `m_flywheelMaster.getVelocity()` is MOTOR velocity.
        // If getting from motor, divide by GearRatio.
        if (m_flywheelSim == null) {
            flywheelRPS /= ShooterConstants.kFlywheelGearRatio;
        }
        // Wait, lines 191-193 above:
        // double flywheelRPS = m_flywheelMaster.getVelocity().getValueAsDouble();
        // if (m_flywheelSim != null) flywheelRPS =
        // m_flywheelSim.getAngularVelocityRPM() / 60.0;
        // FlywheelSim RPM IS the flywheel. Motor velocity is Motor.
        // So line 191 should be divided by gear ratio? Yes.
        // But I will fix the main spawn method which uses passed setpoints usually.
        // This 0-arg spawnProjectile seems to use current state. Let's fix 191 too.

        // Actually, let's just ensure the passed-in logic in the other overload is
        // correct.
        // For this method, let's fix it:
        if (m_flywheelSim == null) {
            flywheelRPS /= ShooterConstants.kFlywheelGearRatio;
        }

        double wheelRadius = Units.inchesToMeters(2.0); // 4 inch wheel

        // Efficiency & Energy Transfer
        double efficiency = ShooterConstants.kFlywheelEfficiency;
        double surfaceSpeed = flywheelRPS * 2.0 * Math.PI * wheelRadius;
        double speed = surfaceSpeed * efficiency;

        // Recoil
        if (m_flywheelSim != null) {
            double ballMass = 0.27; // kg
            double ballKE = 0.5 * ballMass * speed * speed;

            double moi = ShooterConstants.kFlywheelMOI;
            double flywheelRadPerSec = Units.rotationsToRadians(flywheelRPS);
            double flywheelKE = 0.5 * moi * flywheelRadPerSec * flywheelRadPerSec;

            double newFlywheelKE = Math.max(0, flywheelKE - ballKE);
            double newFlywheelRadPerSec = Math.sqrt(2 * newFlywheelKE / moi);

            m_flywheelSim.setState(VecBuilder.fill(newFlywheelRadPerSec));
        }
        // Assuming a fixed yaw for manual launch for now, or use robot's current yaw
        Rotation2d yawField = (robotPose != null) ? robotPose.getRotation() : new Rotation2d();
        spawnProjectile(m_targetSpeedRPS, Rotation2d.fromRotations(m_targetHoodAngleRotations), yawField);
    }

    private void spawnProjectile(double speedRPS, Rotation2d pitch, Rotation2d yawField) {
        // 1. Find Stored Note
        PhysicsSim.Projectile p = null;
        for (PhysicsSim.Projectile note : m_projectiles) {
            if (note.isStored) {
                p = note;
                break;
            }
        }

        if (p == null) {
            // Dry Fire
            return;
        }

        // 2. Calculate Launch Vector

        // Muzzle Position (Robot Center + Offset)
        Pose2d robotPose = com.team9470.subsystems.Swerve.getInstance().getPose();

        // Flywheel Tangential Speed
        // v = r * w
        // r = 0.05 (Wheel Radius approx 2 inches?)
        // speedRPS is MOTOR RPS. We need FLYWHEEL RPS.
        double gearRatio = com.team9470.Constants.ShooterConstants.kFlywheelGearRatio;
        double flywheelRPS = speedRPS / gearRatio;

        double wheelRadius = 0.05; // m
        double exitSpeed = flywheelRPS * 2.0 * Math.PI * wheelRadius;

        // Apply Efficiency
        exitSpeed *= com.team9470.Constants.ShooterConstants.kFlywheelEfficiency;

        // Pitch & Yaw Vectors
        // Orientation: Pitch is up from horizontal. Yaw is field relative.
        double vZ = exitSpeed * pitch.getSin();
        double vXY = exitSpeed * pitch.getCos();
        double vX = vXY * yawField.getCos();
        double vY = vXY * yawField.getSin();

        Translation3d vWorld = new Translation3d(vX, vY, vZ);

        // Add Robot Velocity
        ChassisSpeeds chassisSpeeds = com.team9470.subsystems.Swerve.getInstance().getChassisSpeeds();
        // Translate Robot Relative Chassis Speeds (vx, vy) to Field Relative Velocity
        // vector
        // This vector (vRobot) is then added to the ball's launch vector (vWorld)
        // Note: Field Relative Velocity = Robot Velocity rotated by Robot Heading
        double rCos = robotPose.getRotation().getCos();
        double rSin = robotPose.getRotation().getSin();
        double vRobotX = chassisSpeeds.vxMetersPerSecond * rCos - chassisSpeeds.vyMetersPerSecond * rSin;
        double vRobotY = chassisSpeeds.vxMetersPerSecond * rSin + chassisSpeeds.vyMetersPerSecond * rCos;

        Translation3d vRobot = new Translation3d(vRobotX, vRobotY, 0.0);
        vWorld = vWorld.plus(vRobot);

        // 3. Launch Stored Note
        p.isStored = false;

        // Shooter Offset: Centralized in Constants
        double offX = ShooterConstants.kShooterOffsetX;
        double offZ = ShooterConstants.kShooterOffsetZ;
        // Rotate Offset
        Translation3d p0 = new Translation3d(
                robotPose.getX() + (offX * robotPose.getRotation().getCos()),
                robotPose.getY() + (offX * robotPose.getRotation().getSin()),
                offZ);

        p.position = p0;
        p.velocity = vWorld;

        // Add Backspin (Magnus Lift)
        // Rolling condition: w = v / r
        // Axis: Horizontal, perpendicular to velocity (Right Hand Rule: V x Up = Right)
        // Spin around Right Vector produces Upward Lift
        double ballRadius = PhysicsSim.Projectile.RADIUS;
        double speed = vWorld.getNorm();
        if (speed > 1e-6) {
            // Spin Axis = Velocity x Up = (vy, -vx, 0)
            Translation3d spinAxis = new Translation3d(vWorld.getY(), -vWorld.getX(), 0.0);
            if (spinAxis.getNorm() > 1e-6) {
                spinAxis = spinAxis.div(spinAxis.getNorm());
            }
            double omegaRadPerSec = speed / ballRadius;
            p.angularVelocity = spinAxis.times(omegaRadPerSec);
        } else {
            p.angularVelocity = new Translation3d();
        }
    }

    public void stop() {
        m_targetSpeedRPS = 0.0;
        m_targetExitSpeed = 0.0;
    }

    public void setFiring(boolean firing) {
        m_isFiring = firing;
    }

    public boolean isAtSetpoint() {
        double flywheelErr = Math.abs(m_targetSpeedRPS - m_flywheelMaster.getVelocity().getValueAsDouble());
        double hoodErr = Math.abs(m_targetHoodAngleRotations - m_hoodMotor.getPosition().getValueAsDouble());
        // Sim State Check (since we are in sim)
        if (m_flywheelSim != null) {
            // Compare Motor Target RPS vs (Flywheel Sim RPM / 60 * GearRatio)
            double simMotorRPS = (m_flywheelSim.getAngularVelocityRPM() / 60.0) * ShooterConstants.kFlywheelGearRatio;
            flywheelErr = Math.abs(m_targetSpeedRPS - simMotorRPS);

            hoodErr = Math.abs(m_targetHoodAngleRotations
                    - Units.radiansToRotations(m_hoodSim.getAngleRads()));
        }

        return flywheelErr < 0.5 && hoodErr < 0.1; // 0.5 RPS (~30 RPM), 0.1 Rotations
    }

    @Override
    public void periodic() {
        // Apply Controls
        m_flywheelMaster.setControl(m_flywheelRequest.withVelocity(m_targetSpeedRPS));
        m_flywheelSlave.setControl(m_flywheelRequest.withVelocity(m_targetSpeedRPS));
        m_hoodMotor.setControl(m_hoodRequest.withPosition(m_targetHoodAngleRotations));

        // Telemetry
        SmartDashboard.putNumber("Shooter/Flywheel/TargetRPM", m_targetSpeedRPS * 60.0);
        SmartDashboard.putNumber("Shooter/Flywheel/ActualRPM",
                m_flywheelMaster.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("Shooter/Hood/TargetDeg", Units.rotationsToDegrees(m_targetHoodAngleRotations));
        SmartDashboard.putNumber("Shooter/Hood/ActualDeg",
                Units.rotationsToDegrees(m_hoodMotor.getPosition().getValueAsDouble()));
        SmartDashboard.putBoolean("Shooter/IsAtSetpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Shooter/IsFiring", m_isFiring);

        // Debug Errors
        double flywheelErr = Math.abs(m_targetSpeedRPS - m_flywheelMaster.getVelocity().getValueAsDouble());
        double hoodErr = Math.abs(m_targetHoodAngleRotations - m_hoodMotor.getPosition().getValueAsDouble());

        // Calculate Exit Velocity Error
        double currentRPS = m_flywheelMaster.getVelocity().getValueAsDouble();

        // Use Sim error if in Sim
        if (m_flywheelSim != null) {
            double simMotorRPS = (m_flywheelSim.getAngularVelocityRPM() / 60.0) * ShooterConstants.kFlywheelGearRatio;
            flywheelErr = Math.abs(m_targetSpeedRPS - simMotorRPS);

            hoodErr = Math.abs(m_targetHoodAngleRotations - Units.radiansToRotations(m_hoodSim.getAngleRads()));
            currentRPS = simMotorRPS; // Use Motor RPS for consistency? or Flywheel RPS for Exit Vel?
            // Exit Velocity depends on FLYWHEEL speed, not Motor.
        }

        // Actual Exit Speed Calculation
        // Exit Speed = Flywheel Surface Speed * Efficiency
        // Flywheel RPS = Motor RPS / GearRatio
        double flywheelRPS = currentRPS / ShooterConstants.kFlywheelGearRatio;

        double wheelRadius = Units.inchesToMeters(2.0);
        double actualExitSpeed = (flywheelRPS * 2.0 * Math.PI * wheelRadius) * ShooterConstants.kFlywheelEfficiency;
        double exitVelErr = Math.abs(m_targetExitSpeed - actualExitSpeed);

        SmartDashboard.putNumber("Debug/FlywheelErrRPM", flywheelErr * 60.0);
        SmartDashboard.putNumber("Debug/HoodErrRot", hoodErr);
        SmartDashboard.putNumber("Debug/ExitVelErr", exitVelErr);
        SmartDashboard.putNumber("Debug/MeasuredBPS", m_measuredBPS);
    }

    private final List<Double> m_shotTimestamps = new ArrayList<>();

    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();

        // 1. Check if we should launch (Flywheel at speed + Hood aligned + Firing
        // Requested)
        // In Sim, we can just check if we are "close enough" and "firing"
        // But for physics realism, let's use the recovery of the flywheel to dictate
        // fire
        // rate.

        // Rolling Average BPS Logic
        if (m_isFiring) {
            // Prune old shots (> 1.0s ago)
            m_shotTimestamps.removeIf(t -> (now - t) > 1.0);
            m_measuredBPS = (double) m_shotTimestamps.size();
        } else {
            m_shotTimestamps.clear();
            m_measuredBPS = 0.0;
        }

        // Update Physics Models
        m_flywheelSim.setInputVoltage(m_flywheelMaster.getSimState().getMotorVoltage());
        m_hoodSim.setInputVoltage(m_hoodMotor.getSimState().getMotorVoltage());

        m_flywheelSim.update(0.02);
        m_hoodSim.update(0.02);

        if (m_isFiring && isAtSetpoint()) {
            // Recovered?
            // Logic: If isAtSetpoint is true (Flywheel err < 0.5 RPS), we are ready.
            // We just fire. The physics sim handling the energy loss will naturally
            // drop the RPM below setpoint, making isAtSetpoint false for a bit (recovery
            // time).

            spawnProjectile();
            m_shotTimestamps.add(now);
            m_lastLaunchTime = now;
        }

        // Update Simulated Sensors
        m_flywheelMaster.getSimState().setRawRotorPosition(m_flywheelSim.getAngularVelocityRPM() / 60.0 * 0.02
                + m_flywheelMaster.getPosition().getValueAsDouble());
        m_flywheelMaster.getSimState().setRotorVelocity(
                m_flywheelSim.getAngularVelocityRPM() / 60.0 * ShooterConstants.kFlywheelGearRatio);

        m_hoodMotor.getSimState().setRawRotorPosition(
                Units.radiansToRotations(m_hoodSim.getAngleRads()) * ShooterConstants.kHoodGearRatio);
        m_hoodMotor.getSimState().setRotorVelocity(
                Units.radiansToRotations(m_hoodSim.getVelocityRadPerSec()) * ShooterConstants.kHoodGearRatio);

        // Auto-Fire Logic
        if (m_isFiring && isAtSetpoint()) {
            double dt = now - m_lastLaunchTime;
            if (dt > 1e-4) { // Avoid divide by zero
                m_measuredBPS = 1.0 / dt;
            }
            spawnProjectile();
            m_shotTimestamps.add(now);
            m_lastLaunchTime = now;
        } else if (now - m_lastLaunchTime > 0.5) {
            m_measuredBPS = 0.0;
        }

        // Update Projectiles
        updateProjectiles(0.02);

        // Publish
        Pose3d[] ballPoses = m_projectiles.stream()
                .map(p -> new Pose3d(p.position, new Rotation3d()))
                .toArray(Pose3d[]::new);
        m_ballPublisher.set(ballPoses);
        m_targetPublisher.set(new Pose3d(com.team9470.util.AutoAim.getTarget(), new Rotation3d()));
    }

    private void updateProjectiles(double dt) {
        // 1. Update Physics & Scoring via Unified Simulator
        edu.wpi.first.math.geometry.Pose2d robotPose = com.team9470.subsystems.Swerve.getInstance().getPose();

        // Fetch Robot Relative Speeds (Forward/Strafe)
        edu.wpi.first.math.kinematics.ChassisSpeeds robotRelSpeeds = com.team9470.subsystems.Swerve.getInstance()
                .getChassisSpeeds();

        // Convert to Field Relative (World X/Y) for Physics Collision
        // We use ChassisSpeeds.fromRobotRelativeSpeeds to rotate the robot-relative
        // velocity vectors into the field frame based on robot rotation
        edu.wpi.first.math.kinematics.ChassisSpeeds fieldSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds
                .fromRobotRelativeSpeeds(
                        robotRelSpeeds, robotPose.getRotation());

        var intake = com.team9470.subsystems.Intake.getInstance();
        double intakeAngle = intake.getPivotAngle();
        boolean intakeActive = intake.isRunning();

        PhysicsSim.simulate(dt, m_projectiles, com.team9470.util.AutoAim.getTarget(), robotPose, fieldSpeeds,
                intakeAngle,
                intakeActive);

        // 2. Remove Scored Projectiles & Bounds Check
        Iterator<PhysicsSim.Projectile> iter = m_projectiles.iterator();
        while (iter.hasNext()) {
            PhysicsSim.Projectile p = iter.next();
            if (p.isScored) {
                iter.remove();
            } else if (p.position.getZ() <= 0 && p.velocity.getZ() < 0.1) {
                // Should we remove? Maybe not.
            }
        }
        // Collision resolution is now handled inside simulate().
    }

    // Debug: Spawn some notes
    // Debug: Spawn Fuel Grid
    public void seedField() {
        double fieldCenterX = 16.54 / 2.0;
        double fieldCenterY = 8.05 / 2.0;

        double fuelDepth = Units.inchesToMeters(72.0); // X-axis depth
        double fuelWidth = Units.inchesToMeters(206.0); // Y-axis width
        double gapWidth = Units.inchesToMeters(9.0); // 2in gap? Image looks wider approx. 2in.

        double spacing = 0.16; // 16cm spacing (Fuel Diam = 15cm)

        double minX = fieldCenterX - (fuelDepth / 2.0);
        double maxX = fieldCenterX + (fuelDepth / 2.0);
        double minY = fieldCenterY - (fuelWidth / 2.0);
        double maxY = fieldCenterY + (fuelWidth / 2.0);

        for (double x = minX; x < maxX; x += spacing) {
            for (double y = minY; y < maxY; y += spacing) {
                // Gap Check
                if (Math.abs(y - fieldCenterY) < (gapWidth / 2.0)) {
                    continue;
                }

                m_projectiles.add(new PhysicsSim.Projectile(
                        new Translation3d(x, y, 0.1), // Slightly off ground
                        new Translation3d()));
            }
        }
    }
}
