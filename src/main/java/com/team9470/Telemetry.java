package com.team9470;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.DriveStatusSnapshot;

/**
 * Publishes drivetrain telemetry using canonical NT4 struct topics.
 */
public class Telemetry {
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    public Telemetry(double maxSpeedMetersPerSecond) {
        SignalLogger.start();
    }

    public void telemeterize(SwerveDriveState state) {
        double odometryFrequencyHz = state.OdometryPeriod > 0.0 ? 1.0 / state.OdometryPeriod : 0.0;

        telemetry.publishDrivePose(state.Pose);
        telemetry.publishDriveSpeeds(state.Speeds);
        telemetry.publishDriveModuleStates(state.ModuleStates);
        telemetry.publishDriveModuleTargets(state.ModuleTargets);
        telemetry.publishDriveModulePositions(state.ModulePositions);
        telemetry.publishDriveStatus(new DriveStatusSnapshot(state.Timestamp, odometryFrequencyHz, state.OdometryPeriod));
    }
}
