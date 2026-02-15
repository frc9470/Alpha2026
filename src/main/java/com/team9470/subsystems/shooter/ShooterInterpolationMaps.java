package com.team9470.subsystems.shooter;

import java.util.Optional;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public final class ShooterInterpolationMaps {
    private ShooterInterpolationMaps() {
    }

    private static final InterpolatingTreeMap<InterpolatingDouble, ShotParameter> kSpeakerMap = new InterpolatingTreeMap<>();

    static {
        // Intentionally empty. Add tuned (distance meters -> hoodDeg, rpm) points here.
        // Example:
        // addSpeakerPoint(2.5, 27.0, 3200.0);
    }

    public static Optional<ShotParameter> getSpeaker(double distanceMeters) {
        ShotParameter parameter = kSpeakerMap.getInterpolated(new InterpolatingDouble(distanceMeters));
        return Optional.ofNullable(parameter);
    }

    public static void addSpeakerPoint(double distanceMeters, double hoodCommandDeg, double flywheelRpm) {
        kSpeakerMap.put(new InterpolatingDouble(distanceMeters), new ShotParameter(hoodCommandDeg, flywheelRpm));
    }
}
