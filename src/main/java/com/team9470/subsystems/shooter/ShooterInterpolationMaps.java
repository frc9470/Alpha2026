package com.team9470.subsystems.shooter;

import java.util.Optional;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public final class ShooterInterpolationMaps {
    private ShooterInterpolationMaps() {
    }

    private static final double kFeedLaunchAngleMultiplier = 1.4;

    private static final InterpolatingTreeMap<InterpolatingDouble, ShotParameter> kHubMap = new InterpolatingTreeMap<>();
    private static final InterpolatingTreeMap<InterpolatingDouble, ShotParameter> kFeedMap = new InterpolatingTreeMap<>();

    static {

        addHubPoint(1.0, 15.0, 1850.0);// center doesn't work
        addHubPoint(1.25, 18.0, 1750.0);
        addHubPoint(1.5, 17.0, 1800.0);
        addHubPoint(1.75, 17.0, 1900.0);
        addHubPoint(2.0, 20.0, 1950.0);
        addHubPoint(3, 31.0, 2070.0);
        addHubPoint(3.5, 33.0, 2190.0);
        addHubPoint(4.0, 34.0, 2300.0);
        addHubPoint(4.5, 34.0, 2530.0);
        addHubPoint(5.0, 37.0, 2780.0);

        // Feed map starts as a copy of hub points and can be tuned independently.
        addFeedPoint(1.0, 15.0, 1850.0);
        addFeedPoint(1.25, 18.0, 1750.0);
        addFeedPoint(1.5, 17.0, 1800.0);
        addFeedPoint(1.75, 17.0, 1900.0);
        addFeedPoint(2.0, 20.0, 1950.0);
        addFeedPoint(3, 31.0, 2070.0);
        addFeedPoint(3.5, 33.0, 2190.0);
        addFeedPoint(4.0, 34.0, 2300.0);
        addFeedPoint(4.5, 34.0, 2530.0);
        addFeedPoint(5.0, 37.0, 2780.0);

    }

    public static Optional<ShotParameter> getHub(double distanceMeters) {
        ShotParameter parameter = kHubMap.getInterpolated(new InterpolatingDouble(distanceMeters));
        return Optional.ofNullable(parameter);
    }

    public static void addHubPoint(double distanceMeters, double hoodCommandDeg, double flywheelRpm) {
        kHubMap.put(new InterpolatingDouble(distanceMeters), new ShotParameter(hoodCommandDeg, flywheelRpm));
    }

    public static Optional<ShotParameter> getFeed(double distanceMeters) {
        ShotParameter parameter = kFeedMap.getInterpolated(new InterpolatingDouble(distanceMeters));
        if (parameter == null) {
            return Optional.empty();
        }
        return Optional.of(new ShotParameter(
                parameter.hoodCommandDeg() * kFeedLaunchAngleMultiplier,
                parameter.flywheelRpm()));
    }

    public static void addFeedPoint(double distanceMeters, double hoodCommandDeg, double flywheelRpm) {
        kFeedMap.put(new InterpolatingDouble(distanceMeters), new ShotParameter(hoodCommandDeg, flywheelRpm));
    }
}
