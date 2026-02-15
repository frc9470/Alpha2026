package com.team9470.subsystems.shooter;

import com.team254.lib.util.Interpolable;

/**
 * Tuned shooter outputs for a distance lookup.
 * hoodCommandDeg is the commanded hood plane angle in degrees.
 */
public record ShotParameter(double hoodCommandDeg, double flywheelRpm) implements Interpolable<ShotParameter> {

    @Override
    public ShotParameter interpolate(ShotParameter other, double x) {
        double clampedX = Math.max(0.0, Math.min(1.0, x));
        double hood = hoodCommandDeg + (other.hoodCommandDeg - hoodCommandDeg) * clampedX;
        double rpm = flywheelRpm + (other.flywheelRpm - flywheelRpm) * clampedX;
        return new ShotParameter(hood, rpm);
    }

    public boolean isValid() {
        return Double.isFinite(hoodCommandDeg) && Double.isFinite(flywheelRpm) && flywheelRpm >= 0.0;
    }
}
