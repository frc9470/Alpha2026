package com.team9470.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;

/**
 * Constants describing camera placement relative to the robot frame.
 */
public final class VisionConstants {
    private VisionConstants() {}

    public static final Transform3d FRONT_LEFT_CAMERA_OFFSET =
            new Transform3d(
                    Units.Inches.of(+12.290427),
                    Units.Inches.of(10.710),
                    Units.Inches.of(+8.803138),
                    new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(-45)));

    public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET =
            new Transform3d(
                    Units.Inches.of(+12.290427),
                    Units.Inches.of(-10.710),
                    Units.Inches.of(+8.803138),
                    new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(45)));
}
