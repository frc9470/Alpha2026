package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record AutoAimSolverSnapshot(
        int modeCode,
        boolean feedModeActive,
        double robotXBlueMeters,
        double distanceMeters,
        boolean valid,
        double hoodCommandRad,
        double flywheelRps) implements StructSerializable {
    public static final Struct<AutoAimSolverSnapshot> struct = StructGenerator.genRecord(AutoAimSolverSnapshot.class);
}
