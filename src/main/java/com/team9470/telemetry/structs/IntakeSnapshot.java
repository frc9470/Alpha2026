package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record IntakeSnapshot(
        boolean homing,
        boolean deployed,
        boolean agitating,
        int stateCode,
        double goalAngleRad,
        double setpointAngleRad,
        double positionAngleRad,
        double errorAngleRad,
        double pivotVelocityRadPerSec,
        double pivotSupplyCurrentAmps,
        double pivotAppliedVolts,
        double rollerAppliedVolts,
        double rollerSupplyCurrentAmps) implements StructSerializable {
    public static final Struct<IntakeSnapshot> struct = StructGenerator.genRecord(IntakeSnapshot.class);
}
