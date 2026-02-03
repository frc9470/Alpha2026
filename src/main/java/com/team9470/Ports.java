package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    // CANivore bus name
    public static final String CANIVORE = "canivore";

    // ==================== SHOOTER ====================
    public static final CanDeviceId FLYWHEEL_MASTER = new CanDeviceId(50, CANIVORE);
    public static final CanDeviceId FLYWHEEL_SLAVE = new CanDeviceId(51, CANIVORE);
    public static final CanDeviceId FLYWHEEL_SLAVE_2 = new CanDeviceId(53, CANIVORE);
    public static final CanDeviceId FLYWHEEL_SLAVE_3 = new CanDeviceId(54, CANIVORE);
    public static final CanDeviceId HOOD_MOTOR = new CanDeviceId(52, CANIVORE);

    // ==================== HOPPER ====================
    public static final CanDeviceId HOPPER_MOTOR_1 = new CanDeviceId(55, CANIVORE);
    public static final CanDeviceId HOPPER_MOTOR_2 = new CanDeviceId(56, CANIVORE);

    // ==================== INTAKE ====================
    public static final CanDeviceId INTAKE_PIVOT = new CanDeviceId(60, CANIVORE);
    public static final CanDeviceId INTAKE_ROLLER = new CanDeviceId(61, CANIVORE);
}
