package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    // ==================== INTAKE ====================
    public static final CanDeviceId INTAKE_ROLLERS = new CanDeviceId(13, "rio");
    public static final CanDeviceId INTAKE_ARM = new CanDeviceId(14, "rio");

    // ==================== INDEXER ====================
    public static final CanDeviceId INDEXER_1 = new CanDeviceId(15, "rio");
    public static final CanDeviceId INDEXER_2 = new CanDeviceId(16, "rio");

    // ==================== ELEVATOR ====================
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(17, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(18, "rio");

    // ==================== ARM ====================
    public static final CanDeviceId ARM_PIVOT = new CanDeviceId(19, "rio");
    public static final CanDeviceId ARM_ROLLERS = new CanDeviceId(20, "rio");

    // ==================== SHOOTER ====================
    public static final CanDeviceId FLYWHEEL_MASTER = new CanDeviceId(50, "rio");
    public static final CanDeviceId FLYWHEEL_SLAVE = new CanDeviceId(51, "rio");
    public static final CanDeviceId FLYWHEEL_SLAVE_2 = new CanDeviceId(53, "rio");
    public static final CanDeviceId FLYWHEEL_SLAVE_3 = new CanDeviceId(54, "rio");
    public static final CanDeviceId HOOD_MOTOR = new CanDeviceId(52, "rio");

    // ==================== HOPPER ====================
    public static final CanDeviceId HOPPER_MOTOR_1 = new CanDeviceId(55, "rio");
    public static final CanDeviceId HOPPER_MOTOR_2 = new CanDeviceId(56, "rio");

    // ==================== INTAKE (new subsystem) ====================
    public static final CanDeviceId INTAKE_PIVOT = new CanDeviceId(60, "rio");
    public static final CanDeviceId INTAKE_ROLLER = new CanDeviceId(61, "rio");

    // ==================== PNEUMATICS ====================
    public static final int CRADLE_BREAK = 0;
    public static final int INTAKE_BREAK = 1;
}
