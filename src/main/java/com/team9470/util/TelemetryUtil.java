package com.team9470.util;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

public final class TelemetryUtil {
    private TelemetryUtil() {
    }

    public static DoublePublisher publishDouble(NetworkTable table, String key, String unit) {
        var topic = table.getDoubleTopic(key);
        topic.setProperty("unit", "\"" + unit + "\"");
        return topic.publish();
    }
}
