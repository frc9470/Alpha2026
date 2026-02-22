package com.team9470.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team9470.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class AutoAimTest {
    private static Pose2d hubAlignedPose() {
        return new Pose2d(
                FieldConstants.Hub.topCenterPoint.getX() - 2.5,
                FieldConstants.Hub.topCenterPoint.getY(),
                new Rotation2d());
    }

    @Test
    void stationaryRobotProducesZeroTargetOmega() {
        var solution = AutoAim.calculate(hubAlignedPose(), new ChassisSpeeds());
        assertTrue(solution.isValid());
        assertEquals(0.0, solution.targetOmega(), 1e-9);
    }

    @Test
    void strafingLeftShiftsYawClockwiseAndOmegaNegative() {
        Pose2d pose = hubAlignedPose();
        var stationary = AutoAim.calculate(pose, new ChassisSpeeds());
        var moving = AutoAim.calculate(pose, new ChassisSpeeds(0.0, 1.0, 0.0));

        assertTrue(moving.isValid());
        assertTrue(
                moving.targetRobotYaw().getRadians() < stationary.targetRobotYaw().getRadians() - 1e-4,
                "Lookahead should bias yaw clockwise when the robot strafes left");
        assertTrue(moving.targetOmega() < 0.0, "Angular feedforward should also command clockwise rotation");
    }
}
