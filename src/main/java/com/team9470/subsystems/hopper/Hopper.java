package com.team9470.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.team9470.Ports;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.subsystems.hopper.HopperConstants.*;

/**
 * Hopper subsystem - high volume game piece feed to shooter.
 * Driven by 2 Kraken X44 motors. Only runs during shooting.
 */
public class Hopper extends SubsystemBase {

    private static Hopper instance;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    // Hardware
    private final TalonFX motor1;
    private final TalonFX motor2;

    // Control
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private boolean running = false;

    // Status signals
    private final StatusSignal<AngularVelocity> motor1Velocity;
    private final StatusSignal<Current> motor1Current;

    private Hopper() {
        motor1 = new TalonFX(Ports.HOPPER_MOTOR_1.getDeviceNumber());
        motor2 = new TalonFX(Ports.HOPPER_MOTOR_2.getDeviceNumber());

        // Apply configs from HopperConstants
        motor1.getConfigurator().apply(HopperConstants.kMotor1Config);
        motor2.getConfigurator().apply(HopperConstants.kMotor2Config);

        // Status signals (just motor1 for telemetry)
        motor1Velocity = motor1.getVelocity();
        motor1Current = motor1.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, motor1Velocity, motor1Current);
        motor1.optimizeBusUtilization();
        motor2.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        double voltage = running ? kFeedVoltage : 0.0;
        motor1.setControl(voltageRequest.withOutput(voltage));
        motor2.setControl(voltageRequest.withOutput(voltage));

        SmartDashboard.putBoolean("Hopper/Running", running);
        SmartDashboard.putNumber("Hopper/Velocity", motor1Velocity.getValueAsDouble());
        SmartDashboard.putNumber("Hopper/Current", motor1Current.getValueAsDouble());
    }

    // --- Control ---

    public void setRunning(boolean run) {
        this.running = run;
    }

    public void run() {
        setRunning(true);
    }

    public void stop() {
        setRunning(false);
    }

    public boolean isRunning() {
        return running;
    }

    // --- Commands ---

    /**
     * Command to run the hopper while held.
     */
    public Command runCommand() {
        return this.startEnd(this::run, this::stop).withName("Hopper Run");
    }
}
