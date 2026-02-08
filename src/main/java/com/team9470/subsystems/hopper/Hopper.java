package com.team9470.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import com.team9470.Ports;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.subsystems.hopper.HopperConstants.*;

/**
 * Hopper subsystem - high volume game piece feed to shooter.
 * Driven by 3 Kraken X44 motors (left, right, top). Only runs during shooting.
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
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX topMotor;

    // Control
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private boolean running = false;

    // Status signals
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Current> leftCurrent;

    private Hopper() {
        leftMotor = TalonFXFactory.createDefaultTalon(Ports.HOPPER_LEFT);
        rightMotor = TalonFXFactory.createDefaultTalon(Ports.HOPPER_RIGHT);
        topMotor = TalonFXFactory.createDefaultTalon(Ports.HOPPER_TOP);

        // Apply configurations
        TalonUtil.applyAndCheckConfiguration(leftMotor, HopperConstants.kLeftConfig);
        TalonUtil.applyAndCheckConfiguration(rightMotor, HopperConstants.kRightConfig);
        TalonUtil.applyAndCheckConfiguration(topMotor, HopperConstants.kTopConfig);

        // Status signals (just left motor for telemetry)
        leftVelocity = leftMotor.getVelocity();
        leftCurrent = leftMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50, leftVelocity, leftCurrent);
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
        topMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        double voltage = running ? kFeedVoltage : 0.0;
        leftMotor.setControl(voltageRequest.withOutput(voltage));
        rightMotor.setControl(voltageRequest.withOutput(voltage));
        topMotor.setControl(voltageRequest.withOutput(voltage));

        SmartDashboard.putBoolean("Hopper/Running", running);
        SmartDashboard.putNumber("Hopper/Velocity", leftVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Hopper/Current", leftCurrent.getValueAsDouble());
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
