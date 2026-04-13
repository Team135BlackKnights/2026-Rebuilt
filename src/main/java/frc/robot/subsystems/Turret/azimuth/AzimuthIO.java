package frc.robot.subsystems.Turret.azimuth;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;


public interface AzimuthIO {
    @AutoLog
    public class AzimuthIOInputs{
        public double tempCelsius = 0;
        public double motorPositionRads = 0;
        public double turretPositionRads = 0;
        public double bigEncoderRads = 0;
        public double smallEncoderRads = 0;
        public double appliedVoltage = 0;
        public boolean motorConnected = true;
        public boolean bothEncodersConnected = true;
        public boolean referenceEncoderConnected = true;
        public boolean magSwitchDetected = false;
        public boolean zeroed = false;
        public String name = "Azimuth";
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double motorVelocityRadsPerSec = 0;
        public double turretVelocityRadsPerSec = 0;
    }

    default void updateInputs(AzimuthIOInputs inputs) {}
    /** Should range from -pi to pi, auto handled for CRT */
    default void setDesiredPosition(double positionRads) {}
    default void stop() {}
    default void disable() {}
    default void enable() {}
    default void runVolts(double volts) {}
    default void setPID(double p, double i, double d, double ks, double kv, double ka, double velocityMax, double accelerationMax, double rampRate) {}
    default void setBrakeMode(boolean brake) {}
    default void setCurrentLimit(double amps) {}
    default void requestRezero() {}
    default boolean wantsZeroing() { return false; }
    default List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<SelfChecking>();
    }
}
