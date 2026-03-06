package frc.robot.subsystems.Turret.hood;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface HoodIO {
    @AutoLog
    public class HoodIOInputs {
        public boolean connected;
        public String name;
        public double positionRads;
        public double velocityRadsPerSec;
        public double appliedVoltage;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius;
    }
    default void updateInputs(HoodIOInputs inputs) {}
    default void setPosition(double positionRads) {}
    default void runVolts(double volts) {}
    default void stop() {}
    /** Start the zeroing routine (if supported by implementation). */
    default void zero() {}
    default void setPID(
        double p,
        double d,
        double ks,
        double kv) {}
    default void setCurrentLimit(double amps) {}
    default void setBrakeMode(boolean brake) {}

    default List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<SelfChecking>();
    }

}
