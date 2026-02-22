package frc.robot.subsystems.intake.arm;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.utils.selfCheck.SelfChecking;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public boolean connected = true;
        public String name = "Arm";
        public double positionRads = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}
    default void setPosition(double positionRads) {}
    default void setVoltage(double volts) {}
    default void stop() {}
    default void zero(){}
    default void configureMotionMagic(double cruiseRadPerSec, double accelRadPerSec2, double jerkRadPerSec3) {}
    default void setPID(double p, double i, double d, double ks, double kv, double kg) {}
    default void setCurrentLimit(double amps) {}
    default void setBrakeMode(boolean brake) {}

    default List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<>();
    }
}