package frc.robot.subsystems.intake.arm;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.utils.selfCheck.SelfChecking;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public boolean connected = true;
        public String name = "IntakeArm";
        public boolean zeroing = false;
        public double positionDeg = 0.0;
        public double velocityDegPerSec = 0.0;
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}
    default void setPosition(double positionDeg) {}
    default void setVoltage(double volts) {}
    default void stop() {}
    default void zero(){}
    default void configureMotionMagic(double cruiseDegPerSec, double accelDegPerSec2, double neutralDeadband) {}
    default void setPID(double p, double i, double d, double ks, double kv, double kg) {}
    default void setPID(double p, double i, double d, double ks, double kv, double kg,
                        double velocityMax, double accelerationMax, double neutralDeadband) {
        setPID(p, i, d, ks, kv, kg);
        configureMotionMagic(velocityMax, accelerationMax,neutralDeadband);
    }
    default void setCurrentLimit(double amps) {}
    default void setBrakeMode(boolean brake) {}

    default List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<>();
    }
}
