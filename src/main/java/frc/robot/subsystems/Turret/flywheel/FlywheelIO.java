package frc.robot.subsystems.Turret.flywheel;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public boolean connected;
    public String name;
    public double accelRadsPerSec2;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }
  default void setVelocity(double velocityRadsPerSec) {}
  default void runVolts(double volts) {}
  default void stop() {}
  default void setPID(
      double p,
      double d,
      double ks,
      double kv,
      double ka) {}
  default void updateInputs(FlywheelIOInputs inputs) {}
  default void setBrakeMode(boolean brake) {}
  default void setCurrentLimit(double amps) {}
  default void setRamp(double closedLoopRampSecs) {}
  default List<SelfChecking> getSelfCheckingHardware() {
    return new ArrayList<SelfChecking>();
  }
}