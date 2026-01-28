package frc.robot.subsystems.Shooter.flywheel;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public class FlywheelIOOutputs {
    public double velocityRadsPerSec = 0.0;
    public double feedForward = 0.0;
    public boolean coast = true;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}
  default void setCurrentLimit(double amps) {}
  default List<SelfChecking> getSelfCheckingHardware() {
    return new ArrayList<SelfChecking>();
  }
  default void applyOutputs(FlywheelIOOutputs outputs) {}
}