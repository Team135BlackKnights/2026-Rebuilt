package frc.robot.subsystems.shooter.turret;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;


public interface TurretIO {
    @AutoLog
    public class TurretIOInputs{
        double tempCelsius = 0;
        double positionRads = 0;
        double appliedVoltage = 0;
        boolean connected = true;
        String name = "swivelMotor";
        double supplyCurrentAmps;
        double torqueCurrentAmps;
        double velocityRadsPerSec = 0;
    }

    public class TurretIOOutputs{
        public double velocityRadsPerSec = 0.0;
        public double kP = 0.0;
        public double kD = 0.0;
        public boolean coast = false;
    }


    default void updateInputs(TurretIOInputs inputs) {}
    default void setCurrentLimit(double amps) {}
    default List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<SelfChecking>();
    }
    default void applyOutputs(TurretIOOutputs outputs) {}
}
