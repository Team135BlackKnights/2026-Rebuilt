package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface ArmIO {
    @AutoLog
    abstract class ArmIOInputs {
        double motorPositionRads = 0;
        double motorTemperatureC = 0;
        double voltage = 0;
        String name = "armIntakeMotor";
        double velocityRadsPerSec = 0;
        boolean connected = true;
        double currentAmps = 0;
    }
    
    default void updateInputs(ArmIOInputs inputs) {
    }

    /** set angle of manip*/
    default void applyPID(double meters) {
    }

    default void setCurrentLimit(int amps) {
    }

    /** Stop feeder */
    default void stop() {
    }
    default List<SelfChecking> getSelfCheckingHardware(){
        return new ArrayList<SelfChecking>();
    }
}
