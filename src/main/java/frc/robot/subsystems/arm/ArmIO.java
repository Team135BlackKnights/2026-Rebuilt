package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface ArmIO {
    @AutoLog
    abstract class LiftIOInputs {
        double liftPositionMeters = 0;
        double leftMotorTemperatureC = 0;
        double rightMotorTemperatureC = 0;
        double voltage = 0;
        String name = "armIntakeMotor";
        double velocityRadsPerSec = 0;
        boolean connected = true;
        double currentAmps = 0;
    }
    
    default void updateInputs(LiftIOInputs inputs) {
    }

    /** set angle of manip*/
    default void setAngle(double meters) {
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
