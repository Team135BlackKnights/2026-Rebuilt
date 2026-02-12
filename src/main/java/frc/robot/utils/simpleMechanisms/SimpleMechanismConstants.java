package frc.robot.utils.simpleMechanisms;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.utils.drive.DriveConstants.MotorVendor;

public class SimpleMechanismConstants {

    public static class Climber {
        public static final int id = 41;
        public static final CANBus bus = Robot.rioCanBus;
        public static final String name = "ClimbMotor";
        public static final MotorVendor motorType = MotorVendor.CTRE_ON_RIO;
        public static final int currentLimitAmps = 40;
        public static final boolean invert = true;
        public static final double reduction = 60.0 / 1.0;
        public static final double maxLengthMeters = Units.inchesToMeters(15.25);
        public static final double drumRadiusMeters = Units.inchesToMeters(1.275);

    }
    // Arms, either single or double, should ALWAYS use state space models.
}
