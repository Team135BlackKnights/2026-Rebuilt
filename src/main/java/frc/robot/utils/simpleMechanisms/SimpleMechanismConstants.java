package frc.robot.utils.simpleMechanisms;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.utils.drive.DriveConstants.MotorVendor;

public class SimpleMechanismConstants {

    public static class Climber {
        public static final int climberId = 41;
        public static final int wedgeArmId = 42;
        public static final CANBus bus = Robot.rioCanBus;
        public static final String climberName = "ClimbMotor";
        public static final String wedgeArmName = "WedgeArmMotor";
        public static final MotorVendor climbMotorType = MotorVendor.CTRE_ON_RIO;
        public static final MotorVendor wedgeArmMotorType = MotorVendor.CTRE_ON_RIO;
        public static final int climbCurrentLimit = 30; // amps
        public static final int wedgeArmCurrentLimit = 30; // amps
        public static final boolean climbInverted = true;
        public static final boolean wedgeArmInverted = true;
        public static final DCMotor wedgeArmMotor = DCMotor.getKrakenX44Foc(1);
        public static final DCMotor climbMotor = DCMotor.getKrakenX44Foc(1);
        public static final double wedgeReduction = 1; // TODO
        public static final double climbMOI = 0.0005; // kg m^2 TODO
        public static final double wedgeMOI = 0.095; // kg m^2 TODO
        public static final double wedgeMinAngleRads = Math.toRadians(0.0);
        public static final double wedgeMaxAngleRads = Math.toRadians(100.0); // actually around 72, but yk
        public static final double climbRollersDiameterMeters = Units.inchesToMeters(2);
        public static final double climbReductionToClimbRollers = 1; // TODO

    }
}
