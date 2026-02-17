package frc.robot.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int intakeMotorID = 40; 
    public static final int indexerMotorID = 41;
    public static final int frontRollersMotorID = 42;
    public static final int intakeCurrentLimit = 30; // amps
    public static final int indexerCurrentLimit = 30; // amps
    public static final int frontRollersCurrentLimit = 30; // amps
    public static final boolean indexerInverted = true;
    public static final boolean intakeInverted = true;
    public static final boolean frontRollersInverted = true;
    public static final DCMotor intakeArmMotor = DCMotor.getKrakenX44Foc(1);
    public static final double intakeReductionToIndexerRollers = 24.0/8.0 * 32.0/24.0; // 4.0
    public static final double frontRollersReduction = 2; //TODO
    public static final double intakeMOI = 0.0005; // kg m^2
    public static final double armMOI = 0.0007; // kg m^2
    public static final double frontRollersMOI = 0.0006; // kg m^2
    public static final double armMinAngleRads = Math.toRadians(0.0);
    public static final double armMaxAngleRads = Math.toRadians(135.0);
    public static final double rollersDiameterMeters = Units.inchesToMeters(1.25);
    public static final double intakeReductinoToIntakeRollers = 24.0/8.0 * 32.0/24.0 * 15.0/24.0 *20.0/24.0; // 2.0833
    public static final double intakeArmReduction = 84.0/12.0; // 7.0
    public static final String frontRollersName = "FrontRollersMotor";

}
