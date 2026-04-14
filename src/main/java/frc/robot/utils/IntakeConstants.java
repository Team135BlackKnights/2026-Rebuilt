package frc.robot.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int intakeMotorID = 40; 
    public static final int frontRollersMotorID = 42;
    public static final int intakeCurrentLimit = 20; // amps
    public static final int frontRollersCurrentLimit = 60; // amps
    public static final boolean intakeInverted = false;
    public static final boolean frontRollersInverted = true;
    public static final DCMotor intakeArmMotor = DCMotor.getKrakenX60Foc(1);
    public static final double frontRollersReduction = 26.0 / 12.0;
    public static final double intakeMOI = 0.0005; // kg m^2
    public static final double intakeArmLengthMeters = Units.inchesToMeters(12.0);
    public static final double frontRollersMOI = 0.0021; // kg m^2
    public static final double armMinAngleRad = 0.0;
    public static final double armMaxAngleRad = Units.degreesToRadians(75.0);
    public static final double armRadiansPerMechanismRotation = Units.rotationsToRadians(1.0);
    public static final double rollersDiameterMeters = Units.inchesToMeters(1.25);
    public static final double intakeReductinoToIntakeRollers = 24.0/8.0 * 32.0/24.0 * 15.0/24.0 *20.0/24.0; // 2.0833
    public static final double intakeArmReduction = 9.0*5.0*50/24.0; 
    public static final String frontRollersName = "FrontRollersMotor";

}
