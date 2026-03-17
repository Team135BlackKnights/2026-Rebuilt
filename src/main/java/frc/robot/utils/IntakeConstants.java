package frc.robot.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final int intakeMotorID = 40; 
    public static final int frontRollersMotorID = 42;
    public static final int intakeCurrentLimit = 80; // amps
    public static final int frontRollersCurrentLimit = 30; // amps
    public static final boolean intakeInverted = true;
    public static final boolean frontRollersInverted = true;
    public static final DCMotor intakeArmMotor = DCMotor.getKrakenX44Foc(1);
    public static final double frontRollersReduction = 2; //TODO
    public static final double intakeMOI = 0.0005; // kg m^2
    public static final double slideCarriageMassKg = 4.0;
    public static final double frontRollersMOI = 0.0021; // kg m^2
    public static final double slideMinInches = 0.0;
    public static final double slideMaxInches = 12.0;
    public static final double slideInchesPerMechanismRotation = Units.metersToInches(0.005) * 36;
    public static final double rollersDiameterMeters = Units.inchesToMeters(1.25);
    public static final double intakeReductinoToIntakeRollers = 24.0/8.0 * 32.0/24.0 * 15.0/24.0 *20.0/24.0; // 2.0833
    public static final double intakeArmReduction = 50/10.0; 
    public static final String frontRollersName = "FrontRollersMotor";

}
