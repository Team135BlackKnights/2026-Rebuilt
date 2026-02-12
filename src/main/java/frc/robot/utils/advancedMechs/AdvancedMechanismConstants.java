package frc.robot.utils.advancedMechs;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class AdvancedMechanismConstants {
        public static class Turret {
                public static final double minTurretAngle = -Math.PI;
                public static final double maxTurretAngle = Math.PI;
                public static final double minHoodAngle = Math.toRadians(12.0);
                public static final double maxHoodAngle = Math.toRadians(50.0);
                public static final int currentLimitAzimuth = 30;
                public static final int currentLimitFlywheel = 40;
                public static final int currentLimitHood = 20;
                public static final int currentLimitKickup = 50;
                public static final int turretTeeth = 77;
                public static final int idlerTeeth = 10;
                public static final double motorRadPerTurretRad = 36.0 / 11.0 * 77.0 / 10.0;// motor to turret ratio
                public static final double flywheelRatio = 1.0 / 1.0;
                public static final double hoodMotorToHoodEncoderRatio = 50.0 / 12.0;
                public static final double hoodEncoderToHoodArmRatio = 14.0 / 12.0;
                public static final double kickerRatio = 5.0 / 1.0;
                public static final double enc1GearTeeth = 36;
                public static final double enc2GearTeeth = 34;
                public static final DCMotor azimuthMotor = DCMotor.getKrakenX44Foc(1);
                public static final DCMotor flywheelMotor = DCMotor.getKrakenX60Foc(2);
                public static final DCMotor hoodMotor = DCMotor.getMinion(1);
                public static final DCMotor kickupMotor = DCMotor.getKrakenX44Foc(1);
                public static final double azimuthMOI = 0.002; // kg m^2
                public static final double flywheelMOI = 0.0005; // kg m
                public static final double hoodMOI = 0.0001; // kg m^2
                public static final double kickupMOI = 0.0002; // kg m^2
                public static final boolean invertKickup = false;

                // Left Turret
                public static final int leftAzimuthID = 22;
                public static final int leftAzimuthBigEncoderID = 23;
                public static final int leftAzimuthSmallEncoderID = 24;
                public static final int leftFlywheelID = 25;
                public static final int leftHoodID = 26;
                public static final int leftHoodEncoderID = 27;
                public static final int leftKickupID = 28;
                public static final String leftName = "LeftTurret";
                public static final Transform2d robotToLeftTurretHoleCenter = new Transform2d();

                // Right Turret
                public static final int rightAzimuthID = 29;
                public static final int rightAzimuthBigEncoderID = 30;
                public static final int rightAzimuthSmallEncoderID = 31;
                public static final int rightFlywheelID = 32;
                public static final int rightHoodID = 33;
                public static final int rightHoodEncoderID = 34;
                public static final int rightKickupID = 35;
                public static final String rightName = "RightTurret";
                public static final Transform2d robotToRightTurretHoleCenter = new Transform2d();

        }
}
