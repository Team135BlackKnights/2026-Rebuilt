package frc.robot.utils.advancedMechs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class AdvancedMechanismConstants {
        public static class Turret {
                public static final double minTurretAngle = -1.3;
                public static final double maxTurretAngle = 4.2;
                public static final double minHoodAngle = Math.toRadians(12.0);
                public static final double maxHoodAngle = Math.toRadians(50.0);
                public static final int currentLimitAzimuth = 30;
                public static final int currentLimitFlywheel = 40;
                public static final int currentLimitHood = 20;
                public static final int currentLimitKickup = 50;
                public static final int turretTeeth = 77;
                public static final int idlerTeeth = 10;
                public static final double motorRadPerTurretRad = 35.0 / 10.0 * 77.0 / 10.0;// motor to turret ratio
                public static final double flywheelRatio = 1.0 / 1.0;
                public static final double hoodMotorToHoodEncoderRatio = 50.0 / 12.0;
                public static final double hoodEncoderToHoodArmRatio = 19.0 / 15.0;
                public static final double kickerRatio = 5.0 / 1.0;
                public static final double enc1GearTeethRight = 36;
                public static final double enc1GearTeethLeft = 34;
                public static final double enc2GearTeethRight = 34;
                public static final double enc2GearTeethLeft = 36;
                public static final DCMotor azimuthMotor = DCMotor.getKrakenX44Foc(1);
                public static final DCMotor flywheelMotor = DCMotor.getKrakenX44Foc(1);
                public static final DCMotor hoodMotor = DCMotor.getMinion(1);
                public static final DCMotor kickupMotor = DCMotor.getNEO(1);
                public static final double azimuthMOI = 0.0424; // kg m^2
                public static final double flywheelMOI = 0.0048; // kg m^2
                public static final double hoodMOI = 0.0101; // kg m^2
                public static final double kickupMOI = 0.0005; // kg m^2
                public static final double flywheelMaxRPM = 5700;
                public static final boolean invertKickup = false;
                public static final boolean invertFlywheel = false;
                public static final boolean invertHood = false;
                public static final boolean invertAzimuth = true;

                // Left Turret
                public static final int leftAzimuthID = 22;
                public static final int leftAzimuthBigEncoderID = 23;
                public static final int leftAzimuthSmallEncoderID = 24;
                public static final int leftFlywheelID = 25;
                public static final int leftHoodID = 26;
                public static final int leftHoodEncoderID = 27;
                public static final int leftKickupID = 28;
                public static final String leftName = "LeftTurret";
                public static final Transform2d robotToLeftTurretHoleCenter = new Transform2d(-.163, -.227, new Rotation2d());
                public static final double leftAzimuthBigEncoderOffset = .2841796875;
                public static final double leftAzimuthSmallEncoderOffset = .201904296875;
                public static final double leftHoodEncoderOffset = 0.0;
                // Right Turret
                public static final int rightAzimuthID = 29;
                public static final int rightAzimuthBigEncoderID = 30;
                public static final int rightAzimuthSmallEncoderID = 31;
                public static final int rightFlywheelID = 32;
                public static final int rightHoodID = 33;
                public static final int rightHoodEncoderID = 34;
                public static final int rightKickupID = 35;
                public static final String rightName = "RightTurret";
                public static final Transform2d robotToRightTurretHoleCenter = new Transform2d(-.163, .227, new Rotation2d());
                public static final double rightAzimuthBigEncoderOffset = -.252197265625;
                public static final double rightAzimuthSmallEncoderOffset = -.1323241875;
                public static final double rightHoodEncoderOffset = 0.0;
        }
}
