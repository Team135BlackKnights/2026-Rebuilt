package frc.robot.utils.advancedMechs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

public class AdvancedMechanismConstants {
        public static class Turret {
                public static final double leftMinTurretAngle = 0;
                public static final double leftMaxTurretAngle = 5.4;
                public static final double rightMinTurretAngle = -5.4;
                public static final double rightMaxTurretAngle = 0;
                public static final double leftMinHoodAngle = Math.toRadians(12.0);
                public static final double leftMaxHoodAngle = Math.toRadians(43.0);
                public static final double rightMinHoodAngle = Math.toRadians(12.0);
                public static final double rightMaxHoodAngle = Math.toRadians(43.0);
                public static final int currentLimitAzimuth = 50;
                public static final int currentLimitFlywheel = 80;
                public static final int currentLimitHood = 20;
                public static final int currentLimitKickup = 60;
                public static final int turretTeeth = 77;
                public static final int idlerTeeth = 10;
                public static final double leftMotorRadPerTurretRad = 36.0 / 12.0 * 77.0 / 10.0;
                public static final double rightMotorRadPerTurretRad = 36.0 / 10.0 * 77.0 / 10.0;
                public static final double flywheelRatio = 1.0 / 1.0;
                public static final double hoodMotorToHoodEncoderRatio = 50.0 / 12.0;
                public static final double hoodEncoderToHoodArmRatio = 19.0 / 15.0;
                public static final double centerIndexerRatio = 5.0 / 1.0;
                public static final double turretKickupRatio = 2.0 / 1.0;
                public static final double enc1GearTeethRight = 36;
                public static final double enc1GearTeethLeft = 34;
                public static final double enc2GearTeethRight = 34;
                public static final double enc2GearTeethLeft = 36;
                public static final DCMotor azimuthMotor = DCMotor.getKrakenX44Foc(1);
                public static final DCMotor flywheelMotor = DCMotor.getKrakenX44Foc(1);
                public static final DCMotor hoodMotor = DCMotor.getMinion(1);
                public static final DCMotor centerIndexerMotor = DCMotor.getNEO(1);
                public static final DCMotor turretKickupMotor = DCMotor.getKrakenX44Foc(1);
                public static final double azimuthMOI = 0.0424; // kg m^2
                public static final double flywheelMOI = 0.0048; // kg m^2
                public static final double hoodMOI = 0.0101; // kg m^2
                public static final double centerIndexerMOI = 0.0005; // kg m^2
                public static final double turretKickupMOI = 0.0005; // kg m^2
                public static final double flywheelMaxRPM = 5700;
                public static final boolean invertKickup = false;
                public static final boolean invertFlywheel = true;
                public static final boolean invertHood = true;
                public static final boolean invertAzimuth = true;
                public static final int centerIndexerID = 28;
                public static final int leftTurretKickupID = SimpleMechanismConstants.Climber.climberId;
                public static final int rightTurretKickupID = SimpleMechanismConstants.Climber.wedgeArmId;

                // Left Turret
                public static final int leftAzimuthID = 22;
                public static final int leftAzimuthBigEncoderID = 23;
                public static final int leftAzimuthSmallEncoderID = 24;
                public static final int leftFlywheelID = 25;
                public static final int leftHoodID = 26;
                public static final int leftHoodEncoderID = 27;
                public static final String leftName = "LeftTurret";
                public static final Transform2d robotToLeftTurretHoleCenter = new Transform2d(-.163, -.227, new Rotation2d());
                public static final Transform3d robotToLeftTurretLaunchBase =
                                new Transform3d(robotToLeftTurretHoleCenter.getX(), robotToLeftTurretHoleCenter.getY(),
                                                Units.inchesToMeters(15.0), new Rotation3d());
                public static final double leftAzimuthBigEncoderOffset = .2841796875;
                public static final double leftAzimuthSmallEncoderOffset = .201904296875;
                public static final double leftHoodEncoderOffset = 0.2294921875;
                public static final double leftLaunchPitchOffsetRads = Math.PI / 2.0;
                public static final double leftLaunchPitchScale = -1.0;
                public static final double leftLaunchPathLengthMeters = 0.20746;
                // Right Turret
                public static final int rightAzimuthID = 29;
                public static final int rightAzimuthBigEncoderID = 30;
                public static final int rightAzimuthSmallEncoderID = 31;
                public static final int rightFlywheelID = 32;
                public static final int rightHoodID = 33;
                public static final int rightHoodEncoderID = 34;
                public static final String rightName = "RightTurret";
                public static final Transform2d robotToRightTurretHoleCenter = new Transform2d(-.163, .227, new Rotation2d());
                public static final Transform3d robotToRightTurretLaunchBase =
                                new Transform3d(robotToRightTurretHoleCenter.getX(), robotToRightTurretHoleCenter.getY(),
                                                Units.inchesToMeters(15.0), new Rotation3d());
                public static final double rightAzimuthBigEncoderOffset = .38916;
                public static final double rightAzimuthSmallEncoderOffset = .084716;
                public static final double rightHoodEncoderOffset = 0.385986328125;
                public static final double rightLaunchPitchOffsetRads = Math.PI / 2.0;
                public static final double rightLaunchPitchScale = -1.0;
                public static final double rightLaunchPathLengthMeters = 0.20746;
                public static final double hubScoringPlaneHeightMeters = Units.inchesToMeters(72.0);
                public static final double hubScoringPlaneLateralToleranceMeters = Units.inchesToMeters(18.0);
        }
}
