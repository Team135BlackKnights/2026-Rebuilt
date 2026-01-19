package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.utils.GeomUtil;

import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static ShotCalculator instance;
  private static Transform2d robotToTurret = new Transform2d();

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / .02));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / .02));

  private Rotation2d lastTurretAngle;
  private double lastHoodAngle;
  private Rotation2d turretAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    shotHoodAngleMap.put(1.45, Rotation2d.fromDegrees(12.0)); //Dist, angle FROM VERTICAL in degrees. (12 = 78 from horizontal)
    shotFlywheelSpeedMap.put(1.45, 175.0); //Dist, speed in Rad/s
    timeOfFlightMap.put(67.0, 2.1); // Dist, time in air.
  }

  public ShootingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate distance from turret to target
    Translation2d target = GeomUtil.apply(new Translation2d()); //TODO find hub center
    Pose2d turretPosition = RobotContainer.drivetrainS.getLookAheadPose().transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = RobotContainer.drivetrainS.getFieldChassisSpeeds();
    double robotAngle = RobotContainer.drivetrainS.getRotation2d().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getY() * Math.cos(robotAngle)
                    - robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (robotToTurret.getX() * Math.cos(robotAngle)
                    - robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight = timeOfFlightMap.get(turretToTargetDistance);
    double offsetX = turretVelocityX * timeOfFlight;
    double offsetY = turretVelocityY * timeOfFlight;
    Pose2d lookaheadPose =
        new Pose2d(
            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
            turretPosition.getRotation());
    double lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() /.02);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / .02);
    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new ShootingParameters(
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}