package frc.robot.subsystems.Turret;

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

import org.littletonrobotics.junction.Logger;

/**
 * Calculates turret + hood + flywheel setpoints for a given field target and robot->turret transform.
 *
 * This expects the target Translation2d is ALREADY alliance-corrected (use GeomUtil.apply())!!!
 */
public class ShotCalculator {
  private static ShotCalculator instance;

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

  /** One "shot style" = one set of maps. */
  public static class ShotProfile {
    public final String name;

    private final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public ShotProfile(String name) {
      this.name = name;
    }

    /** Dist (m) -> hood angle (Rotation2d). hood angle should go from the 12 deg to 50 deg (in rads)*/
    public ShotProfile putHoodAngle(double distanceMeters, Rotation2d hoodAngle) {
      hoodAngleMap.put(distanceMeters, hoodAngle);
      return this;
    }

    /** Dist (m) -> flywheel speed (rad/s). */
    public ShotProfile putFlywheelSpeed(double distanceMeters, double speedRadsPerSec) {
      flywheelSpeedMap.put(distanceMeters, speedRadsPerSec);
      return this;
    }

    /** Dist (m) -> time of flight (s). */
    public ShotProfile putTimeOfFlight(double distanceMeters, double timeSeconds) {
      timeOfFlightMap.put(distanceMeters, timeSeconds);
      return this;
    }

    private Rotation2d getHoodAngle(double distanceMeters) {
      return hoodAngleMap.get(distanceMeters);
    }

    private double getFlywheelSpeed(double distanceMeters) {
      return flywheelSpeedMap.get(distanceMeters);
    }

    private double getTimeOfFlight(double distanceMeters) {
      return timeOfFlightMap.get(distanceMeters);
    }
  }

  public static final ShotProfile HUB_PROFILE = new ShotProfile("Hub")
      // TODO: replace with real calibration points, and not just one point each lol
      .putHoodAngle(1.45, Rotation2d.fromDegrees(12.0))
      .putFlywheelSpeed(1.45, 175.0)
      .putTimeOfFlight(1.45, 0.55);
  public static final ShotProfile TRENCH_PROFILE = new ShotProfile("Trench")
      .putHoodAngle(2.00, Rotation2d.fromDegrees(18.0))
      .putFlywheelSpeed(2.00, 140.0)
      .putTimeOfFlight(2.00, 0.75);
  public static final ShotProfile NEUTRAL_ZONE_PROFILE = new ShotProfile("NeutralZone")
      .putHoodAngle(3.50, Rotation2d.fromDegrees(30.0))
      .putFlywheelSpeed(3.50, 120.0)
      .putTimeOfFlight(3.50, 1.10);

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / .02));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / .02));

  private Rotation2d lastTurretAngle = null;
  private double lastHoodAngle = Double.NaN;

  private ShootingParameters latestParameters = null;
  private Translation2d latestTarget = null;
  private Transform2d latestRobotToTurret = null;
  private ShotProfile latestProfile = null;

  public ShootingParameters getParameters(Translation2d target, Transform2d robotToTurret) {
    return getParameters(target, robotToTurret, HUB_PROFILE);
  }

  public ShootingParameters getParameters(Translation2d target, Transform2d robotToTurret, ShotProfile profile) {
    if (profile == null) profile = HUB_PROFILE;

    if (latestParameters != null
        && latestProfile == profile
        && approxEqual(latestTarget, target, 1e-6)
        && approxEqual(latestRobotToTurret, robotToTurret, 1e-6)) {
      return latestParameters;
    }

    latestTarget = target;
    latestRobotToTurret = robotToTurret;
    latestProfile = profile;

    // Calculate distance from turret to target
    Pose2d turretPosition = RobotContainer.drivetrainS.getLookAheadPose().transformBy(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity (same math you had)
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

    // Lookahead using time of flight from the chosen profile
    double timeOfFlight = profile.getTimeOfFlight(turretToTargetDistance);
    double offsetX = turretVelocityX * timeOfFlight;
    double offsetY = turretVelocityY * timeOfFlight;

    Pose2d lookaheadPose =
        new Pose2d(
            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
            turretPosition.getRotation());

    double lookaheadDist = target.getDistance(lookaheadPose.getTranslation());

    Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle().minus(new Rotation2d(robotAngle));
    double hoodAngle = profile.getHoodAngle(lookaheadDist).getRadians();
    double flywheelSpeed = profile.getFlywheelSpeed(lookaheadDist);

    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    double turretVel =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / .02);
    double hoodVel =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / .02);

    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new ShootingParameters(
            turretAngle,
            turretVel,
            hoodAngle,
            hoodVel,
            flywheelSpeed);

    Logger.recordOutput("SuperStructure/ShotCalculator/"+profile.name+"/LookaheadPose", lookaheadPose);
    Logger.recordOutput("SuperStructure/ShotCalculator/"+profile.name+"/LookaheadDist", lookaheadDist);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
    latestTarget = null;
    latestRobotToTurret = null;
    latestProfile = null;
  }

  private static boolean approxEqual(Translation2d a, Translation2d b, double eps) {
    if (a == null || b == null) return false;
    return Math.abs(a.getX() - b.getX()) < eps && Math.abs(a.getY() - b.getY()) < eps;
  }

  private static boolean approxEqual(Transform2d a, Transform2d b, double eps) {
    if (a == null || b == null) return false;
    return Math.abs(a.getX() - b.getX()) < eps
        && Math.abs(a.getY() - b.getY()) < eps
        && Math.abs(a.getRotation().getRadians() - b.getRotation().getRadians()) < eps;
  }
}
