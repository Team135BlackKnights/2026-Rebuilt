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
import edu.wpi.first.math.util.Units;
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
      return timeOfFlightMap.get(distanceMeters)/10;
    }
  }

  public static final ShotProfile HUB_PROFILE = new ShotProfile("Hub")
      // Close range — extrapolated from 1.731m real shot
      .putHoodAngle(1.567, Rotation2d.fromRadians(0.33))
      .putFlywheelSpeed(1.567, Units.rotationsPerMinuteToRadiansPerSecond(3500))
      .putTimeOfFlight(1.567, 1.01)

      .putHoodAngle(1.65, Rotation2d.fromRadians(0.35))
      .putFlywheelSpeed(1.65, Units.rotationsPerMinuteToRadiansPerSecond(3600))
      .putTimeOfFlight(1.65, 1.03)

      // REAL DATA
      .putHoodAngle(1.731, Rotation2d.fromRadians(0.35))
      .putFlywheelSpeed(1.731, Units.rotationsPerMinuteToRadiansPerSecond(3700))
      .putTimeOfFlight(1.731, 1.05)

      // interpolated between 1.731m and 2.244m
      .putHoodAngle(1.85, Rotation2d.fromRadians(0.37))
      .putFlywheelSpeed(1.85, Units.rotationsPerMinuteToRadiansPerSecond(3770))
      .putTimeOfFlight(1.85, 1.05)

      .putHoodAngle(2.0, Rotation2d.fromRadians(0.41))
      .putFlywheelSpeed(2.0, Units.rotationsPerMinuteToRadiansPerSecond(3870))
      .putTimeOfFlight(2.0, 1.06)

      // REAL DATA
      .putHoodAngle(2.244, Rotation2d.fromRadians(0.47))
      .putFlywheelSpeed(2.244, Units.rotationsPerMinuteToRadiansPerSecond(4000))
      .putTimeOfFlight(2.244, 1.08)

      // interpolated from 2.244m toward 3m
      .putHoodAngle(2.45, Rotation2d.fromRadians(0.50))
      .putFlywheelSpeed(2.45, Units.rotationsPerMinuteToRadiansPerSecond(4100))
      .putTimeOfFlight(2.45, 1.10)

      .putHoodAngle(2.663, Rotation2d.fromRadians(0.52))
      .putFlywheelSpeed(2.663, Units.rotationsPerMinuteToRadiansPerSecond(4200))
      .putTimeOfFlight(2.663, 1.12)

      .putHoodAngle(2.85, Rotation2d.fromRadians(0.54))
      .putFlywheelSpeed(2.85, Units.rotationsPerMinuteToRadiansPerSecond(4300))
      .putTimeOfFlight(2.85, 1.15)

      // adjusted — was undershooting with too much hood / too little RPM
      .putHoodAngle(3.0, Rotation2d.fromRadians(0.56))
      .putFlywheelSpeed(3.0, Units.rotationsPerMinuteToRadiansPerSecond(4400))
      .putTimeOfFlight(3.0, 1.18)

      // REAL DATA
      .putHoodAngle(3.246, Rotation2d.fromRadians(0.59))
      .putFlywheelSpeed(3.246, Units.rotationsPerMinuteToRadiansPerSecond(4500))
      .putTimeOfFlight(3.246, 1.20)

      // interpolated bridge between 3.246 and 3.592
      .putHoodAngle(3.4, Rotation2d.fromRadians(0.60))
      .putFlywheelSpeed(3.4, Units.rotationsPerMinuteToRadiansPerSecond(4600))
      .putTimeOfFlight(3.4, 1.23)

      // REAL DATA
      .putHoodAngle(3.592, Rotation2d.fromRadians(0.62))
      .putFlywheelSpeed(3.592, Units.rotationsPerMinuteToRadiansPerSecond(4700))
      .putTimeOfFlight(3.592, 1.26)

      // interpolated bridge between 3.592 and 3.991
      .putHoodAngle(3.8, Rotation2d.fromRadians(0.64))
      .putFlywheelSpeed(3.8, Units.rotationsPerMinuteToRadiansPerSecond(4850))
      .putTimeOfFlight(3.8, 1.30)

      // REAL DATA
      .putHoodAngle(3.991, Rotation2d.fromRadians(0.66))
      .putFlywheelSpeed(3.991, Units.rotationsPerMinuteToRadiansPerSecond(5000))
      .putTimeOfFlight(3.991, 1.34)

      ;
  // Extended data points — ramping from last real point (3.991m: hood 0.66, 5000rpm, tof 1.34)
  // These ARE NOT REAL DATA POINTS.
  static {
    HUB_PROFILE
        // 4.5 m
        .putHoodAngle(4.5, Rotation2d.fromRadians(0.69))
        .putFlywheelSpeed(4.5, Units.rotationsPerMinuteToRadiansPerSecond(5200))
        .putTimeOfFlight(4.5, 1.42)
        // 5.0 m
        .putHoodAngle(5.0, Rotation2d.fromRadians(0.72))
        .putFlywheelSpeed(5.0, Units.rotationsPerMinuteToRadiansPerSecond(5400))
        .putTimeOfFlight(5.0, 1.50)
        // 5.5 m
        .putHoodAngle(5.5, Rotation2d.fromRadians(0.74))
        .putFlywheelSpeed(5.5, Units.rotationsPerMinuteToRadiansPerSecond(5550))
        .putTimeOfFlight(5.5, 1.58)
        // 6.0 m
        .putHoodAngle(6.0, Rotation2d.fromRadians(0.76))
        .putFlywheelSpeed(6.0, Units.rotationsPerMinuteToRadiansPerSecond(5700))
        .putTimeOfFlight(6.0, 1.66)
        // 6.5 m
        .putHoodAngle(6.5, Rotation2d.fromRadians(0.78))
        .putFlywheelSpeed(6.5, Units.rotationsPerMinuteToRadiansPerSecond(5850))
        .putTimeOfFlight(6.5, 1.74)
        // 7.0 m — near max practical range (hood ~45 deg, still under 50 deg cap)
        .putHoodAngle(7.0, Rotation2d.fromRadians(0.80))
        .putFlywheelSpeed(7.0, Units.rotationsPerMinuteToRadiansPerSecond(6000))
        .putTimeOfFlight(7.0, 1.82);
  }
  public static final ShotProfile TRENCH_PROFILE = new ShotProfile("Trench")
      .putHoodAngle(2.00, Rotation2d.fromDegrees(35.0))
      .putFlywheelSpeed(2.00, 140.0)
      .putTimeOfFlight(2.00, 0.75);
  public static final ShotProfile NEUTRAL_ZONE_PROFILE = new ShotProfile("NeutralZone")
      .putHoodAngle(3.50, Rotation2d.fromDegrees(40))
      .putFlywheelSpeed(3.50, Units.rotationsPerMinuteToRadiansPerSecond(5000))
      .putTimeOfFlight(3.50, .2);

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
    return getParameters(target, robotToTurret, HUB_PROFILE, 0.0);
  }

  public ShootingParameters getParameters(Translation2d target, Transform2d robotToTurret, ShotProfile profile, double distanceOffset) {
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
    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation()) + distanceOffset;

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

    double lookaheadDist = target.getDistance(lookaheadPose.getTranslation()) + distanceOffset;

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
