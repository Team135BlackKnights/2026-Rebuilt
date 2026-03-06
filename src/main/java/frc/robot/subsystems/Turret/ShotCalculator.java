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
      return timeOfFlightMap.get(distanceMeters);
    }
  }

  public static final ShotProfile HUB_PROFILE = new ShotProfile("Hub")
      // Close range — hood stays fairly flat, RPM moderate
      .putHoodAngle(1.567, Rotation2d.fromRadians(0.32))
      .putFlywheelSpeed(1.567, Units.rotationsPerMinuteToRadiansPerSecond(3000))
      .putTimeOfFlight(1.567, 0.86)

      .putHoodAngle(1.669, Rotation2d.fromRadians(0.34))
      .putFlywheelSpeed(1.669, Units.rotationsPerMinuteToRadiansPerSecond(3100))
      .putTimeOfFlight(1.669, 0.87)

      .putHoodAngle(1.769, Rotation2d.fromRadians(0.35))
      .putFlywheelSpeed(1.769, Units.rotationsPerMinuteToRadiansPerSecond(3150))
      .putTimeOfFlight(1.769, 0.87)

      .putHoodAngle(1.885, Rotation2d.fromRadians(0.36))
      .putFlywheelSpeed(1.885, Units.rotationsPerMinuteToRadiansPerSecond(3200))
      .putTimeOfFlight(1.885, 0.87)

      // Mid-close — hood starts climbing, RPM stays moderate (lob transition)
      .putHoodAngle(2.065, Rotation2d.fromRadians(0.38))
      .putFlywheelSpeed(2.065, Units.rotationsPerMinuteToRadiansPerSecond(3250))
      .putTimeOfFlight(2.065, 0.87)

      .putHoodAngle(2.215, Rotation2d.fromRadians(0.40))
      .putFlywheelSpeed(2.215, Units.rotationsPerMinuteToRadiansPerSecond(3300))
      .putTimeOfFlight(2.215, 0.87)

      .putHoodAngle(2.373, Rotation2d.fromRadians(0.43))
      .putFlywheelSpeed(2.373, Units.rotationsPerMinuteToRadiansPerSecond(3350))
      .putTimeOfFlight(2.373, 0.87)

      .putHoodAngle(2.542, Rotation2d.fromRadians(0.47))
      .putFlywheelSpeed(2.542, Units.rotationsPerMinuteToRadiansPerSecond(3400))
      .putTimeOfFlight(2.542, 0.87)

      // REAL DATA
      .putHoodAngle(2.663, Rotation2d.fromRadians(0.52))
      .putFlywheelSpeed(2.663, Units.rotationsPerMinuteToRadiansPerSecond(3500))
      .putTimeOfFlight(2.663, 0.87)
      // interpolated bridge between 2.663 and 3.246
            .putHoodAngle(2.95, Rotation2d.fromRadians(0.57))
      .putFlywheelSpeed(2.95, Units.rotationsPerMinuteToRadiansPerSecond(3600))
      .putTimeOfFlight(2.95, 0.87)
      // REAL DATA
            .putHoodAngle(3.246, Rotation2d.fromRadians(0.63))
      .putFlywheelSpeed(3.246, Units.rotationsPerMinuteToRadiansPerSecond(3700))
      .putTimeOfFlight(3.246, 0.87)
      // interpolated bridge between 3.246 and 3.592
            .putHoodAngle(3.4, Rotation2d.fromRadians(0.63))
      .putFlywheelSpeed(3.4, Units.rotationsPerMinuteToRadiansPerSecond(3850))
      .putTimeOfFlight(3.4, 0.93)
      // REAL DATA
            .putHoodAngle(3.592, Rotation2d.fromRadians(0.63))
      .putFlywheelSpeed(3.592, Units.rotationsPerMinuteToRadiansPerSecond(4000))
      .putTimeOfFlight(3.592, 1.00)
      // interpolated bridge between 3.592 and 3.991
            .putHoodAngle(3.8, Rotation2d.fromRadians(0.67))
      .putFlywheelSpeed(3.8, Units.rotationsPerMinuteToRadiansPerSecond(4150))
      .putTimeOfFlight(3.8, 1.05)
      // REAL DATA
            .putHoodAngle(3.991, Rotation2d.fromRadians(0.70))
      .putFlywheelSpeed(3.991, Units.rotationsPerMinuteToRadiansPerSecond(4300))
      .putTimeOfFlight(3.991, 1.10)

      
      ;
  // Extended data points — ramping from last real point (3.991m: hood 0.70, 4300rpm, tof 1.10)
  // These WILL need real-robot tuning.
  static {
    HUB_PROFILE
        // 4.5 m
        .putHoodAngle(4.5, Rotation2d.fromRadians(0.73))
        .putFlywheelSpeed(4.5, Units.rotationsPerMinuteToRadiansPerSecond(4550))
        .putTimeOfFlight(4.5, 1.20)
        // 5.0 m
        .putHoodAngle(5.0, Rotation2d.fromRadians(0.76))
        .putFlywheelSpeed(5.0, Units.rotationsPerMinuteToRadiansPerSecond(4800))
        .putTimeOfFlight(5.0, 1.30)
        // 5.5 m
        .putHoodAngle(5.5, Rotation2d.fromRadians(0.78))
        .putFlywheelSpeed(5.5, Units.rotationsPerMinuteToRadiansPerSecond(5000))
        .putTimeOfFlight(5.5, 1.40)
        // 6.0 m
        .putHoodAngle(6.0, Rotation2d.fromRadians(0.80))
        .putFlywheelSpeed(6.0, Units.rotationsPerMinuteToRadiansPerSecond(5200))
        .putTimeOfFlight(6.0, 1.50)
        // 6.5 m
        .putHoodAngle(6.5, Rotation2d.fromRadians(0.82))
        .putFlywheelSpeed(6.5, Units.rotationsPerMinuteToRadiansPerSecond(5350))
        .putTimeOfFlight(6.5, 1.60)
        // 7.0 m — near max practical range (hood ~47 deg, still under 50 deg cap)
        .putHoodAngle(7.0, Rotation2d.fromRadians(0.84))
        .putFlywheelSpeed(7.0, Units.rotationsPerMinuteToRadiansPerSecond(5500))
        .putTimeOfFlight(7.0, 1.70);
  }
  public static final ShotProfile TRENCH_PROFILE = new ShotProfile("Trench")
      .putHoodAngle(2.00, Rotation2d.fromDegrees(0.0))
      .putFlywheelSpeed(2.00, 140.0)
      .putTimeOfFlight(2.00, 0.75);
  public static final ShotProfile NEUTRAL_ZONE_PROFILE = new ShotProfile("NeutralZone")
      .putHoodAngle(3.50, Rotation2d.fromDegrees(0.0))
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
