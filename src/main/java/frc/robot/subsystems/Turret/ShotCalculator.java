package frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TuningConstants;
import frc.robot.RobotContainer;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedBoolean;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

/**
 * Calculates turret + hood + flywheel setpoints for a given field target and robot->turret transform.
 *
 * This expects the target Translation2d is ALREADY alliance-corrected (use GeomUtil.apply()).
 */
public class ShotCalculator {
  private static final double GRAVITY_METERS_PER_SEC2 = FieldConstants.COEFFICIENT_OF_GRAVITY;
  private static final double HUB_TARGET_MATCH_EPSILON_METERS = 1e-3;
  private static final int HYBRID_HEADING_SOLVE_ITERATIONS = 5;
  private static final int TABLE_2D_BALLISTIC_TOF_ITERATIONS = 3;
  private static final double MAX_FLYWHEEL_SPEED_RAD_PER_SEC =
      Units.rotationsPerMinuteToRadiansPerSecond(AdvancedMechanismConstants.Turret.flywheelMaxRPM);
  private static final Translation2d INVALID_TRANSLATION =
      new Translation2d(Double.NaN, Double.NaN);
  private static final Pose3d INVALID_POSE =
      new Pose3d(Double.NaN, Double.NaN, Double.NaN, new Rotation3d());

  private record TrustedFitPoint(double distanceMeters, double rpm, double hoodRadians) {}

  private static final TrustedFitPoint[] TRUSTED_FIT_POINTS = {
      new TrustedFitPoint(1.691, 3000.0, 0.31),
      new TrustedFitPoint(1.956, 3000.0, 0.37),
      new TrustedFitPoint(2.170, 3200.0, 0.41),
      new TrustedFitPoint(2.530, 3500.0, 0.45),
      new TrustedFitPoint(2.761, 3600.0, 0.50)
  };

  private static final LoggableTunedBoolean useHybrid3DLead =
      new LoggableTunedBoolean(
          "ShotCalculator/UseHybrid3DLead", true, TuningConstants.isTuningShooter);

  private static final double FITTED_BALL_SPEED_MPS_PER_RPM = fitBaseBallSpeedMetersPerSecPerRPM();

  private static final LoggableTunedNumber ballisticBallSpeedMetersPerSecPerRPM =
      new LoggableTunedNumber(
          "ShotCalculator/BallSpeedMpsPerRPM",
          FITTED_BALL_SPEED_MPS_PER_RPM,
          TuningConstants.isTuningShooter);
  private static final LoggableTunedNumber motionCompensationDeadbandSpeedMetersPerSec =
      new LoggableTunedNumber(
          "ShotCalculator/MotionCompDeadbandSpeedMps", 0.5, TuningConstants.isTuningShooter);
  private static final LoggableTunedNumber motionCompensationFullSpeedMetersPerSec =
      new LoggableTunedNumber(
          "ShotCalculator/MotionCompFullSpeedMps", 1.0, TuningConstants.isTuningShooter);

  private static final TurretBallisticsConfig LEFT_TURRET_CONFIG =
      new TurretBallisticsConfig(
          AdvancedMechanismConstants.Turret.leftName,
          AdvancedMechanismConstants.Turret.robotToLeftTurretHoleCenter,
          AdvancedMechanismConstants.Turret.robotToLeftTurretLaunchBase,
          AdvancedMechanismConstants.Turret.leftMinHoodAngle,
          AdvancedMechanismConstants.Turret.leftMaxHoodAngle,
          new LoggableTunedNumber(
              "ShotCalculator/LeftTurret/PitchOffsetRad",
              AdvancedMechanismConstants.Turret.leftLaunchPitchOffsetRads,
              TuningConstants.isTuningShooter),
          new LoggableTunedNumber(
              "ShotCalculator/LeftTurret/PitchScale",
              AdvancedMechanismConstants.Turret.leftLaunchPitchScale,
              TuningConstants.isTuningShooter),
          AdvancedMechanismConstants.Turret.leftLaunchPathLengthMeters);

  private static final TurretBallisticsConfig RIGHT_TURRET_CONFIG =
      new TurretBallisticsConfig(
          AdvancedMechanismConstants.Turret.rightName,
          AdvancedMechanismConstants.Turret.robotToRightTurretHoleCenter,
          AdvancedMechanismConstants.Turret.robotToRightTurretLaunchBase,
          AdvancedMechanismConstants.Turret.rightMinHoodAngle,
          AdvancedMechanismConstants.Turret.rightMaxHoodAngle,
          new LoggableTunedNumber(
              "ShotCalculator/RightTurret/PitchOffsetRad",
              AdvancedMechanismConstants.Turret.rightLaunchPitchOffsetRads,
              TuningConstants.isTuningShooter),
          new LoggableTunedNumber(
              "ShotCalculator/RightTurret/PitchScale",
              AdvancedMechanismConstants.Turret.rightLaunchPitchScale,
              TuningConstants.isTuningShooter),
          AdvancedMechanismConstants.Turret.rightLaunchPathLengthMeters);

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

  private enum ShotModel {
    TABLE_2D,
    HYBRID_3D_LEAD
  }

  private record TargetPlaneGeometry(
      String name,
      Translation2d center,
      double heightMeters,
      double lateralToleranceMeters) {}

  private record BallisticState(
      boolean valid,
      Translation2d launchBasePosition,
      Translation2d correctedTargetPoint,
      Pose3d launchPose,
      double launchPitchRad,
      double launchSpeedMps,
      double timeOfFlightSec,
      Translation2d rawChassisVelocityAtMuzzle,
      double motionCompensationScale,
      Translation2d chassisVelocityAtMuzzle,
      Translation2d predictedCrossingPoint,
      double rangeErrorMeters,
      double lateralErrorMeters,
      boolean withinLateralTolerance) {
    private static BallisticState invalid() {
      return new BallisticState(
          false,
          INVALID_TRANSLATION,
          INVALID_TRANSLATION,
          INVALID_POSE,
          Double.NaN,
          Double.NaN,
          Double.NaN,
          INVALID_TRANSLATION,
          Double.NaN,
          INVALID_TRANSLATION,
          INVALID_TRANSLATION,
          Double.NaN,
          Double.NaN,
          false);
    }
  }

  private record BallisticCandidate(
      double timeOfFlightSec,
      Translation2d correctedTargetPoint,
      Translation2d predictedCrossingPoint,
      double rangeErrorMeters,
      double lateralErrorMeters,
      boolean withinLateralTolerance,
      double planarErrorMeters) {}

  private record StaticShotCommand(double hoodAngleRad, double flywheelSpeedRadPerSec) {}

  private record ShotSolution(
      ShotModel model,
      boolean valid,
      Rotation2d turretAngle,
      double hoodAngle,
      double flywheelSpeed,
      double leadTimeOfFlightSec,
      Pose2d lookaheadPose,
      double lookaheadDist,
      BallisticState ballisticState) {}

  private static class TurretBallisticsConfig {
    private final String name;
    private final Transform2d turretCenterTransform;
    private final Transform3d launchBaseTransform;
    private final double minHoodAngleRad;
    private final double maxHoodAngleRad;
    private final LoggableTunedNumber pitchOffsetRad;
    private final LoggableTunedNumber pitchScale;
    private final double launchPathLengthMeters;

    private TurretBallisticsConfig(
        String name,
        Transform2d turretCenterTransform,
        Transform3d launchBaseTransform,
        double minHoodAngleRad,
        double maxHoodAngleRad,
        LoggableTunedNumber pitchOffsetRad,
        LoggableTunedNumber pitchScale,
        double launchPathLengthMeters) {
      this.name = name;
      this.turretCenterTransform = turretCenterTransform;
      this.launchBaseTransform = launchBaseTransform;
      this.minHoodAngleRad = minHoodAngleRad;
      this.maxHoodAngleRad = maxHoodAngleRad;
      this.pitchOffsetRad = pitchOffsetRad;
      this.pitchScale = pitchScale;
      this.launchPathLengthMeters = launchPathLengthMeters;
    }

    private Translation2d launchBaseOffsetRobot() {
      return new Translation2d(launchBaseTransform.getX(), launchBaseTransform.getY());
    }

    private double launchBaseHeightMeters() {
      return launchBaseTransform.getZ();
    }

    private double launchPitchRad(double hoodAngleRad) {
      double pitch = pitchOffsetRad.get() + (pitchScale.get() * hoodAngleRad);
      return MathUtil.clamp(pitch, Units.degreesToRadians(1.0), Units.degreesToRadians(89.0));
    }
  }

  private static class ShotCurve {
    private final List<Double> xValues = new ArrayList<>();
    private final List<Double> yValues = new ArrayList<>();

    public void put(double x, double y) {
      int insertIndex = 0;
      while (insertIndex < xValues.size() && xValues.get(insertIndex) < x) {
        insertIndex++;
      }

      if (insertIndex < xValues.size() && Math.abs(xValues.get(insertIndex) - x) < 1e-9) {
        yValues.set(insertIndex, y);
        return;
      }

      xValues.add(insertIndex, x);
      yValues.add(insertIndex, y);
    }

    public double get(double x) {
      int size = xValues.size();
      if (size == 0) {
        return 0.0;
      }
      if (size == 1) {
        return yValues.get(0);
      }

      double[] tangents = computeTangents();
      if (x <= xValues.get(0)) {
        return yValues.get(0) + tangents[0] * (x - xValues.get(0));
      }
      if (x >= xValues.get(size - 1)) {
        return yValues.get(size - 1) + tangents[size - 1] * (x - xValues.get(size - 1));
      }

      int interval = 0;
      while (interval < size - 2 && x > xValues.get(interval + 1)) {
        interval++;
      }

      double x0 = xValues.get(interval);
      double x1 = xValues.get(interval + 1);
      double y0 = yValues.get(interval);
      double y1 = yValues.get(interval + 1);
      double h = x1 - x0;
      double t = (x - x0) / h;
      double t2 = t * t;
      double t3 = t2 * t;

      double h00 = (2.0 * t3) - (3.0 * t2) + 1.0;
      double h10 = t3 - (2.0 * t2) + t;
      double h01 = (-2.0 * t3) + (3.0 * t2);
      double h11 = t3 - t2;

      return h00 * y0
          + h10 * h * tangents[interval]
          + h01 * y1
          + h11 * h * tangents[interval + 1];
    }

    private double[] computeTangents() {
      int size = xValues.size();
      double[] tangents = new double[size];
      if (size == 2) {
        double slope = slope(0);
        tangents[0] = slope;
        tangents[1] = slope;
        return tangents;
      }

      tangents[0] = endpointTangent(0);
      tangents[size - 1] = endpointTangent(size - 1);
      for (int i = 1; i < size - 1; i++) {
        double previousSlope = slope(i - 1);
        double nextSlope = slope(i);
        if (previousSlope == 0.0
            || nextSlope == 0.0
            || Math.signum(previousSlope) != Math.signum(nextSlope)) {
          tangents[i] = 0.0;
          continue;
        }

        double previousStep = xValues.get(i) - xValues.get(i - 1);
        double nextStep = xValues.get(i + 1) - xValues.get(i);
        double weightA = (2.0 * nextStep) + previousStep;
        double weightB = nextStep + (2.0 * previousStep);
        tangents[i] =
            (weightA + weightB) / ((weightA / previousSlope) + (weightB / nextSlope));
      }
      return tangents;
    }

    private double endpointTangent(int endpointIndex) {
      if (endpointIndex == 0) {
        double h0 = xValues.get(1) - xValues.get(0);
        double h1 = xValues.get(2) - xValues.get(1);
        double delta0 = slope(0);
        double delta1 = slope(1);
        double tangent = (((2.0 * h0) + h1) * delta0 - (h0 * delta1)) / (h0 + h1);
        return clampEndpointTangent(tangent, delta0, delta1);
      }

      int last = xValues.size() - 1;
      double h0 = xValues.get(last - 1) - xValues.get(last - 2);
      double h1 = xValues.get(last) - xValues.get(last - 1);
      double delta0 = slope(last - 2);
      double delta1 = slope(last - 1);
      double tangent = (((2.0 * h1) + h0) * delta1 - (h1 * delta0)) / (h0 + h1);
      return clampEndpointTangent(tangent, delta1, delta0);
    }

    private double clampEndpointTangent(double tangent, double edgeSlope, double adjacentSlope) {
      if (Math.signum(tangent) != Math.signum(edgeSlope)) {
        return 0.0;
      }
      if (Math.signum(edgeSlope) != Math.signum(adjacentSlope)
          && Math.abs(tangent) > Math.abs(3.0 * edgeSlope)) {
        return 3.0 * edgeSlope;
      }
      return tangent;
    }

    private double slope(int intervalIndex) {
      double dx = xValues.get(intervalIndex + 1) - xValues.get(intervalIndex);
      if (Math.abs(dx) < 1e-9) {
        return 0.0;
      }
      return (yValues.get(intervalIndex + 1) - yValues.get(intervalIndex)) / dx;
    }
  }

  /** One "shot style" = one set of curves. */
  public static class ShotProfile {
    public final String name;

    private final ShotCurve hoodAngleCurve = new ShotCurve();
    private final ShotCurve flywheelSpeedCurve = new ShotCurve();
    private final ShotCurve timeOfFlightCurve = new ShotCurve();

    public ShotProfile(String name) {
      this.name = name;
    }

    public ShotProfile putHoodAngle(double distanceMeters, Rotation2d hoodAngle) {
      hoodAngleCurve.put(distanceMeters, hoodAngle.getRadians());
      return this;
    }

    public ShotProfile putFlywheelSpeed(double distanceMeters, double speedRadsPerSec) {
      flywheelSpeedCurve.put(distanceMeters, speedRadsPerSec);
      return this;
    }

    public ShotProfile putTimeOfFlight(double distanceMeters, double timeSeconds) {
      timeOfFlightCurve.put(distanceMeters, timeSeconds);
      return this;
    }

    private Rotation2d getHoodAngle(double distanceMeters) {
      return Rotation2d.fromRadians(hoodAngleCurve.get(distanceMeters));
    }

    private double getFlywheelSpeed(double distanceMeters) {
      return flywheelSpeedCurve.get(distanceMeters);
    }

    private double getTimeOfFlight(double distanceMeters) {
      return timeOfFlightCurve.get(distanceMeters);
    }
  }

  private static final boolean USE_HUB_PROFILE_FOR_TRENCH_AND_NEUTRAL = true;

  private static ShotProfile addCalibrationPoint(
      ShotProfile profile,
      double distanceMeters,
      double rpm,
      double hoodRadians,
      double timeOfFlightSeconds) {
    return profile
        .putHoodAngle(distanceMeters, Rotation2d.fromRadians(hoodRadians))
        .putFlywheelSpeed(distanceMeters, Units.rotationsPerMinuteToRadiansPerSecond(rpm))
        .putTimeOfFlight(distanceMeters, timeOfFlightSeconds);
  }

  private static ShotProfile buildHubProfile() {
    ShotProfile profile = new ShotProfile("Hub");
    addCalibrationPoint(profile, 1.691, 3000.0, 0.31, 0.920);
    addCalibrationPoint(profile, 1.956, 3000.0, 0.37, 0.940);
    addCalibrationPoint(profile, 2.170, 3200.0, 0.41, 0.970);
    addCalibrationPoint(profile, 2.530, 3500.0, 0.45, 1.080);
    addCalibrationPoint(profile, 2.761, 3600.0, 0.50, 1.170);
    addCalibrationPoint(profile, 3.021, 3800.0, 0.53, 1.140);
    addCalibrationPoint(profile, 3.318, 4200.0, 0.62, 1.070);
    addCalibrationPoint(profile, 3.555, 4400.0, 0.70, 1.030);
    return profile;
  }

  private static ShotProfile buildTrenchProfile() {
    return new ShotProfile("Trench")
        .putHoodAngle(2.00, Rotation2d.fromDegrees(35.0))
        .putFlywheelSpeed(2.00, 140.0)
        .putTimeOfFlight(2.00, 0.75);
  }

  private static ShotProfile buildNeutralZoneProfile() {
    return new ShotProfile("NeutralZone")
        .putHoodAngle(3.50, Rotation2d.fromDegrees(40))
        .putFlywheelSpeed(3.50, Units.rotationsPerMinuteToRadiansPerSecond(5000))
        .putTimeOfFlight(3.50, .2);
  }

  public static final ShotProfile HUB_PROFILE = buildHubProfile();
  private static final ShotProfile DEDICATED_TRENCH_PROFILE = buildTrenchProfile();
  private static final ShotProfile DEDICATED_NEUTRAL_ZONE_PROFILE = buildNeutralZoneProfile();
  public static final ShotProfile TRENCH_PROFILE =
      USE_HUB_PROFILE_FOR_TRENCH_AND_NEUTRAL ? HUB_PROFILE : DEDICATED_TRENCH_PROFILE;
  public static final ShotProfile NEUTRAL_ZONE_PROFILE =
      USE_HUB_PROFILE_FOR_TRENCH_AND_NEUTRAL ? HUB_PROFILE : DEDICATED_NEUTRAL_ZONE_PROFILE;

  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

  private Rotation2d lastTurretAngle = null;
  private double lastHoodAngle = Double.NaN;

  public ShootingParameters getParameters(Translation2d target, Transform2d robotToTurret) {
    return getParameters(target, robotToTurret, HUB_PROFILE, 0.0);
  }

  public ShootingParameters getParameters(
      Translation2d target,
      Transform2d robotToTurret,
      ShotProfile profile,
      double distanceOffset) {
    if (profile == null) profile = HUB_PROFILE;

    Pose2d robotPose = RobotContainer.drivetrainS.getPose();
    Rotation2d robotHeading = robotPose.getRotation();
    ChassisSpeeds fieldChassisSpeeds = RobotContainer.drivetrainS.getFieldChassisSpeeds();
    TargetPlaneGeometry targetGeometry = resolveTargetPlaneGeometry(target);
    Translation2d fieldLinearVelocity =
        new Translation2d(fieldChassisSpeeds.vxMetersPerSecond, fieldChassisSpeeds.vyMetersPerSecond);

    ShotSolution table2dSolution =
        createTable2dSolution(
            target,
            robotToTurret,
            profile,
            distanceOffset,
            robotPose,
            robotHeading,
            fieldChassisSpeeds,
            targetGeometry);
    ShotSolution hybridSolution =
        createHybrid3dLeadSolution(
            target, robotToTurret, profile, distanceOffset, robotPose, robotHeading, fieldChassisSpeeds, targetGeometry);

    boolean wantsHybrid = useHybrid3DLead.get();
    ShotSolution selectedSolution =
        wantsHybrid && hybridSolution.valid() ? hybridSolution : table2dSolution;

    Rotation2d turretAngle = selectedSolution.turretAngle();
    double hoodAngle = selectedSolution.hoodAngle();
    double flywheelSpeed = selectedSolution.flywheelSpeed();

    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    double turretVel =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.02);
    double hoodVel = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);

    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngle;

    logSolution(profile.name, "Table2D", table2dSolution, targetGeometry);
    logSolution(profile.name, "Hybrid3DLead", hybridSolution, targetGeometry);

    Logger.recordOutput("SuperStructure/ShotCalculator/" + profile.name + "/UseHybrid3DLeadRequested", wantsHybrid);
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/SelectedModel",
        selectedSolution.model().toString());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/BallSpeedMpsPerRPMFit",
        FITTED_BALL_SPEED_MPS_PER_RPM);
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/BallSpeedMpsPerRPMUsed",
        ballisticBallSpeedMetersPerSecPerRPM.get());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/LookaheadPose",
        selectedSolution.lookaheadPose());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/LookaheadDist",
        selectedSolution.lookaheadDist());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/LeadTimeOfFlightSec",
        selectedSolution.leadTimeOfFlightSec());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/BallisticTimeOfFlightSec",
        selectedSolution.ballisticState().timeOfFlightSec());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/FieldLinearSpeedMps",
        fieldLinearVelocity.getNorm());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/MotionCompDeadbandSpeedMps",
        motionCompensationDeadbandSpeedMetersPerSec.get());
    Logger.recordOutput(
        "SuperStructure/ShotCalculator/" + profile.name + "/MotionCompFullSpeedMps",
        motionCompensationFullSpeedMetersPerSec.get());

    return new ShootingParameters(turretAngle, turretVel, hoodAngle, hoodVel, flywheelSpeed);
  }

  public void clearShootingParameters() {}

  private ShotSolution createTable2dSolution(
      Translation2d target,
      Transform2d robotToTurret,
      ShotProfile profile,
      double distanceOffset,
      Pose2d robotPose,
      Rotation2d robotHeading,
      ChassisSpeeds fieldChassisSpeeds,
      TargetPlaneGeometry targetGeometry) {
    Pose2d turretCenterPose = robotPose.transformBy(robotToTurret);
    TurretBallisticsConfig turretConfig = resolveTurretConfig(robotToTurret);
    double turretToTargetDistance = target.getDistance(turretCenterPose.getTranslation()) + distanceOffset;
    Translation2d rawTurretCenterVelocity =
        computeFieldVelocityAtRobotOffset(robotToTurret.getTranslation(), robotHeading, fieldChassisSpeeds);
    double turretCenterMotionCompScale = computeMotionCompensationScale(rawTurretCenterVelocity);
    Translation2d turretCenterVelocity = rawTurretCenterVelocity.times(turretCenterMotionCompScale);
    double leadTimeOfFlightSec = profile.getTimeOfFlight(turretToTargetDistance);
    double lookaheadDist = turretToTargetDistance;
    Pose2d lookaheadPose = turretCenterPose;
    StaticShotCommand shotCommand =
        getStaticShotCommandForDistance(
            profile, turretToTargetDistance, targetGeometry, turretConfig, robotPose, robotHeading);
    String table2dLogPrefix = "SuperStructure/ShotCalculator/" + profile.name + "/Table2D";
    Logger.recordOutput(table2dLogPrefix + "/MotionCompScale", turretCenterMotionCompScale);
    Logger.recordOutput(table2dLogPrefix + "/RawTurretCenterVelocityMps", rawTurretCenterVelocity.getNorm());
    Logger.recordOutput(table2dLogPrefix + "/ScaledTurretCenterVelocityMps", turretCenterVelocity.getNorm());
    if (targetGeometry == null) {
      lookaheadPose =
          new Pose2d(
              turretCenterPose.getTranslation().plus(turretCenterVelocity.times(leadTimeOfFlightSec)),
              turretCenterPose.getRotation());
      lookaheadDist = target.getDistance(lookaheadPose.getTranslation()) + distanceOffset;
    } else {
      for (int i = 0; i < TABLE_2D_BALLISTIC_TOF_ITERATIONS; i++) {
        Double ballisticLeadTimeSec =
            estimateStationaryBallisticTimeOfFlightSec(
                targetGeometry,
                turretConfig,
                robotPose,
                robotHeading,
                shotCommand.hoodAngleRad(),
                shotCommand.flywheelSpeedRadPerSec());
        if (ballisticLeadTimeSec == null || !Double.isFinite(ballisticLeadTimeSec)) {
          break;
        }
        leadTimeOfFlightSec = ballisticLeadTimeSec;
        lookaheadPose =
            new Pose2d(
                turretCenterPose.getTranslation().plus(turretCenterVelocity.times(leadTimeOfFlightSec)),
                turretCenterPose.getRotation());
        lookaheadDist = target.getDistance(lookaheadPose.getTranslation()) + distanceOffset;
        shotCommand =
            getStaticShotCommandForDistance(
                profile, lookaheadDist, targetGeometry, turretConfig, robotPose, robotHeading);
      }
    }
    if (targetGeometry == null) {
      shotCommand =
          getStaticShotCommandForDistance(
              profile, lookaheadDist, targetGeometry, turretConfig, robotPose, robotHeading);
    }

    Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle().minus(robotHeading);
    double hoodAngle = shotCommand.hoodAngleRad();
    double flywheelSpeed = shotCommand.flywheelSpeedRadPerSec();

    BallisticState ballisticState =
        targetGeometry == null
            ? BallisticState.invalid()
            : computeBallisticState(
                robotHeading.plus(turretAngle),
                hoodAngle,
                flywheelSpeed,
                targetGeometry,
                turretConfig,
                robotPose,
                robotHeading,
                fieldChassisSpeeds);

    return new ShotSolution(
        ShotModel.TABLE_2D,
        true,
        turretAngle,
        hoodAngle,
        flywheelSpeed,
        leadTimeOfFlightSec,
        lookaheadPose,
        lookaheadDist,
        ballisticState);
  }

  private ShotSolution createHybrid3dLeadSolution(
      Translation2d target,
      Transform2d robotToTurret,
      ShotProfile profile,
      double distanceOffset,
      Pose2d robotPose,
      Rotation2d robotHeading,
      ChassisSpeeds fieldChassisSpeeds,
      TargetPlaneGeometry targetGeometry) {
    Pose2d turretCenterPose = robotPose.transformBy(robotToTurret);
    double staticDistance = target.getDistance(turretCenterPose.getTranslation()) + distanceOffset;
    TurretBallisticsConfig turretConfig = resolveTurretConfig(robotToTurret);
    StaticShotCommand shotCommand =
        getStaticShotCommandForDistance(
            profile, staticDistance, targetGeometry, turretConfig, robotPose, robotHeading);
    double hoodAngle = shotCommand.hoodAngleRad();
    double flywheelSpeed = shotCommand.flywheelSpeedRadPerSec();

    if (targetGeometry == null) {
      return new ShotSolution(
          ShotModel.HYBRID_3D_LEAD,
          false,
          new Rotation2d(),
          hoodAngle,
          flywheelSpeed,
          Double.NaN,
          new Pose2d(),
          staticDistance,
          BallisticState.invalid());
    }

    Translation2d launchBasePosition =
        robotPose.getTranslation().plus(turretConfig.launchBaseOffsetRobot().rotateBy(robotHeading));
    Rotation2d shotHeadingField = targetGeometry.center().minus(launchBasePosition).getAngle();

    BallisticState ballisticState = BallisticState.invalid();
    for (int i = 0; i < HYBRID_HEADING_SOLVE_ITERATIONS; i++) {
      ballisticState =
          computeBallisticState(
              shotHeadingField,
              hoodAngle,
              flywheelSpeed,
              targetGeometry,
              turretConfig,
              robotPose,
              robotHeading,
              fieldChassisSpeeds);
      if (!ballisticState.valid()) {
        return new ShotSolution(
            ShotModel.HYBRID_3D_LEAD,
            false,
            new Rotation2d(),
            hoodAngle,
            flywheelSpeed,
            Double.NaN,
            new Pose2d(),
            staticDistance,
            ballisticState);
      }

      Rotation2d updatedHeading =
          ballisticState.correctedTargetPoint().minus(ballisticState.launchBasePosition()).getAngle();
      if (Math.abs(updatedHeading.minus(shotHeadingField).getRadians()) < 1e-6) {
        shotHeadingField = updatedHeading;
        break;
      }
      shotHeadingField = updatedHeading;
    }

    ballisticState =
        computeBallisticState(
            shotHeadingField,
            hoodAngle,
            flywheelSpeed,
            targetGeometry,
            turretConfig,
            robotPose,
            robotHeading,
            fieldChassisSpeeds);
    if (!ballisticState.valid()) {
      return new ShotSolution(
          ShotModel.HYBRID_3D_LEAD,
          false,
          new Rotation2d(),
          hoodAngle,
          flywheelSpeed,
          Double.NaN,
          new Pose2d(),
          staticDistance,
          ballisticState);
    }

    Rotation2d turretAngle = shotHeadingField.minus(robotHeading);
    Pose2d lookaheadPose = new Pose2d(ballisticState.correctedTargetPoint(), shotHeadingField);
    return new ShotSolution(
        ShotModel.HYBRID_3D_LEAD,
        true,
        turretAngle,
        hoodAngle,
        flywheelSpeed,
        ballisticState.timeOfFlightSec(),
        lookaheadPose,
        staticDistance,
        ballisticState);
  }

  private StaticShotCommand getStaticShotCommandForDistance(
      ShotProfile profile,
      double distanceMeters,
      TargetPlaneGeometry targetGeometry,
      TurretBallisticsConfig turretConfig,
      Pose2d robotPose,
      Rotation2d robotHeading) {
    double hoodAngleRad =
        MathUtil.clamp(
            profile.getHoodAngle(distanceMeters).getRadians(),
            turretConfig.minHoodAngleRad,
            turretConfig.maxHoodAngleRad);
    double flywheelSpeedRadPerSec =
        MathUtil.clamp(profile.getFlywheelSpeed(distanceMeters), 0.0, MAX_FLYWHEEL_SPEED_RAD_PER_SEC);

    if (targetGeometry != null) {
      Double requiredFlywheelSpeedRadPerSec =
          estimateRequiredFlywheelSpeedRadPerSec(
              targetGeometry, turretConfig, robotPose, robotHeading, hoodAngleRad);
      if (requiredFlywheelSpeedRadPerSec != null && Double.isFinite(requiredFlywheelSpeedRadPerSec)) {
        flywheelSpeedRadPerSec =
            MathUtil.clamp(
                Math.max(flywheelSpeedRadPerSec, requiredFlywheelSpeedRadPerSec),
                0.0,
                MAX_FLYWHEEL_SPEED_RAD_PER_SEC);
      }
    }

    return new StaticShotCommand(hoodAngleRad, flywheelSpeedRadPerSec);
  }

  private static Double estimateRequiredFlywheelSpeedRadPerSec(
      TargetPlaneGeometry targetGeometry,
      TurretBallisticsConfig turretConfig,
      Pose2d robotPose,
      Rotation2d robotHeading,
      double hoodAngleRad) {
    double launchPitchRad = turretConfig.launchPitchRad(hoodAngleRad);
    double launchPathLengthMeters = turretConfig.launchPathLengthMeters;
    double launchForwardMeters = launchPathLengthMeters * Math.cos(launchPitchRad);
    double launchHeightMeters =
        turretConfig.launchBaseHeightMeters() + (launchPathLengthMeters * Math.sin(launchPitchRad));

    Translation2d launchBasePosition =
        robotPose.getTranslation().plus(turretConfig.launchBaseOffsetRobot().rotateBy(robotHeading));
    Rotation2d shotHeadingField = targetGeometry.center().minus(launchBasePosition).getAngle();
    Rotation2d turretRelativeHeading = shotHeadingField.minus(robotHeading);
    Translation2d muzzleOffsetRobot =
        turretConfig.launchBaseOffsetRobot().plus(new Translation2d(launchForwardMeters, turretRelativeHeading));
    Translation2d launchPosition =
        robotPose.getTranslation().plus(muzzleOffsetRobot.rotateBy(robotHeading));

    double horizontalDistanceMeters = targetGeometry.center().getDistance(launchPosition);
    double verticalDeltaMeters = targetGeometry.heightMeters() - launchHeightMeters;
    double cosPitch = Math.cos(launchPitchRad);
    double denominator =
        2.0
            * cosPitch
            * cosPitch
            * ((horizontalDistanceMeters * Math.tan(launchPitchRad)) - verticalDeltaMeters);

    if (horizontalDistanceMeters <= 1e-6 || denominator <= 1e-9) {
      return MAX_FLYWHEEL_SPEED_RAD_PER_SEC;
    }

    double launchSpeedMps =
        Math.sqrt(
            (GRAVITY_METERS_PER_SEC2 * horizontalDistanceMeters * horizontalDistanceMeters)
                / denominator);
    double coefficient = ballisticBallSpeedMetersPerSecPerRPM.get();
    if (!Double.isFinite(coefficient) || coefficient <= 1e-9) {
      return MAX_FLYWHEEL_SPEED_RAD_PER_SEC;
    }

    double requiredRpm = launchSpeedMps / coefficient;
    return Units.rotationsPerMinuteToRadiansPerSecond(requiredRpm);
  }

  private static Double estimateStationaryBallisticTimeOfFlightSec(
      TargetPlaneGeometry targetGeometry,
      TurretBallisticsConfig turretConfig,
      Pose2d robotPose,
      Rotation2d robotHeading,
      double hoodAngleRad,
      double flywheelSpeedRadPerSec) {
    Translation2d launchBasePosition =
        robotPose.getTranslation().plus(turretConfig.launchBaseOffsetRobot().rotateBy(robotHeading));
    Rotation2d shotHeadingField = targetGeometry.center().minus(launchBasePosition).getAngle();
    BallisticState ballisticState =
        computeBallisticState(
            shotHeadingField,
            hoodAngleRad,
            flywheelSpeedRadPerSec,
            targetGeometry,
            turretConfig,
            robotPose,
            robotHeading,
            new ChassisSpeeds());
    if (!ballisticState.valid() || !Double.isFinite(ballisticState.timeOfFlightSec())) {
      return null;
    }
    return ballisticState.timeOfFlightSec();
  }

  private static BallisticState computeBallisticState(
      Rotation2d shotHeadingField,
      double hoodAngleRad,
      double flywheelSpeedRadPerSec,
      TargetPlaneGeometry targetGeometry,
      TurretBallisticsConfig turretConfig,
      Pose2d robotPose,
      Rotation2d robotHeading,
      ChassisSpeeds fieldChassisSpeeds) {
    double launchPitchRad = turretConfig.launchPitchRad(hoodAngleRad);
    double launchPathLengthMeters = turretConfig.launchPathLengthMeters;
    double launchForwardMeters = launchPathLengthMeters * Math.cos(launchPitchRad);
    double launchHeightMeters =
        turretConfig.launchBaseHeightMeters() + (launchPathLengthMeters * Math.sin(launchPitchRad));
    double flywheelRpm = Units.radiansPerSecondToRotationsPerMinute(flywheelSpeedRadPerSec);
    double launchSpeedMps = flywheelRpm * ballisticBallSpeedMetersPerSecPerRPM.get();

    Translation2d launchBaseOffsetRobot = turretConfig.launchBaseOffsetRobot();
    Translation2d launchBasePosition =
        robotPose.getTranslation().plus(launchBaseOffsetRobot.rotateBy(robotHeading));
    Rotation2d turretRelativeHeading = shotHeadingField.minus(robotHeading);
    Translation2d muzzleOffsetRobot =
        launchBaseOffsetRobot.plus(new Translation2d(launchForwardMeters, turretRelativeHeading));
    Translation2d launchPosition =
        robotPose.getTranslation().plus(muzzleOffsetRobot.rotateBy(robotHeading));
    Pose3d launchPose =
        new Pose3d(launchPosition.getX(), launchPosition.getY(), launchHeightMeters, new Rotation3d());

    double horizontalLaunchSpeedMps = launchSpeedMps * Math.cos(launchPitchRad);
    double verticalLaunchSpeedMps = launchSpeedMps * Math.sin(launchPitchRad);
    Translation2d rawChassisVelocityAtMuzzle =
        computeFieldVelocityAtRobotOffset(muzzleOffsetRobot, robotHeading, fieldChassisSpeeds);
    double motionCompensationScale = computeMotionCompensationScale(rawChassisVelocityAtMuzzle);
    Translation2d chassisVelocityAtMuzzle =
        rawChassisVelocityAtMuzzle.times(motionCompensationScale);

    double[] crossingTimesSec =
        solvePlaneCrossTimesAtHeight(
            launchHeightMeters, verticalLaunchSpeedMps, targetGeometry.heightMeters());
    if (crossingTimesSec.length == 0) {
      return new BallisticState(
          false,
          launchBasePosition,
          INVALID_TRANSLATION,
          launchPose,
          launchPitchRad,
          launchSpeedMps,
          Double.NaN,
          rawChassisVelocityAtMuzzle,
          motionCompensationScale,
          chassisVelocityAtMuzzle,
          INVALID_TRANSLATION,
          Double.NaN,
          Double.NaN,
          false);
    }

    Translation2d totalHorizontalVelocity =
        chassisVelocityAtMuzzle.plus(new Translation2d(horizontalLaunchSpeedMps, shotHeadingField));
    Translation2d launchToTarget = targetGeometry.center().minus(launchPosition);
    double targetDistance = launchToTarget.getNorm();
    BallisticCandidate bestCandidate = null;
    for (double crossingTimeSec : crossingTimesSec) {
      Translation2d correctedTargetPoint =
          targetGeometry.center().minus(chassisVelocityAtMuzzle.times(crossingTimeSec));
      Translation2d predictedCrossingPoint =
          launchPosition.plus(totalHorizontalVelocity.times(crossingTimeSec));

      double rangeErrorMeters = Double.NaN;
      double lateralErrorMeters = Double.NaN;
      if (targetDistance > 1e-6) {
        Translation2d launchToCross = predictedCrossingPoint.minus(launchPosition);
        double along =
            ((launchToCross.getX() * launchToTarget.getX())
                    + (launchToCross.getY() * launchToTarget.getY()))
                / targetDistance;
        rangeErrorMeters = along - targetDistance;
        lateralErrorMeters =
            ((launchToCross.getX() * launchToTarget.getY())
                    - (launchToCross.getY() * launchToTarget.getX()))
                / targetDistance;
      }

      boolean withinTolerance =
          Double.isFinite(lateralErrorMeters)
              && Math.abs(lateralErrorMeters) <= targetGeometry.lateralToleranceMeters();
      BallisticCandidate candidate =
          new BallisticCandidate(
              crossingTimeSec,
              correctedTargetPoint,
              predictedCrossingPoint,
              rangeErrorMeters,
              lateralErrorMeters,
              withinTolerance,
              predictedCrossingPoint.getDistance(targetGeometry.center()));
      if (bestCandidate == null || candidate.planarErrorMeters() < bestCandidate.planarErrorMeters()) {
        bestCandidate = candidate;
      }
    }

    if (bestCandidate == null) {
      return new BallisticState(
          false,
          launchBasePosition,
          INVALID_TRANSLATION,
          launchPose,
          launchPitchRad,
          launchSpeedMps,
          Double.NaN,
          rawChassisVelocityAtMuzzle,
          motionCompensationScale,
          chassisVelocityAtMuzzle,
          INVALID_TRANSLATION,
          Double.NaN,
          Double.NaN,
          false);
    }

    return new BallisticState(
        true,
        launchBasePosition,
        bestCandidate.correctedTargetPoint(),
        launchPose,
        launchPitchRad,
        launchSpeedMps,
        bestCandidate.timeOfFlightSec(),
        rawChassisVelocityAtMuzzle,
        motionCompensationScale,
        chassisVelocityAtMuzzle,
        bestCandidate.predictedCrossingPoint(),
        bestCandidate.rangeErrorMeters(),
        bestCandidate.lateralErrorMeters(),
        bestCandidate.withinLateralTolerance());
  }

  private void logSolution(
      String profileName, String solutionName, ShotSolution solution, TargetPlaneGeometry targetGeometry) {
    String prefix = "SuperStructure/ShotCalculator/" + profileName + "/" + solutionName;
    BallisticState ballisticState = solution.ballisticState();

    Logger.recordOutput(prefix + "/Valid", solution.valid());
    Logger.recordOutput(prefix + "/TurretAngleRad", solution.turretAngle().getRadians());
    Logger.recordOutput(prefix + "/HoodAngleRad", solution.hoodAngle());
    Logger.recordOutput(prefix + "/FlywheelRadsPerSec", solution.flywheelSpeed());
    Logger.recordOutput(prefix + "/LeadTimeOfFlightSec", solution.leadTimeOfFlightSec());
    Logger.recordOutput(prefix + "/LookaheadPose", solution.lookaheadPose());
    Logger.recordOutput(prefix + "/LookaheadDist", solution.lookaheadDist());
    Logger.recordOutput(prefix + "/LaunchPose", ballisticState.launchPose());
    Logger.recordOutput(prefix + "/LaunchPitchDeg", Math.toDegrees(ballisticState.launchPitchRad()));
    Logger.recordOutput(prefix + "/LaunchSpeedMps", ballisticState.launchSpeedMps());
    Logger.recordOutput(prefix + "/BallisticTimeOfFlightSec", ballisticState.timeOfFlightSec());
    Logger.recordOutput(prefix + "/MotionCompScale", ballisticState.motionCompensationScale());
    Logger.recordOutput(prefix + "/RawMuzzleVelocityMps", ballisticState.rawChassisVelocityAtMuzzle().getNorm());
    Logger.recordOutput(prefix + "/ScaledMuzzleVelocityMps", ballisticState.chassisVelocityAtMuzzle().getNorm());
    Logger.recordOutput(
        prefix + "/PredictedCrossPose",
        targetGeometry == null
            ? INVALID_POSE
            : new Pose3d(
                ballisticState.predictedCrossingPoint().getX(),
                ballisticState.predictedCrossingPoint().getY(),
                targetGeometry.heightMeters(),
                new Rotation3d()));
    Logger.recordOutput(prefix + "/RangeErrorM", ballisticState.rangeErrorMeters());
    Logger.recordOutput(prefix + "/LateralErrorM", ballisticState.lateralErrorMeters());
    Logger.recordOutput(prefix + "/WithinLateralTolerance", ballisticState.withinLateralTolerance());
  }

  private static TargetPlaneGeometry resolveTargetPlaneGeometry(Translation2d target) {
    Translation2d hubCenter =
        GeomUtil.apply(
            new Translation2d(
                FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY()));
    if (target.getDistance(hubCenter) > HUB_TARGET_MATCH_EPSILON_METERS) {
      return null;
    }

    return new TargetPlaneGeometry(
        "Hub72Plane",
        hubCenter,
        AdvancedMechanismConstants.Turret.hubScoringPlaneHeightMeters,
        AdvancedMechanismConstants.Turret.hubScoringPlaneLateralToleranceMeters);
  }

  private static TurretBallisticsConfig resolveTurretConfig(Transform2d robotToTurret) {
    double leftError = transformTranslationError(robotToTurret, LEFT_TURRET_CONFIG.turretCenterTransform);
    double rightError = transformTranslationError(robotToTurret, RIGHT_TURRET_CONFIG.turretCenterTransform);
    return leftError <= rightError ? LEFT_TURRET_CONFIG : RIGHT_TURRET_CONFIG;
  }

  private static double transformTranslationError(Transform2d a, Transform2d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  private static Translation2d computeFieldVelocityAtRobotOffset(
      Translation2d robotOffset,
      Rotation2d robotHeading,
      ChassisSpeeds fieldChassisSpeeds) {
    Translation2d linearVelocity =
        new Translation2d(fieldChassisSpeeds.vxMetersPerSecond, fieldChassisSpeeds.vyMetersPerSecond);
    Translation2d fieldOffset = robotOffset.rotateBy(robotHeading);
    Translation2d rotationalVelocity =
        fieldOffset.rotateBy(Rotation2d.fromDegrees(90.0))
            .times(fieldChassisSpeeds.omegaRadiansPerSecond);
    return linearVelocity.plus(rotationalVelocity);
  }

  private static double computeMotionCompensationScale(Translation2d compensationVelocity) {
    double deadbandSpeed = Math.max(0.0, motionCompensationDeadbandSpeedMetersPerSec.get());
    double fullSpeed = Math.max(0.0, motionCompensationFullSpeedMetersPerSec.get());
    double compensationSpeed = compensationVelocity.getNorm();

    if (fullSpeed <= deadbandSpeed + 1e-9) {
      return compensationSpeed >= deadbandSpeed ? 1.0 : 0.0;
    }

    return MathUtil.clamp(
        (compensationSpeed - deadbandSpeed) / (fullSpeed - deadbandSpeed), 0.0, 1.0);
  }

  private static double[] solvePlaneCrossTimesAtHeight(
      double launchHeightMeters, double launchVerticalVelocityMps, double targetHeightMeters) {
    double discriminant =
        (launchVerticalVelocityMps * launchVerticalVelocityMps)
            - (2.0 * GRAVITY_METERS_PER_SEC2 * (targetHeightMeters - launchHeightMeters));
    if (discriminant < 0.0) {
      return new double[0];
    }

    double sqrtDiscriminant = Math.sqrt(discriminant);
    double ascendingTimeSec = (launchVerticalVelocityMps - sqrtDiscriminant) / GRAVITY_METERS_PER_SEC2;
    double descendingTimeSec = (launchVerticalVelocityMps + sqrtDiscriminant) / GRAVITY_METERS_PER_SEC2;
    boolean ascendingValid = ascendingTimeSec > 0.0 && Double.isFinite(ascendingTimeSec);
    boolean descendingValid = descendingTimeSec > 0.0 && Double.isFinite(descendingTimeSec);

    if (ascendingValid && descendingValid && Math.abs(ascendingTimeSec - descendingTimeSec) > 1e-9) {
      return new double[] {ascendingTimeSec, descendingTimeSec};
    }
    if (ascendingValid) {
      return new double[] {ascendingTimeSec};
    }
    if (descendingValid) {
      return new double[] {descendingTimeSec};
    }
    return new double[0];
  }

  private static double fitBaseBallSpeedMetersPerSecPerRPM() {
    double sumRpmTimesSpeed = 0.0;
    double sumRpmSquared = 0.0;

    double launchBaseHeightMeters = AdvancedMechanismConstants.Turret.robotToLeftTurretLaunchBase.getZ();
    double launchPathLengthMeters = AdvancedMechanismConstants.Turret.leftLaunchPathLengthMeters;
    double pitchOffsetRad = AdvancedMechanismConstants.Turret.leftLaunchPitchOffsetRads;
    double pitchScale = AdvancedMechanismConstants.Turret.leftLaunchPitchScale;
    double targetHeightMeters = AdvancedMechanismConstants.Turret.hubScoringPlaneHeightMeters;

    for (TrustedFitPoint point : TRUSTED_FIT_POINTS) {
      double launchPitchRad = pitchOffsetRad + (pitchScale * point.hoodRadians());
      double horizontalLaunchDistanceMeters =
          point.distanceMeters() - (launchPathLengthMeters * Math.cos(launchPitchRad));
      double launchHeight =
          launchBaseHeightMeters + (launchPathLengthMeters * Math.sin(launchPitchRad));
      double verticalDeltaMeters = targetHeightMeters - launchHeight;
      double denominator =
          2.0
              * Math.pow(Math.cos(launchPitchRad), 2)
              * ((horizontalLaunchDistanceMeters * Math.tan(launchPitchRad)) - verticalDeltaMeters);
      if (horizontalLaunchDistanceMeters <= 0.0 || denominator <= 0.0) {
        continue;
      }

      double requiredLaunchSpeedMps =
          Math.sqrt(
              (GRAVITY_METERS_PER_SEC2
                      * horizontalLaunchDistanceMeters
                      * horizontalLaunchDistanceMeters)
                  / denominator);
      sumRpmTimesSpeed += point.rpm() * requiredLaunchSpeedMps;
      sumRpmSquared += point.rpm() * point.rpm();
    }

    if (sumRpmSquared <= 1e-9) {
      return 0.001682;
    }
    return sumRpmTimesSpeed / sumRpmSquared;
  }
}
