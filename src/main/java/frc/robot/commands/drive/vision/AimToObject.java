package frc.robot.commands.drive.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.vision.Vision.PreferredObjDetectObservation;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.vision.VisionConstants;

import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIO.ObjDetectTxyObservation;

public class AimToObject extends Command {
  private static final double TRENCH_WALL_THICKNESS_METERS = Units.inchesToMeters(12.0);

  private final DrivetrainS drive;
  private final CameraID cam;
  private final int desiredClassId;
  private final double desiredDistanceMeters;

  private final LoggableTunedNumber kPTx = new LoggableTunedNumber("AimToObjectTx/kP", 4,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber kDTx = new LoggableTunedNumber("AimToObjectTx/kD", 0.6,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber kPDistance = new LoggableTunedNumber("AimToObjectTx/kPDistance", 3.5,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber maxSpeed = new LoggableTunedNumber("AimToObjectTx/maxSpeedMetersPerSec", 4.5,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber maxRotation = new LoggableTunedNumber("AimToObjectTx/MaxRotationRadPerSec", 15,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber maxAccelMetersPerSecSq =
      new LoggableTunedNumber("AimToObjectTx/MaxAccelMetersPerSecSq", 2.0, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber maxAngularAccelRadPerSecSq =
      new LoggableTunedNumber("AimToObjectTx/MaxAngularAccelRadPerSecSq", 8.0, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber wallSlowDistanceMeters =
      new LoggableTunedNumber("AimToObjectTx/WallSlowDistanceMeters", 0.9, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber wallMaxApproachSpeedMetersPerSec =
      new LoggableTunedNumber("AimToObjectTx/WallMaxApproachSpeedMetersPerSec", 1.2, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchCircleRadiusMeters =
      new LoggableTunedNumber("AimToObjectTx/SearchCircleRadiusMeters", 2.5, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchPoseToleranceMeters =
      new LoggableTunedNumber("AimToObjectTx/SearchPoseToleranceMeters", 0.2, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchDriveKp =
      new LoggableTunedNumber("AimToObjectTx/SearchDriveKp", 2.0, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchMaxSpeedMetersPerSec =
      new LoggableTunedNumber("AimToObjectTx/SearchMaxSpeedMetersPerSec", 2.0, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchRotationKp =
      new LoggableTunedNumber("AimToObjectTx/SearchRotationKp", 4.0, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchBiasDeg =
      new LoggableTunedNumber("AimToObjectTx/SearchBiasDeg", 15.0, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchSweepDeg =
      new LoggableTunedNumber("AimToObjectTx/SearchSweepDeg", 20.0, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchSweepPeriodSec =
      new LoggableTunedNumber("AimToObjectTx/SearchSweepPeriodSec", 1.5, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchFieldEdgeMarginMeters =
      new LoggableTunedNumber("AimToObjectTx/SearchFieldEdgeMarginMeters", 0.35, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber autoCenterLineMarginMeters =
      new LoggableTunedNumber("AimToObjectTx/AutoCenterLineMarginMeters", 0.15, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber searchCircleDriverAssistDistanceMeters =
      new LoggableTunedNumber("AimToObjectTx/SearchCircleDriverAssistDistanceMeters", 0.25,
          TuningConstants.isTuningMacros);
  private final LoggableTunedNumber towerNoGoMarginMeters =
      new LoggableTunedNumber("AimToObjectTx/TowerNoGoMarginMeters", 0.3, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber trenchWallNoGoMarginMeters =
      new LoggableTunedNumber("AimToObjectTx/TrenchWallNoGoMarginMeters", 0.15, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber robotCenterToFrontMeters =
      new LoggableTunedNumber("AimToObjectTx/RobotCenterToFrontMeters", Units.inchesToMeters(15.0),
          TuningConstants.isTuningMacros);
  private final LoggableTunedNumber intakeExtensionBeyondFrontMeters =
      new LoggableTunedNumber("AimToObjectTx/IntakeExtensionBeyondFrontMeters", Units.inchesToMeters(12.0),
          TuningConstants.isTuningMacros);
  private final LoggableTunedNumber intakeSafetyMarginMeters =
      new LoggableTunedNumber("AimToObjectTx/IntakeSafetyMarginMeters", 0.0, TuningConstants.isTuningMacros);

  // tolerances
  private final LoggableTunedNumber txTolerance = new LoggableTunedNumber("AimToObjectTx/txToleranceRad", .05,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber distanceTolerance = new LoggableTunedNumber("AimToObjectTx/distanceToleranceMeters",
      Units.inchesToMeters(3), TuningConstants.isTuningMacros);
  private final LoggableTunedNumber staleTime = new LoggableTunedNumber("AimToObjectTx/staleTime", .5,
      TuningConstants.isTuningMacros);

  private double prevTxRadians = 0.0; // raw tx (negative=left)
  private double latestTxRadians = 0.0; // raw tx (negative=left)
  private double latestTyRadians = 0.0;
  private double latestDistanceMeters = 0.0;
  private int latestClusterCount = 0;
  private double latestClusterScore = 0.0;
  private boolean hasValidObservation = false;
  private boolean isFinished = false;
  private double searchStartTimestampSec = 0.0;
  private Translation2d latestObjectFieldTarget = Translation2d.kZero;
  private boolean latestObservationBlockedByTower = false;
  private SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(maxAccelMetersPerSecSq.get());
  private SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(maxAccelMetersPerSecSq.get());
  private SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(maxAngularAccelRadPerSecSq.get());
  private ChassisSpeeds lastLimitedSpeeds = new ChassisSpeeds();

  /**
   * Aim at the preferred detected object cluster in a given camera.
   *
   * @param drive                 drivetrain
   * @param cam                   which camera to use
   * @param desiredClassId        object class to aim at (-1 means any class)
   * @param desiredDistanceMeters standoff distance to maintain (0 means just aim
   *                              and GO TOWARDS until we can't see it anymore)
   */
  public AimToObject(DrivetrainS drive,
      CameraID cam,
      int desiredClassId,
      double desiredDistanceMeters) {
    this.drive = drive;
    this.cam = cam;
    this.desiredClassId = desiredClassId;
    this.desiredDistanceMeters = desiredDistanceMeters;
  }

  @Override
  public void initialize() {
    RobotContainer.currentPath = "AIMTOOBJECT_CAM_" + cam + "_CLASS_" + desiredClassId;
    prevTxRadians = 0.0;
    isFinished = false;
    searchStartTimestampSec = Timer.getFPGATimestamp();
    configureAccelerationLimiters(drive.getChassisSpeeds());
  }

  @Override
  public void execute() {
    hasValidObservation = false;
    latestObservationBlockedByTower = false;
    latestClusterCount = 0;
    latestClusterScore = 0.0;
    Logger.recordOutput("Drive/AimToObject/SearchEnabled", false);
    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistEnabled", false);
    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVx", 0.0);
    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVy", 0.0);
    Logger.recordOutput("Drive/AimToObject/SearchDistance", Double.NaN);
    Logger.recordOutput("Drive/AimToObject/SearchAngularCommand", Double.NaN);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusMeters", Double.NaN);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusErrorMeters", Double.NaN);
    Pose2d robotPose = drive.getPose();
    Optional<PreferredObjDetectObservation> preferredObsOpt =
        RobotContainer.visionS.getPreferredObjDetectObservation(cam, desiredClassId);

    if (preferredObsOpt.isPresent()) {
      ObjDetectTxyObservation obs = preferredObsOpt.get().observation();
      boolean freshOk = (Timer.getTimestamp() - obs.timestamp()) < staleTime.get();
      if (freshOk) {
        latestTxRadians = obs.tx().getRadians();
        latestTyRadians = obs.ty().getRadians();
        latestDistanceMeters = obs.distanceMeters();
        latestClusterCount = preferredObsOpt.get().clusterCount();
        latestClusterScore = preferredObsOpt.get().clusterScore();

        Translation2d projectedObjectField =
            GeomUtil.projectObjectObservationToField(
                robotPose,
                getRobotToCameraPose(),
                Rotation2d.fromRadians(latestTxRadians),
                Rotation2d.fromRadians(latestTyRadians),
                latestDistanceMeters);
        Logger.recordOutput(
            "Drive/AimToObject/ObjectFieldProjected",
            new Pose2d(projectedObjectField, robotPose.getRotation()));

        if (isInsideTowerNoGoZone(projectedObjectField)) {
          latestObservationBlockedByTower = true;
        } else {
          hasValidObservation = true;
          latestObjectFieldTarget = limitAutoIntakeTargetToAllianceSide(projectedObjectField);
        }
      }
    }
    ChassisSpeeds rawDesiredSpeeds = hasValidObservation
        ? buildObjectTrackingSpeeds(robotPose, latestObjectFieldTarget)
        : buildSearchSpeeds(robotPose);
    ChassisSpeeds desiredSpeeds = limitCommandAcceleration(rawDesiredSpeeds);

    drive.setChassisSpeeds(desiredSpeeds);

    Logger.recordOutput("Drive/AimToObject/HasObservation", hasValidObservation);
    Logger.recordOutput("Drive/AimToObject/ObservationBlockedByTower", latestObservationBlockedByTower);
    Logger.recordOutput("Drive/AimToObject/txRad", latestTxRadians);
    Logger.recordOutput("Drive/AimToObject/tyRad", latestTyRadians);
    Logger.recordOutput("Drive/AimToObject/distance", latestDistanceMeters);
    Logger.recordOutput("Drive/AimToObject/ClusterCount", latestClusterCount);
    Logger.recordOutput("Drive/AimToObject/ClusterScore", latestClusterScore);
    Logger.recordOutput("Drive/AimToObject/RawCommandVx", rawDesiredSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/AimToObject/RawCommandVy", rawDesiredSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/AimToObject/RawCommandOmega", rawDesiredSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput("Drive/AimToObject/LimitedCommandVx", desiredSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/AimToObject/LimitedCommandVy", desiredSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/AimToObject/LimitedCommandOmega", desiredSpeeds.omegaRadiansPerSecond);
  }

  private void configureAccelerationLimiters(ChassisSpeeds seedSpeeds) {
    xVelocityLimiter = new SlewRateLimiter(maxAccelMetersPerSecSq.get());
    yVelocityLimiter = new SlewRateLimiter(maxAccelMetersPerSecSq.get());
    angularVelocityLimiter = new SlewRateLimiter(maxAngularAccelRadPerSecSq.get());
    xVelocityLimiter.reset(seedSpeeds.vxMetersPerSecond);
    yVelocityLimiter.reset(seedSpeeds.vyMetersPerSecond);
    angularVelocityLimiter.reset(seedSpeeds.omegaRadiansPerSecond);
    lastLimitedSpeeds = seedSpeeds;
  }

  private ChassisSpeeds limitCommandAcceleration(ChassisSpeeds desiredSpeeds) {
    LoggableTunedNumber.ifChanged(
        hashCode(),
        () -> configureAccelerationLimiters(lastLimitedSpeeds),
        maxAccelMetersPerSecSq,
        maxAngularAccelRadPerSecSq);

    lastLimitedSpeeds = new ChassisSpeeds(
        xVelocityLimiter.calculate(desiredSpeeds.vxMetersPerSecond),
        yVelocityLimiter.calculate(desiredSpeeds.vyMetersPerSecond),
        angularVelocityLimiter.calculate(desiredSpeeds.omegaRadiansPerSecond));
    return lastLimitedSpeeds;
  }

  private ChassisSpeeds buildObjectTrackingSpeeds(Pose2d robotPose, Translation2d objectField) {
    double angularCommand = 0.0;
    double forwardCommand = 0.0;
    double dTx = latestTxRadians - prevTxRadians;

    if (Math.abs(latestTxRadians) > txTolerance.get()) {
      angularCommand = -(kPTx.get() * latestTxRadians + kDTx.get() * dTx);
      angularCommand = MathUtil.clamp(angularCommand, -maxRotation.get(), maxRotation.get());
    }

    double distanceError = latestDistanceMeters - desiredDistanceMeters;
    if (desiredDistanceMeters <= 1e-6 || Math.abs(distanceError) > distanceTolerance.get()) {
      forwardCommand = kPDistance.get() * distanceError;
      forwardCommand = MathUtil.clamp(forwardCommand, -maxSpeed.get(), maxSpeed.get());
    }

    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    Translation2d robotToObject = objectField.minus(protectedIntakePoint);
    Translation2d driveVelocity = Translation2d.kZero;
    if (robotToObject.getNorm() > 1e-6 && Math.abs(forwardCommand) > 1e-6) {
      driveVelocity = robotToObject.div(robotToObject.getNorm()).times(forwardCommand);
      driveVelocity = GeomUtil.limitVelocityTowardFieldEdge(
          driveVelocity,
          protectedIntakePoint,
          objectField,
          wallSlowDistanceMeters.get(),
          wallMaxApproachSpeedMetersPerSec.get());
      driveVelocity = limitAutoIntakeVelocityToAllianceSide(driveVelocity, protectedIntakePoint);
      driveVelocity = limitVelocityToAvoidTowerNoGoZone(driveVelocity, robotPose, angularCommand);
      driveVelocity = limitVelocityToAvoidTrenchWallNoGoZone(driveVelocity, robotPose, angularCommand);
    }

    prevTxRadians = latestTxRadians;
    Logger.recordOutput("Drive/AimToObject/ObjectFieldPose", new Pose2d(objectField, robotPose.getRotation()));
    Logger.recordOutput("Drive/AimToObject/ProtectedIntakePoint", new Pose2d(protectedIntakePoint, robotPose.getRotation()));
    Logger.recordOutput("Drive/AimToObject/forwardCommand", forwardCommand);
    Logger.recordOutput("Drive/AimToObject/angularCommand", angularCommand);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(),
        driveVelocity.getY(),
        angularCommand,
        robotPose.getRotation());
  }

  private ChassisSpeeds buildSearchSpeeds(Pose2d robotPose) {
    if (!isInAllianceZone(robotPose)) {
      Logger.recordOutput("Drive/AimToObject/SearchEnabled", false);
      return new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    Logger.recordOutput("Drive/AimToObject/SearchEnabled", true);
    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    Pose2d searchPose = getSearchPose(robotPose);
    Translation2d toSearchPose = searchPose.getTranslation().minus(protectedIntakePoint);
    double distanceToSearchPose = toSearchPose.getNorm();

    Translation2d driveVelocity = Translation2d.kZero;
    if (distanceToSearchPose > searchPoseToleranceMeters.get()) {
      double speed = Math.min(searchMaxSpeedMetersPerSec.get(), searchDriveKp.get() * distanceToSearchPose);
      driveVelocity = toSearchPose.div(distanceToSearchPose).times(speed);
      driveVelocity = GeomUtil.limitVelocityTowardFieldEdge(
          driveVelocity,
          protectedIntakePoint,
          searchPose.getTranslation(),
          wallSlowDistanceMeters.get(),
          wallMaxApproachSpeedMetersPerSec.get());
      driveVelocity = limitAutoIntakeVelocityToAllianceSide(driveVelocity, protectedIntakePoint);
      driveVelocity = limitVelocityToAvoidTowerNoGoZone(driveVelocity, robotPose, 0.0);
      driveVelocity = limitVelocityToAvoidTrenchWallNoGoZone(driveVelocity, robotPose, 0.0);
    }

    Translation2d hubCenter = GeomUtil.apply(FieldConstants.Hub.innerCenterPoint, false).toTranslation2d();
    Translation2d fromHub = protectedIntakePoint.minus(hubCenter);
    double currentSearchRadiusMeters = fromHub.getNorm();
    double searchRadiusErrorMeters = Math.abs(currentSearchRadiusMeters - searchCircleRadiusMeters.get());
    boolean searchCircleDriverAssistEnabled =
        DriverStation.isTeleopEnabled()
            && fromHub.getNorm() > 1e-6
            && searchRadiusErrorMeters <= searchCircleDriverAssistDistanceMeters.get();
    if (searchCircleDriverAssistEnabled) {
      Translation2d tangentialDriverVelocity = getSearchCircleTangentialDriverVelocity(fromHub);
      driveVelocity = driveVelocity.plus(tangentialDriverVelocity);
      if (driveVelocity.getNorm() > searchMaxSpeedMetersPerSec.get()) {
        driveVelocity = driveVelocity.div(driveVelocity.getNorm()).times(searchMaxSpeedMetersPerSec.get());
      }
      driveVelocity = GeomUtil.limitVelocityTowardFieldEdge(
          driveVelocity,
          protectedIntakePoint,
          searchPose.getTranslation(),
          wallSlowDistanceMeters.get(),
          wallMaxApproachSpeedMetersPerSec.get());
      driveVelocity = limitAutoIntakeVelocityToAllianceSide(driveVelocity, protectedIntakePoint);
      driveVelocity = limitVelocityToAvoidTowerNoGoZone(driveVelocity, robotPose, 0.0);
      driveVelocity = limitVelocityToAvoidTrenchWallNoGoZone(driveVelocity, robotPose, 0.0);
      Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVx", tangentialDriverVelocity.getX());
      Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVy", tangentialDriverVelocity.getY());
    }

    double headingErrorRad = searchPose.getRotation().minus(robotPose.getRotation()).getRadians();
    double angularCommand = MathUtil.clamp(
        searchRotationKp.get() * headingErrorRad,
        -maxRotation.get(),
        maxRotation.get());

    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistEnabled", searchCircleDriverAssistEnabled);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusMeters", currentSearchRadiusMeters);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusErrorMeters", searchRadiusErrorMeters);
    Logger.recordOutput("Drive/AimToObject/SearchPose", searchPose);
    Logger.recordOutput("Drive/AimToObject/SearchDistance", distanceToSearchPose);
    Logger.recordOutput("Drive/AimToObject/ProtectedIntakePoint", new Pose2d(protectedIntakePoint, robotPose.getRotation()));
    Logger.recordOutput("Drive/AimToObject/SearchAngularCommand", angularCommand);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(),
        driveVelocity.getY(),
        angularCommand,
        robotPose.getRotation());
  }

  private Translation2d getSearchCircleTangentialDriverVelocity(Translation2d fromHub) {
    Translation2d rawDriverVelocity = getDriverFieldVelocity();
    Translation2d tangentUnit = fromHub.div(fromHub.getNorm()).rotateBy(Rotation2d.fromDegrees(90.0));
    double tangentialSpeedMetersPerSec =
        rawDriverVelocity.getX() * tangentUnit.getX() + rawDriverVelocity.getY() * tangentUnit.getY();
    return tangentUnit.times(tangentialSpeedMetersPerSec);
  }

  private Translation2d getDriverFieldVelocity() {
    double xSpeed = -MathUtil.applyDeadband(
        RobotContainer.driveController.getHID().getLeftY(),
        DriveConstants.DriverConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeed = -MathUtil.applyDeadband(
        RobotContainer.driveController.getHID().getLeftX(),
        DriveConstants.DriverConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    if (Robot.isRed) {
      xSpeed *= -1.0;
      ySpeed *= -1.0;
    }
    return new Translation2d(xSpeed, ySpeed);
  }

  private Pose2d getSearchPose(Pose2d robotPose) {
    Translation2d hubCenter = GeomUtil.apply(FieldConstants.Hub.innerCenterPoint, false).toTranslation2d();
    Translation2d fromHub = getProtectedIntakePoint(robotPose).minus(hubCenter);
    if (fromHub.getNorm() < 1e-6) {
      fromHub = new Translation2d(1.0, 0.0);
    }

    Translation2d searchPoint =
        hubCenter.plus(fromHub.div(fromHub.getNorm()).times(searchCircleRadiusMeters.get()));
    searchPoint = clampToField(searchPoint, searchFieldEdgeMarginMeters.get());
    searchPoint = pushSearchPointOutOfTowerNoGoZone(searchPoint);
    searchPoint = pushSearchPointOutOfTrenchWallNoGoZone(searchPoint);
    searchPoint = limitAutoIntakeTargetToAllianceSide(searchPoint);
    searchPoint = pushSearchPointOutOfTowerNoGoZone(searchPoint);
    searchPoint = pushSearchPointOutOfTrenchWallNoGoZone(searchPoint);
    return new Pose2d(searchPoint, getSearchHeading(searchPoint, hubCenter));
  }

  private Translation2d getProtectedIntakePoint(Pose2d robotPose) {
    return robotPose.getTranslation()
        .plus(new Translation2d(getProtectedIntakeReachMeters(), robotPose.getRotation()));
  }

  private Translation2d getProtectedIntakePoint(Translation2d robotPosition, Rotation2d robotHeading) {
    return robotPosition.plus(new Translation2d(getProtectedIntakeReachMeters(), robotHeading));
  }

  private double getProtectedIntakeReachMeters() {
    return Math.max(0.0, robotCenterToFrontMeters.get())
        + Math.max(0.0, intakeExtensionBeyondFrontMeters.get())
        + Math.max(0.0, intakeSafetyMarginMeters.get());
  }

  private boolean isInAllianceZone(Pose2d robotPose) {
    if (Robot.isRed) {
      return robotPose.getX() >= FieldConstants.LinesVertical.oppAllianceZone;
    }
    return robotPose.getX() <= FieldConstants.LinesVertical.allianceZone;
  }

  private Rotation2d getSearchHeading(Translation2d searchPoint, Translation2d hubCenter) {
    Rotation2d outwardHeading = searchPoint.minus(hubCenter).getAngle();
    double biasSign = searchPoint.getY() >= FieldConstants.FIELD_HEIGHT / 2.0 ? -1.0 : 1.0;
    Rotation2d biasedHeading = outwardHeading.rotateBy(Rotation2d.fromDegrees(biasSign * searchBiasDeg.get()));
    double sweepPeriod = Math.max(searchSweepPeriodSec.get(), 0.1);
    double sweepRadians = Units.degreesToRadians(searchSweepDeg.get())
        * Math.sin((Timer.getFPGATimestamp() - searchStartTimestampSec) * 2.0 * Math.PI / sweepPeriod);
    return biasedHeading.rotateBy(Rotation2d.fromRadians(sweepRadians));
  }

  private Pose3d getRobotToCameraPose() {
    return VisionConstants.cameras[cam.ordinal()].getPose().get();
  }

  private Translation2d clampToField(Translation2d point, double margin) {
    return new Translation2d(
        MathUtil.clamp(point.getX(), margin, FieldConstants.FIELD_WIDTH - margin),
        MathUtil.clamp(point.getY(), margin, FieldConstants.FIELD_HEIGHT - margin));
  }

  private Translation2d limitAutoIntakeTargetToAllianceSide(Translation2d point) {
    if (!DriverStation.isAutonomousEnabled()) {
      return point;
    }

    double centerLineX = FieldConstants.LinesVertical.center;
    double margin = Math.max(0.0, autoCenterLineMarginMeters.get());
    if (Robot.isRed) {
      return new Translation2d(Math.max(point.getX(), centerLineX + margin), point.getY());
    }
    return new Translation2d(Math.min(point.getX(), centerLineX - margin), point.getY());
  }

  private Translation2d limitAutoIntakeVelocityToAllianceSide(
      Translation2d desiredFieldVelocity,
      Translation2d robotPosition) {
    if (!DriverStation.isAutonomousEnabled()) {
      return desiredFieldVelocity;
    }

    double centerLineX = FieldConstants.LinesVertical.center;
    double margin = Math.max(0.0, autoCenterLineMarginMeters.get());
    double dtSec = 0.02;
    double limitedX = desiredFieldVelocity.getX();

    if (Robot.isRed) {
      double minAllowedX = centerLineX + margin;
      double maxNegativeSpeed = (robotPosition.getX() - minAllowedX) / dtSec;
      if (limitedX < 0.0) {
        limitedX = Math.max(limitedX, -Math.max(0.0, maxNegativeSpeed));
      }
    } else {
      double maxAllowedX = centerLineX - margin;
      double maxPositiveSpeed = (maxAllowedX - robotPosition.getX()) / dtSec;
      if (limitedX > 0.0) {
        limitedX = Math.min(limitedX, Math.max(0.0, maxPositiveSpeed));
      }
    }

    return new Translation2d(limitedX, desiredFieldVelocity.getY());
  }

  private Translation2d pushSearchPointOutOfTowerNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, towerNoGoMarginMeters.get());

    double blueMinY = Math.min(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) - margin;
    double blueMaxY = Math.max(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) + margin;
    double blueMaxX = FieldConstants.Tower.frontFaceX + margin;
    if (point.getX() <= blueMaxX && point.getY() >= blueMinY && point.getY() <= blueMaxY) {
      return new Translation2d(blueMaxX + 1e-3, point.getY());
    }

    double redMinY =
        Math.min(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) - margin;
    double redMaxY =
        Math.max(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) + margin;
    double redMinX = FieldConstants.FIELD_WIDTH - FieldConstants.Tower.frontFaceX - margin;
    if (point.getX() >= redMinX && point.getY() >= redMinY && point.getY() <= redMaxY) {
      return new Translation2d(redMinX - 1e-3, point.getY());
    }

    return point;
  }

  private Translation2d pushSearchPointOutOfTrenchWallNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, trenchWallNoGoMarginMeters.get());

    double blueOpeningX = FieldConstants.RightTrench.openingTopLeft.getX();
    double blueMinX = blueOpeningX - FieldConstants.RightTrench.depth - margin;
    double blueMaxX = blueOpeningX + margin;

    double blueBottomMinY = FieldConstants.RightTrench.openingTopLeft.getY() - margin;
    double blueBottomMaxY = FieldConstants.RightTrench.openingTopLeft.getY() + TRENCH_WALL_THICKNESS_METERS + margin;
    if (point.getX() >= blueMinX && point.getX() <= blueMaxX
        && point.getY() >= blueBottomMinY && point.getY() <= blueBottomMaxY) {
      return new Translation2d(point.getX(), blueBottomMaxY + 1e-3);
    }

    double blueTopMinY = FieldConstants.LeftTrench.openingTopRight.getY() - TRENCH_WALL_THICKNESS_METERS - margin;
    double blueTopMaxY = FieldConstants.LeftTrench.openingTopRight.getY() + margin;
    if (point.getX() >= blueMinX && point.getX() <= blueMaxX
        && point.getY() >= blueTopMinY && point.getY() <= blueTopMaxY) {
      return new Translation2d(point.getX(), blueTopMinY - 1e-3);
    }

    double redOpeningX = FieldConstants.RightTrench.oppOpeningTopLeft.getX();
    double redMinX = redOpeningX - margin;
    double redMaxX = redOpeningX + FieldConstants.RightTrench.depth + margin;

    double redBottomMinY = FieldConstants.RightTrench.oppOpeningTopLeft.getY() - margin;
    double redBottomMaxY = FieldConstants.RightTrench.oppOpeningTopLeft.getY() + TRENCH_WALL_THICKNESS_METERS + margin;
    if (point.getX() >= redMinX && point.getX() <= redMaxX
        && point.getY() >= redBottomMinY && point.getY() <= redBottomMaxY) {
      return new Translation2d(point.getX(), redBottomMaxY + 1e-3);
    }

    double redTopMinY = FieldConstants.LeftTrench.oppOpeningTopRight.getY() - TRENCH_WALL_THICKNESS_METERS - margin;
    double redTopMaxY = FieldConstants.LeftTrench.oppOpeningTopRight.getY() + margin;
    if (point.getX() >= redMinX && point.getX() <= redMaxX
        && point.getY() >= redTopMinY && point.getY() <= redTopMaxY) {
      return new Translation2d(point.getX(), redTopMinY - 1e-3);
    }

    return point;
  }

  private boolean isInsideTowerNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, towerNoGoMarginMeters.get());

    double blueMinY = Math.min(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) - margin;
    double blueMaxY = Math.max(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) + margin;
    boolean insideBlueTower =
        point.getX() >= 0.0
            && point.getX() <= FieldConstants.Tower.frontFaceX + margin
            && point.getY() >= blueMinY
            && point.getY() <= blueMaxY;
    if (insideBlueTower) {
      return true;
    }

    double redMinY =
        Math.min(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) - margin;
    double redMaxY =
        Math.max(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) + margin;
    return point.getX() >= FieldConstants.FIELD_WIDTH - FieldConstants.Tower.frontFaceX - margin
        && point.getX() <= FieldConstants.FIELD_WIDTH
        && point.getY() >= redMinY
        && point.getY() <= redMaxY;
  }

  private boolean isInsideTrenchWallNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, trenchWallNoGoMarginMeters.get());

    double blueOpeningX = FieldConstants.RightTrench.openingTopLeft.getX();
    double blueMinX = blueOpeningX - FieldConstants.RightTrench.depth - margin;
    double blueMaxX = blueOpeningX + margin;

    boolean insideBlueBottom =
        point.getX() >= blueMinX
            && point.getX() <= blueMaxX
            && point.getY() >= FieldConstants.RightTrench.openingTopLeft.getY() - margin
            && point.getY() <= FieldConstants.RightTrench.openingTopLeft.getY() + TRENCH_WALL_THICKNESS_METERS + margin;
    if (insideBlueBottom) {
      return true;
    }

    boolean insideBlueTop =
        point.getX() >= blueMinX
            && point.getX() <= blueMaxX
            && point.getY() >= FieldConstants.LeftTrench.openingTopRight.getY() - TRENCH_WALL_THICKNESS_METERS - margin
            && point.getY() <= FieldConstants.LeftTrench.openingTopRight.getY() + margin;
    if (insideBlueTop) {
      return true;
    }

    double redOpeningX = FieldConstants.RightTrench.oppOpeningTopLeft.getX();
    double redMinX = redOpeningX - margin;
    double redMaxX = redOpeningX + FieldConstants.RightTrench.depth + margin;

    boolean insideRedBottom =
        point.getX() >= redMinX
            && point.getX() <= redMaxX
            && point.getY() >= FieldConstants.RightTrench.oppOpeningTopLeft.getY() - margin
            && point.getY() <= FieldConstants.RightTrench.oppOpeningTopLeft.getY() + TRENCH_WALL_THICKNESS_METERS + margin;
    if (insideRedBottom) {
      return true;
    }

    return point.getX() >= redMinX
        && point.getX() <= redMaxX
        && point.getY() >= FieldConstants.LeftTrench.oppOpeningTopRight.getY() - TRENCH_WALL_THICKNESS_METERS - margin
        && point.getY() <= FieldConstants.LeftTrench.oppOpeningTopRight.getY() + margin;
  }

  private Translation2d limitVelocityToAvoidTowerNoGoZone(
      Translation2d desiredFieldVelocity,
      Pose2d robotPose,
      double desiredAngularVelocityRadPerSec) {
    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    if (desiredFieldVelocity.getNorm() <= 1e-6 || isInsideTowerNoGoZone(protectedIntakePoint)) {
      return desiredFieldVelocity;
    }

    Translation2d nextRobotPosition = robotPose.getTranslation().plus(desiredFieldVelocity.times(0.02));
    Rotation2d nextRobotHeading =
        robotPose.getRotation().plus(Rotation2d.fromRadians(desiredAngularVelocityRadPerSec * 0.02));
    Translation2d nextProtectedIntakePoint = getProtectedIntakePoint(nextRobotPosition, nextRobotHeading);
    if (isInsideTowerNoGoZone(nextProtectedIntakePoint)) {
      return Translation2d.kZero;
    }

    return desiredFieldVelocity;
  }

  private Translation2d limitVelocityToAvoidTrenchWallNoGoZone(
      Translation2d desiredFieldVelocity,
      Pose2d robotPose,
      double desiredAngularVelocityRadPerSec) {
    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    if (desiredFieldVelocity.getNorm() <= 1e-6 || isInsideTrenchWallNoGoZone(protectedIntakePoint)) {
      return desiredFieldVelocity;
    }

    Translation2d nextRobotPosition = robotPose.getTranslation().plus(desiredFieldVelocity.times(0.02));
    Rotation2d nextRobotHeading =
        robotPose.getRotation().plus(Rotation2d.fromRadians(desiredAngularVelocityRadPerSec * 0.02));
    Translation2d nextProtectedIntakePoint = getProtectedIntakePoint(nextRobotPosition, nextRobotHeading);
    if (isInsideTrenchWallNoGoZone(nextProtectedIntakePoint)) {
      return Translation2d.kZero;
    }

    return desiredFieldVelocity;
  }

  @Override
  public void end(boolean interrupted) {
    configureAccelerationLimiters(new ChassisSpeeds());
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    RobotContainer.currentPath = "";
    isFinished = true;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
