package frc.robot.commands.drive.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.utils.vision.VisionConstants;

import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIO.ObjDetectTxyObservation;

public class AimToObject extends Command {
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
  }

  @Override
  public void execute() {
    hasValidObservation = false;
    latestClusterCount = 0;
    latestClusterScore = 0.0;
    Optional<PreferredObjDetectObservation> preferredObsOpt =
        RobotContainer.visionS.getPreferredObjDetectObservation(cam, desiredClassId);

    if (preferredObsOpt.isPresent()) {
      ObjDetectTxyObservation obs = preferredObsOpt.get().observation();
      boolean freshOk = (Timer.getTimestamp() - obs.timestamp()) < staleTime.get();
      if (freshOk) {
        hasValidObservation = true;
        latestTxRadians = obs.tx().getRadians();
        latestTyRadians = obs.ty().getRadians();
        latestDistanceMeters = obs.distanceMeters();
        latestClusterCount = preferredObsOpt.get().clusterCount();
        latestClusterScore = preferredObsOpt.get().clusterScore();
      }
    }
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds desiredSpeeds = hasValidObservation
        ? buildObjectTrackingSpeeds(robotPose)
        : buildSearchSpeeds(robotPose);

    drive.setChassisSpeeds(desiredSpeeds);

    Logger.recordOutput("Drive/AimToObject/HasObservation", hasValidObservation);
    Logger.recordOutput("Drive/AimToObject/txRad", latestTxRadians);
    Logger.recordOutput("Drive/AimToObject/tyRad", latestTyRadians);
    Logger.recordOutput("Drive/AimToObject/distance", latestDistanceMeters);
    Logger.recordOutput("Drive/AimToObject/ClusterCount", latestClusterCount);
    Logger.recordOutput("Drive/AimToObject/ClusterScore", latestClusterScore);
  }

  private ChassisSpeeds buildObjectTrackingSpeeds(Pose2d robotPose) {
    double angularCommand = 0.0;
    double forwardCommand = 0.0;
    double dTx = latestTxRadians - prevTxRadians;

    if (Math.abs(latestTxRadians) > txTolerance.get()) {
      angularCommand = -(kPTx.get() * latestTxRadians + kDTx.get() * dTx);
      angularCommand = MathUtil.clamp(angularCommand, -maxRotation.get(), maxRotation.get());
    }

    double distanceError = latestDistanceMeters - desiredDistanceMeters;
    if (Math.abs(distanceError) > distanceTolerance.get()) {
      forwardCommand = kPDistance.get() * distanceError;
      forwardCommand = MathUtil.clamp(forwardCommand, -maxSpeed.get(), maxSpeed.get());
    }

    Rotation2d tx = Rotation2d.fromRadians(latestTxRadians);
    Rotation2d ty = Rotation2d.fromRadians(latestTyRadians);
    Translation2d objectField =
        limitAutoIntakeTargetToAllianceSide(
            GeomUtil.projectObjectObservationToField(
                robotPose,
                getRobotToCameraPose(),
                tx,
                ty,
                latestDistanceMeters));

    Translation2d robotToObject = objectField.minus(robotPose.getTranslation());
    Translation2d driveVelocity = Translation2d.kZero;
    if (robotToObject.getNorm() > 1e-6 && Math.abs(forwardCommand) > 1e-6) {
      driveVelocity = robotToObject.div(robotToObject.getNorm()).times(forwardCommand);
      driveVelocity = GeomUtil.limitVelocityTowardFieldEdge(
          driveVelocity,
          robotPose.getTranslation(),
          objectField,
          wallSlowDistanceMeters.get(),
          wallMaxApproachSpeedMetersPerSec.get());
      driveVelocity = limitAutoIntakeVelocityToAllianceSide(driveVelocity, robotPose.getTranslation());
    }

    prevTxRadians = latestTxRadians;
    Logger.recordOutput("Drive/AimToObject/ObjectFieldPose", new Pose2d(objectField, robotPose.getRotation()));
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
    Pose2d searchPose = getSearchPose(robotPose);
    Translation2d toSearchPose = searchPose.getTranslation().minus(robotPose.getTranslation());
    double distanceToSearchPose = toSearchPose.getNorm();

    Translation2d driveVelocity = Translation2d.kZero;
    if (distanceToSearchPose > searchPoseToleranceMeters.get()) {
      double speed = Math.min(searchMaxSpeedMetersPerSec.get(), searchDriveKp.get() * distanceToSearchPose);
      driveVelocity = toSearchPose.div(distanceToSearchPose).times(speed);
      driveVelocity = GeomUtil.limitVelocityTowardFieldEdge(
          driveVelocity,
          robotPose.getTranslation(),
          searchPose.getTranslation(),
          wallSlowDistanceMeters.get(),
          wallMaxApproachSpeedMetersPerSec.get());
      driveVelocity = limitAutoIntakeVelocityToAllianceSide(driveVelocity, robotPose.getTranslation());
    }

    double headingErrorRad = searchPose.getRotation().minus(robotPose.getRotation()).getRadians();
    double angularCommand = MathUtil.clamp(
        searchRotationKp.get() * headingErrorRad,
        -maxRotation.get(),
        maxRotation.get());

    Logger.recordOutput("Drive/AimToObject/SearchPose", searchPose);
    Logger.recordOutput("Drive/AimToObject/SearchDistance", distanceToSearchPose);
    Logger.recordOutput("Drive/AimToObject/SearchAngularCommand", angularCommand);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        driveVelocity.getX(),
        driveVelocity.getY(),
        angularCommand,
        robotPose.getRotation());
  }

  private Pose2d getSearchPose(Pose2d robotPose) {
    Translation2d hubCenter = GeomUtil.apply(FieldConstants.Hub.innerCenterPoint, false).toTranslation2d();
    Translation2d fromHub = robotPose.getTranslation().minus(hubCenter);
    if (fromHub.getNorm() < 1e-6) {
      fromHub = new Translation2d(1.0, 0.0);
    }

    Translation2d searchPoint =
        hubCenter.plus(fromHub.div(fromHub.getNorm()).times(searchCircleRadiusMeters.get()));
    searchPoint = clampToField(searchPoint, searchFieldEdgeMarginMeters.get());
    searchPoint = limitAutoIntakeTargetToAllianceSide(searchPoint);
    return new Pose2d(searchPoint, getSearchHeading(searchPoint, hubCenter));
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

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    RobotContainer.currentPath = "";
    isFinished = true;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
