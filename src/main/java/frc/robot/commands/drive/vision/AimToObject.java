package frc.robot.commands.drive.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
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

  // tolerances
  private final LoggableTunedNumber txTolerance = new LoggableTunedNumber("AimToObjectTx/txToleranceRad", .05,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber distanceTolerance = new LoggableTunedNumber("AimToObjectTx/distanceToleranceMeters",
      Units.inchesToMeters(3), TuningConstants.isTuningMacros);
  private final LoggableTunedNumber staleTime = new LoggableTunedNumber("AimToObjectTx/staleTime", .5,
      TuningConstants.isTuningMacros);

  private double prevTxRadians = 0.0; // raw tx (negative=left)
  private double latestTxRadians = 0.0; // raw tx (negative=left)
  private double latestDistanceMeters = 0.0;
  private boolean hasValidObservation = false;
  private boolean isFinished = false;

  /**
   * Aim at the closest detected object in a given camera.
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
  }

  @Override
  public void execute() {
    hasValidObservation = false;
    Optional<ObjDetectTxyObservation> obsOpt = RobotContainer.visionS.getClosestObjDetectTxyObservation(cam);

    if (obsOpt.isPresent()) {
      var obs = obsOpt.get();
      boolean classOk = (desiredClassId < 0) || (obs.classId() == desiredClassId);
      boolean freshOk = (Timer.getTimestamp() - obs.timestamp()) < staleTime.get();

      if (classOk && freshOk) {
        hasValidObservation = true;
        latestTxRadians = obs.tx().getRadians();
        latestDistanceMeters = obs.distanceMeters();
      }
    }
    if (!hasValidObservation) {
      drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      return;
    }

    double angularCommand = 0.0;
    double forwardCommand = 0.0;

    double dTx = latestTxRadians - prevTxRadians;

    // TURNING:
    // tx is negative (object is TO THE LEFT), we want +omega (turn left/CCW).
    if (Math.abs(latestTxRadians) > txTolerance.get()) {
      angularCommand = -(kPTx.get() * latestTxRadians + kDTx.get() * dTx);
      angularCommand = Math.max(-maxRotation.get(), Math.min(maxRotation.get(), angularCommand));
    }

    double distanceError = latestDistanceMeters - desiredDistanceMeters;
    if (Math.abs(distanceError) > distanceTolerance.get()) {
      forwardCommand = kPDistance.get() * distanceError;
      forwardCommand = Math.max(-maxSpeed.get(), Math.min(maxSpeed.get(), forwardCommand));
    }

    Pose2d robotPose = drive.getPose();
    int camIndex = cam.ordinal();
    Transform2d robotToCamera = GeomUtil.poseToTransform(VisionConstants.cameras[camIndex].getPose().get().toPose2d());
    Pose2d cameraPose = robotPose.plus(robotToCamera);
    // Build an estimated object point in the field, then drive toward it.
    Rotation2d dir = cameraPose.getRotation().minus(new Rotation2d(latestTxRadians));
    Translation2d unit = new Translation2d(dir.getCos(), dir.getSin());
    Translation2d objField = cameraPose.getTranslation().plus(unit.times(latestDistanceMeters));

    Translation2d robotToObj = objField.minus(robotPose.getTranslation());
    Translation2d robotToObjUnit = robotToObj.div(robotToObj.getNorm() + 1e-9);

    Translation2d driveVelocity = robotToObjUnit.times(forwardCommand);

    drive.setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(),
            driveVelocity.getY(),
            angularCommand,
            robotPose.getRotation()));

    prevTxRadians = latestTxRadians;

    Logger.recordOutput("AimToObject/txRad", latestTxRadians);
    Logger.recordOutput("AimToObject/distance", latestDistanceMeters);
    Logger.recordOutput("AimToObject/forwardCommand", forwardCommand);
    Logger.recordOutput("AimToObject/angularCommand", angularCommand);
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
