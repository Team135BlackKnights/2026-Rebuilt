package frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.Turret.azimuth.AzimuthIOInputsAutoLogged;
import frc.robot.subsystems.Turret.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Turret.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.Turret.ShotCalculator.ShotProfile;
import frc.robot.subsystems.Turret.azimuth.AzimuthIO;
import frc.robot.subsystems.Turret.flywheel.FlywheelIO;
import frc.robot.subsystems.Turret.hood.HoodIO;
import frc.robot.subsystems.Turret.kickup.Kickup;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.selfCheck.SelfChecking;

import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedBoolean;

public class Turret extends SubsystemChecker {
  // Azimuth setPID(p,i,d,ks,kv,ka,velMax,accelMax)
  private final LoggableTunedNumber azimuth_kP;
  private final LoggableTunedNumber azimuth_kI;
  private final LoggableTunedNumber azimuth_kD;
  private final LoggableTunedNumber azimuth_kS;
  private final LoggableTunedNumber azimuth_kV;
  private final LoggableTunedNumber azimuth_kA;
  private final LoggableTunedNumber azimuth_velMax;
  private final LoggableTunedNumber azimuth_accelMax;
  private final LoggableTunedNumber azimuth_ramp;

  // Flywheel setPID(p,d,ks,kv)
  private final LoggableTunedNumber flywheel_kP;
  private final LoggableTunedNumber flywheel_kD;
  private final LoggableTunedNumber flywheel_kS;
  private final LoggableTunedNumber flywheel_kV;
  private final LoggableTunedNumber flywheel_kA;
  private final LoggableTunedNumber flywheel_ramp;
  private final LoggableTunedNumber flywheel_idle;

  // Hood setPID(p,d,ks,kv)
  private final LoggableTunedNumber hood_kP;
  private final LoggableTunedNumber hood_kD;
  private final LoggableTunedNumber hood_kS;
  private final LoggableTunedNumber hood_kV;
  // Other tunables
  private final LoggableTunedNumber offsetRPM;
  private final LoggableTunedNumber offsetHoodAngle;

  private final LoggableTunedNumber aimToleranceRads;
  private final LoggableTunedNumber hoodToleranceRads;
  private final LoggableTunedNumber flywheelToleranceRadsPerSec;
  private final LoggableTunedNumber turretAngleSnapDeadbandDeg;
  private final LoggableTunedNumber turretAngleSnapAlpha;

  // Shots

  private final LoggableTunedNumber shot_HUB_TOP_CENTER_RPM;
  private final LoggableTunedNumber shot_HUB_TOP_CENTER_HOOD_DEG;

  private final AzimuthIO azimuthIO;
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;
  private final Kickup kickup;
  private final String name;

  private final Transform2d robotToTurret;

  private double distanceOffset = 0.0;
  private final AzimuthIOInputsAutoLogged azimuthInputs = new AzimuthIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  public enum Goal {
    AIMING, // hub top center + pre-spin
    SHOOTING, // hub top center using HUB profile
    SHOOTING_CUSTOM,
    SHOOTING_FROM_HUB,
    JACKHAMMER,
    IDLE,
    TUNING_FLYWHEEL,
    TUNING_AZIMUTH,
    TUNING_HOOD,
    TUNING_SHOT // Sets RPM + hood from tuning numbers, for shot calibration
  }

  public enum PresetTarget {
    HUB_TOP_CENTER,
    LEFT_TRENCH_CENTER,
    RIGHT_TRENCH_CENTER,
    OVER_NEUTRAL_ZONE,
  }

  private Goal goal = Goal.IDLE;
  private Goal lastGoal = Goal.IDLE;
  private Goal previousControlGoal = Goal.IDLE;
  private double kickupWaitStartSec = Double.NaN;

  private PresetTarget activePreset = PresetTarget.HUB_TOP_CENTER;

  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  private ShotProfile profile = ShotCalculator.HUB_PROFILE;
  private Translation2d target = new Translation2d();

  private double desiredTurretRads = 0.0;
  private double lastTurretRads = 0.0;
  private double filteredTurretRads = Double.NaN;
  private double desiredHoodRads = 0.0;
  private double desiredFlywheelRadsPerSec = 0.0;
  private boolean backupRobotAimingEnabled = false;
  private double fixedShotDistanceOverrideMeters = Double.NaN;

  private final LoggableTunedNumber tuning_RPM;
  private final LoggableTunedNumber tuning_hoodDeg;
  private final LoggableTunedNumber tuning_TOF;
  private final LoggableTunedBoolean tuning_logFlag;
  private final List<String> loggedShots = new ArrayList<>();
  private boolean lastLogFlag = false;
  private boolean canChangeGoal = true;
  private boolean trenchHoodLock = false;
  private boolean manualHoodRezeroHeld = false;
  private boolean hoodForcedDownLastLoop = false;
  private double lastAutoRezeroSec = Double.NEGATIVE_INFINITY;
  private double lastManualRezeroSec = Double.NEGATIVE_INFINITY;
  private Goal jackhammerBaseGoal = Goal.IDLE;
  private boolean flywheelOutputSuppressed = false;

  private static final double SAFE_HOOD_DOWN_RADS = Units.degreesToRadians(13.0);
  private static final double SAFE_TRENCH_HOOD_RADS = Units.degreesToRadians(31.0);
  private static final double AUTO_REZERO_INTERVAL_SEC = 2.5;
  private static final double MANUAL_REZERO_INTERVAL_SEC = 1;

  public Turret(
      AzimuthIO azimuthIO,
      FlywheelIO flywheelIO,
      HoodIO hoodIO,
      Kickup kickup,
      Transform2d robotToTurret,
      String name) {
    this.azimuthIO = azimuthIO;
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    this.kickup = kickup;
    this.robotToTurret = robotToTurret;
    this.name = name;
    if ("LeftTurret".equals(name)) {
      azimuth_kP = new LoggableTunedNumber(name + "/Azimuth/kP", 6.0, TuningConstants.isTuningShooter); // 75
      azimuth_kI = new LoggableTunedNumber(name + "/Azimuth/kI", 0.0, TuningConstants.isTuningShooter);
      azimuth_kD = new LoggableTunedNumber(name + "/Azimuth/kD", 0.5, TuningConstants.isTuningShooter); // .25
      azimuth_kS = new LoggableTunedNumber(name + "/Azimuth/kS", 0.0, TuningConstants.isTuningShooter);
      azimuth_kV = new LoggableTunedNumber(name + "/Azimuth/kV", 0.0, TuningConstants.isTuningShooter);
      azimuth_kA = new LoggableTunedNumber(name + "/Azimuth/kA", 0.0, TuningConstants.isTuningShooter);
      azimuth_velMax = new LoggableTunedNumber(name + "/Azimuth/velMaxRadPerSec", 30, TuningConstants.isTuningShooter);
      azimuth_accelMax = new LoggableTunedNumber(name + "/Azimuth/accelMaxRadPerSec2", 100.0,
          TuningConstants.isTuningShooter);
      azimuth_ramp = new LoggableTunedNumber(name + "/Azimuth/ramp", 0.1, TuningConstants.isTuningShooter);

      flywheel_kP = new LoggableTunedNumber(name + "/Flywheel/kP", 0.098, TuningConstants.isTuningShooter); // 3
      flywheel_kD = new LoggableTunedNumber(name + "/Flywheel/kD", 0.0, TuningConstants.isTuningShooter);
      flywheel_kS = new LoggableTunedNumber(name + "/Flywheel/kS", 0.0, TuningConstants.isTuningShooter);
      flywheel_kV = new LoggableTunedNumber(name + "/Flywheel/kV", 0.115, TuningConstants.isTuningShooter); // .098
      flywheel_kA = new LoggableTunedNumber(name + "/Flywheel/kA", 0.0, TuningConstants.isTuningShooter);
      flywheel_ramp = new LoggableTunedNumber(name + "/Flywheel/Ramp", 0.25, TuningConstants.isTuningShooter);

      hood_kP = new LoggableTunedNumber(name + "/Hood/kP", 15, TuningConstants.isTuningShooter); // 15
      hood_kD = new LoggableTunedNumber(name + "/Hood/kD", 0.01, TuningConstants.isTuningShooter); // 1.5
      hood_kS = new LoggableTunedNumber(name + "/Hood/kS", 0.0, TuningConstants.isTuningShooter);
      hood_kV = new LoggableTunedNumber(name + "/Hood/kV", 0.0, TuningConstants.isTuningShooter);
      offsetRPM = new LoggableTunedNumber(name + "/Flywheel/offset", 0, TuningConstants.isTuningShooter);
      offsetHoodAngle = new LoggableTunedNumber(name + "/Hood/DONOTTOUCH", 0, TuningConstants.isTuningShooter);

    } else {
      // right turret
      azimuth_kP = new LoggableTunedNumber(name + "/Azimuth/kP", 6, TuningConstants.isTuningShooter); // 75
      azimuth_kI = new LoggableTunedNumber(name + "/Azimuth/kI", 0.0, TuningConstants.isTuningShooter);
      azimuth_kD = new LoggableTunedNumber(name + "/Azimuth/kD", 0.2, TuningConstants.isTuningShooter); // .25
      azimuth_kS = new LoggableTunedNumber(name + "/Azimuth/kS", 0.0, TuningConstants.isTuningShooter);
      azimuth_kV = new LoggableTunedNumber(name + "/Azimuth/kV", 0.0, TuningConstants.isTuningShooter);
      azimuth_kA = new LoggableTunedNumber(name + "/Azimuth/kA", 0.0, TuningConstants.isTuningShooter);
      azimuth_velMax = new LoggableTunedNumber(name + "/Azimuth/velMaxRadPerSec", 30, TuningConstants.isTuningShooter);
      azimuth_accelMax = new LoggableTunedNumber(name + "/Azimuth/accelMaxRadPerSec2", 150.0,
          TuningConstants.isTuningShooter);
      azimuth_ramp = new LoggableTunedNumber(name + "/Azimuth/ramp", 0.1, TuningConstants.isTuningShooter);

      flywheel_kP = new LoggableTunedNumber(name + "/Flywheel/kP", 0.098, TuningConstants.isTuningShooter); // 3
      flywheel_kD = new LoggableTunedNumber(name + "/Flywheel/kD", 0.0, TuningConstants.isTuningShooter);
      flywheel_kS = new LoggableTunedNumber(name + "/Flywheel/kS", 0.0, TuningConstants.isTuningShooter);
      flywheel_kV = new LoggableTunedNumber(name + "/Flywheel/kV", 0.115, TuningConstants.isTuningShooter); // .098
      flywheel_kA = new LoggableTunedNumber(name + "/Flywheel/kA", 0.0, TuningConstants.isTuningShooter);
      flywheel_ramp = new LoggableTunedNumber(name + "/Flywheel/Ramp", 0.25, TuningConstants.isTuningShooter);
      offsetRPM = new LoggableTunedNumber(name + "/Flywheel/Offset", 0, TuningConstants.isTuningShooter);
      offsetHoodAngle = new LoggableTunedNumber(name + "/Hood/DONOTTOUCH", 2.5, TuningConstants.isTuningShooter);

      hood_kP = new LoggableTunedNumber(name + "/Hood/kP", 15, TuningConstants.isTuningShooter); // 15
      hood_kD = new LoggableTunedNumber(name + "/Hood/kD", .01, TuningConstants.isTuningShooter); // 1.5
      hood_kS = new LoggableTunedNumber(name + "/Hood/kS", 0.0, TuningConstants.isTuningShooter);
      hood_kV = new LoggableTunedNumber(name + "/Hood/kV", 0.0, TuningConstants.isTuningShooter);

    }
    flywheel_idle = new LoggableTunedNumber(name + "/Flywheel/AimingSpeedRPM", 3000, TuningConstants.isTuningShooter);
    aimToleranceRads = new LoggableTunedNumber(name + "/Tolerance/AimRads", Math.toRadians(5),
        TuningConstants.isTuningShooter);
    hoodToleranceRads = new LoggableTunedNumber(name + "/Tolerance/HoodRads", Math.toRadians(2),
        TuningConstants.isTuningShooter);
    flywheelToleranceRadsPerSec = new LoggableTunedNumber(name + "/Tolerance/FlywheelRadsPerSec",
        Units.rotationsPerMinuteToRadiansPerSecond(1500), TuningConstants.isTuningShooter);
    turretAngleSnapDeadbandDeg = new LoggableTunedNumber(name + "/Turret/SnapDeadbandDeg", 1.0,
        TuningConstants.isTuningShooter);
    turretAngleSnapAlpha = new LoggableTunedNumber(name + "/Turret/SnapAlpha", 0.35, TuningConstants.isTuningShooter);
    shot_HUB_TOP_CENTER_RPM = new LoggableTunedNumber(name + "/Shot/HUB_TOP_CENTER_RPM", 3500,
        TuningConstants.isTuningShooter);
    shot_HUB_TOP_CENTER_HOOD_DEG = new LoggableTunedNumber(name + "/Shot/HUB_TOP_CENTER_HOOD_DEG", 13,
        TuningConstants.isTuningShooter);

    // Apply initial PIDs once
    applyAllPIDs();

    // Initialize targets
    target = getPresetTarget2d(PresetTarget.HUB_TOP_CENTER);

    tuning_RPM = new LoggableTunedNumber(name + "/ShotTuning/RPM", 0.0, TuningConstants.isTuningShooter);
    tuning_hoodDeg = new LoggableTunedNumber(name + "/ShotTuning/HoodDeg", 0.0, TuningConstants.isTuningShooter);
    tuning_TOF = new LoggableTunedNumber(name + "/ShotTuning/TOF_Sec", 0.5, TuningConstants.isTuningShooter);
    tuning_logFlag = new LoggableTunedBoolean(name + "/ShotTuning/LogShot", false, TuningConstants.isTuningShooter);
  }

  public void setGoal(Goal goal) {
    if (goal != null && canChangeGoal) {
      if (goal == Goal.JACKHAMMER) {
        if (this.goal != Goal.JACKHAMMER) {
          jackhammerBaseGoal = this.goal;
        }
      } else {
        jackhammerBaseGoal = goal;
      }
      this.goal = goal;
      // When entering a shooting-style goal, cancel any in-progress hood zeroing
      // immediately
      // so the hood can move to the shooting angle without waiting for zero to
      // finish.
      if (isShootLikeGoal(getShooterControlGoal())) {
        hoodIO.cancelZero();
      }
    }
  }

  public Goal getGoal() {
    return goal;
  }

  public boolean isShotModeActive() {
    return switch (goal) {
      case SHOOTING, SHOOTING_CUSTOM, SHOOTING_FROM_HUB -> true;
      default -> false;
    };
  }

  public void setPresetTarget(PresetTarget preset) {
    if (preset == null)
      return;
    this.activePreset = preset;
    this.target = getPresetTarget2d(preset);
    this.profile = getPresetTargetProfile(preset);
  }

  public PresetTarget getPresetTarget() {
    return activePreset;
  }

  public void setBackupRobotAimingEnabled(boolean enabled) {
    backupRobotAimingEnabled = enabled;
  }

  public void setFixedShotDistanceOverrideMeters(double distanceMeters) {
    fixedShotDistanceOverrideMeters = Math.max(0.0, distanceMeters);
  }

  public void clearFixedShotDistanceOverride() {
    fixedShotDistanceOverrideMeters = Double.NaN;
  }

  public Translation2d getActiveTargetPosition() {
    return activePreset == PresetTarget.OVER_NEUTRAL_ZONE ? getPresetTarget2d(PresetTarget.OVER_NEUTRAL_ZONE) : target;
  }

  private void applyAllPIDs() {
    azimuthIO.setPID(
        azimuth_kP.get(), azimuth_kI.get(), azimuth_kD.get(),
        azimuth_kS.get(), azimuth_kV.get(), azimuth_kA.get(),
        azimuth_velMax.get(), azimuth_accelMax.get(), azimuth_ramp.get());

    flywheelIO.setPID(
        flywheel_kP.get(), flywheel_kD.get(),
        flywheel_kS.get(), flywheel_kV.get(), flywheel_kA.get());
    flywheelIO.setRamp(flywheel_ramp.get());
    hoodIO.setPID(
        hood_kP.get(), hood_kD.get(),
        hood_kS.get(), hood_kV.get());

  }

  private void updateTunablePIDs() {
    LoggableTunedNumber.ifChanged(
        hashCode(),
        () -> azimuthIO.setPID(
            azimuth_kP.get(), azimuth_kI.get(), azimuth_kD.get(),
            azimuth_kS.get(), azimuth_kV.get(), azimuth_kA.get(),
            azimuth_velMax.get(), azimuth_accelMax.get(), azimuth_ramp.get()),
        azimuth_kP, azimuth_kI, azimuth_kD, azimuth_kS, azimuth_kV, azimuth_kA, azimuth_velMax, azimuth_accelMax,
        azimuth_ramp);

    LoggableTunedNumber.ifChanged(
        hashCode(),
        () -> flywheelIO.setPID(
            flywheel_kP.get(), flywheel_kD.get(),
            flywheel_kS.get(), flywheel_kV.get(), flywheel_kA.get()),
        flywheel_kP, flywheel_kD, flywheel_kS, flywheel_kV, flywheel_kA);

    LoggableTunedNumber.ifChanged(
        hashCode(),
        () -> flywheelIO.setRamp(flywheel_ramp.get()),
        flywheel_ramp);
    LoggableTunedNumber.ifChanged(
        hashCode(),
        () -> hoodIO.setPID(
            hood_kP.get(), hood_kD.get(),
            hood_kS.get(), hood_kV.get()),
        hood_kP, hood_kD, hood_kS, hood_kV);
  }

  private boolean isAzimuthConnected() {
    return azimuthInputs.motorConnected && azimuthInputs.zeroed;
  }

  private boolean isFlywheelConnected() {
    return flywheelInputs.connected;
  }

  private boolean isHoodConnected() {
    return hoodInputs.connected;
  }

  private double turretAngleErrorRads() {
    return MathUtil.angleModulus(applyTurretAngleFilter(desiredTurretRads) - azimuthInputs.turretPositionRads);
  }

  private double backupRobotHeadingErrorRads() {
    Rotation2d desiredRobotHeading = GeomUtil.rotationFromCurrentToTarget(
        RobotContainer.drivetrainS.getLookAheadPose().getTranslation(),
        RobotContainer.getBackupAimTargetTranslation(),
        GeomUtil.ApproachDirection.FRONT);
    return MathUtil.angleModulus(
        desiredRobotHeading.minus(RobotContainer.drivetrainS.getLookAheadPose().getRotation()).getRadians());
  }

  private double effectiveAimErrorRads() {
    return backupRobotAimingEnabled ? backupRobotHeadingErrorRads() : turretAngleErrorRads();
  }

  private double applyTurretAngleFilter(double targetRads) {
    // Initialize filter state on first use
    if (!Double.isFinite(filteredTurretRads)) {
      filteredTurretRads = targetRads;
      return filteredTurretRads;
    }

    double deadband = Units.degreesToRadians(turretAngleSnapDeadbandDeg.get());
    double alpha = MathUtil.clamp(turretAngleSnapAlpha.get(), 0.0, 1.0);

    double error = MathUtil.angleModulus(targetRads - filteredTurretRads);
    if (Math.abs(error) < deadband) {
      return filteredTurretRads;
    }

    filteredTurretRads = MathUtil.angleModulus(filteredTurretRads + alpha * error);
    return filteredTurretRads;
  }

  private void commandTurretPosition(double targetRads) {
    desiredTurretRads = targetRads;
    double filtered = applyTurretAngleFilter(targetRads);
    azimuthIO.setDesiredPosition(filtered);
    Logger.recordOutput(name + "/Setpoints/TurretRadsFiltered", filtered);
  }

  private void commandTurretPositionForMode(double targetRads) {
    commandTurretPosition(backupRobotAimingEnabled ? 0.0 : targetRads);
  }

  private void commandFlywheelVelocity(double velocityRadsPerSec) {
    if (flywheelOutputSuppressed) {
      flywheelIO.stop();
      return;
    }
    flywheelIO.setVelocity(velocityRadsPerSec + Units.rotationsPerMinuteToRadiansPerSecond(offsetRPM.get()));
  }

  private boolean hasFixedShotDistanceOverride() {
    return Double.isFinite(fixedShotDistanceOverrideMeters);
  }

  private void applyFixedShotDistanceOverride() {
    var fixedShotParameters = shotCalculator.getFixedDistanceShotParameters(robotToTurret, profile,
        fixedShotDistanceOverrideMeters);
    desiredHoodRads = fixedShotParameters.hoodAngle();
    desiredFlywheelRadsPerSec = fixedShotParameters.flywheelSpeed();
  }

  public boolean atAimAngle() {
    return isAzimuthConnected() && Math.abs(effectiveAimErrorRads()) < aimToleranceRads.get();
  }

  public void forceShot(double angle, double hood, double rpm) {
    goal = Goal.SHOOTING_CUSTOM;
    jackhammerBaseGoal = goal;
    desiredFlywheelRadsPerSec = rpm * 2.0 * Math.PI / 60.0;
    desiredTurretRads = angle;
    desiredHoodRads = hood;

  }

  public void setCharRPM(double rpm) {
    goal = Goal.TUNING_FLYWHEEL;
    jackhammerBaseGoal = goal;
    desiredFlywheelRadsPerSec = rpm * 2.0 * Math.PI / 60.0;
    commandFlywheelVelocity(desiredFlywheelRadsPerSec);
  }

  public void setCharTurretPos(double radians) {
    goal = Goal.TUNING_AZIMUTH;
    desiredTurretRads = radians;
    commandTurretPosition(desiredTurretRads);
  }

  public void setCharHoodPos(double radians) {
    goal = Goal.TUNING_HOOD;
    jackhammerBaseGoal = goal;
    desiredHoodRads = radians;

  }

  public double getCharTurretPos() {
    return azimuthInputs.turretPositionRads;
  }

  public double getCharHoodPos() {
    return hoodInputs.positionRads;
  }

  public double getCharFlywheelRPM() {
    return flywheelInputs.velocityRadsPerSec * 60.0 / (2.0 * Math.PI);
  }

  public Transform2d getRobotToTurret() {
    return robotToTurret;
  }

  public void enterShotTuning() {
    goal = Goal.TUNING_SHOT;
    jackhammerBaseGoal = goal;
    canChangeGoal = false;
    kickupWaitStartSec = Double.NaN;
  }

  public boolean isShotTuningActive() {
    return goal == Goal.TUNING_SHOT && !canChangeGoal;
  }

  public void clearLoggedShots() {
    loggedShots.clear();
    canChangeGoal = true;
    kickupWaitStartSec = Double.NaN;
  }

  public double getCharTurretVelocity() {
    return azimuthInputs.turretVelocityRadsPerSec;
  }

  public double getCharHoodVelocity() {
    return hoodInputs.velocityRadsPerSec;
  }

  public double getCharFlywheelVelocity() {
    return flywheelInputs.accelRadsPerSec2;
  }

  public double hoodAngle() {
    return hoodInputs.positionRads;
  }

  public void offsetDistance(double offset) {
    distanceOffset += offset;
  }

  /** Start zeroing the hood (delegates to underlying IO). */
  public void zeroHood() {
    hoodIO.zero();
  }

  public void rezeroAzimuth() {
    azimuthIO.requestRezero();
  }

  public void disAllowHood() {
    setManualHoodRezeroHeld(true);
  }

  public void allowHood() {
    setManualHoodRezeroHeld(false);
  }

  public void setTrenchHoodLock(boolean locked) {
    trenchHoodLock = locked;
  }

  public void setManualHoodRezeroHeld(boolean held) {
    manualHoodRezeroHeld = held;
  }

  public void setFlywheelOutputSuppressed(boolean suppressed) {
    flywheelOutputSuppressed = suppressed;
  }

  public boolean isHoodAboveDegrees(double degrees) {
    return desiredHoodRads > Units.degreesToRadians(degrees) && !isHoodForcedDown(getShooterControlGoal());
  }

  public double turretAngle() {
    return azimuthInputs.turretPositionRads;
  }

  public boolean atShootSetpoints() {
    return isAzimuthConnected()
        && isHoodConnected()
        && isFlywheelConnected()
        // && !isHoodForcedDown(getShooterControlGoal())
        && Math.abs(effectiveAimErrorRads()) < aimToleranceRads.get()
        && Math.abs(hoodInputs.positionRads - desiredHoodRads
            - Units.degreesToRadians(offsetHoodAngle.get())) < hoodToleranceRads.get()
        && Math.abs(flywheelInputs.velocityRadsPerSec - desiredFlywheelRadsPerSec
            + Units.rotationsPerMinuteToRadiansPerSecond(offsetRPM.get())) < flywheelToleranceRadsPerSec.get();
  }

  @Override
  public void periodic() {
    if (azimuthInputs.motorConnected && (azimuthInputs.zeroed || azimuthIO.wantsZeroing())) {
      azimuthIO.enable();
    } else {
      azimuthIO.disable();
    }
    azimuthIO.updateInputs(azimuthInputs);
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);

    Logger.processInputs(name + "/Azimuth", azimuthInputs);
    Logger.processInputs(name + "/Flywheel", flywheelInputs);
    Logger.processInputs(name + "/Hood", hoodInputs);
    updateTunablePIDs(); // way cleaner than last year lol

    /*
     * if (DriverStation.isDisabled()) {
     * goal = Goal.IDLE;
     * }
     */

    shotCalculator.clearShootingParameters();

    Goal controlGoal = getShooterControlGoal();

    switch (controlGoal) {
      case IDLE -> {
        if (activePreset == PresetTarget.OVER_NEUTRAL_ZONE) {
          target = getPresetTarget2d(PresetTarget.OVER_NEUTRAL_ZONE); // update for the robot pos, since target moves
        }
        desiredFlywheelRadsPerSec = 0.0;

        if (backupRobotAimingEnabled) {
          commandTurretPosition(0.0);
        } else if (!azimuthInputs.zeroed || azimuthIO.wantsZeroing()) {
          desiredTurretRads = lastTurretRads;
        } else {
          azimuthIO.stop();
          desiredTurretRads = lastTurretRads;
        }
        flywheelIO.stop();
        desiredHoodRads = SAFE_HOOD_DOWN_RADS;
      }

      case AIMING -> {
        // Use HUB profile for turret + hood, but override flywheel speed to a constant
        // pre-spin
        if (activePreset == PresetTarget.OVER_NEUTRAL_ZONE) {
          target = getPresetTarget2d(PresetTarget.OVER_NEUTRAL_ZONE); // update for the robot pos, since target moves
        }
        var params = shotCalculator.getParameters(target, robotToTurret, ShotCalculator.HUB_PROFILE, distanceOffset);

        desiredTurretRads = params.turretAngle().getRadians();
        desiredHoodRads = SAFE_HOOD_DOWN_RADS;
        desiredFlywheelRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flywheel_idle.get());
        commandTurretPositionForMode(desiredTurretRads);

        commandFlywheelVelocity(desiredFlywheelRadsPerSec);
      }

      case SHOOTING -> {
        if (activePreset == PresetTarget.OVER_NEUTRAL_ZONE) {
          target = getPresetTarget2d(PresetTarget.OVER_NEUTRAL_ZONE); // update for the robot pos, since target moves
        }
        var params = shotCalculator.getParameters(target, robotToTurret, profile, distanceOffset);

        desiredTurretRads = params.turretAngle().getRadians();
        desiredHoodRads = params.hoodAngle();
        desiredFlywheelRadsPerSec = params.flywheelSpeed();
        if (hasFixedShotDistanceOverride()) {
          applyFixedShotDistanceOverride();
        }
        commandTurretPositionForMode(desiredTurretRads);

        commandFlywheelVelocity(desiredFlywheelRadsPerSec);
      }
      case SHOOTING_CUSTOM -> {
        commandTurretPositionForMode(desiredTurretRads);
        commandFlywheelVelocity(desiredFlywheelRadsPerSec);
      }
      case SHOOTING_FROM_HUB -> {
        var params = shotCalculator.getParameters(target, robotToTurret, profile, distanceOffset);
        desiredTurretRads = params.turretAngle().getRadians();
        if (hasFixedShotDistanceOverride()) {
          applyFixedShotDistanceOverride();
        } else {
          desiredHoodRads = Units.degreesToRadians(shot_HUB_TOP_CENTER_HOOD_DEG.get());
          desiredFlywheelRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(shot_HUB_TOP_CENTER_RPM.get());
        }
        commandTurretPositionForMode(desiredTurretRads);
        commandFlywheelVelocity(desiredFlywheelRadsPerSec);
      }

      case TUNING_FLYWHEEL -> {
        commandFlywheelVelocity(desiredFlywheelRadsPerSec);
      }
      case TUNING_AZIMUTH -> {
        commandTurretPositionForMode(desiredTurretRads);
      }
      case TUNING_HOOD -> {
      }
      case TUNING_SHOT -> {
        desiredFlywheelRadsPerSec = tuning_RPM.get() * 2.0 * Math.PI / 60.0;
        desiredHoodRads = Math.toRadians(tuning_hoodDeg.get());
        // just grab the shot angle from tuning
        setPresetTarget(PresetTarget.HUB_TOP_CENTER);
        var params = shotCalculator.getParameters(target, robotToTurret, profile, distanceOffset);
        desiredTurretRads = params.turretAngle().getRadians();
        commandFlywheelVelocity(desiredFlywheelRadsPerSec);
        commandTurretPositionForMode(desiredTurretRads);
      }
      case JACKHAMMER -> {
      }
    }

    // Decide kickup goal after updating shooter/turret/hood setpoints for this
    // loop.
    Kickup.Goal kickupGoal = getKickupGoal(controlGoal, previousControlGoal);
    kickup.setGoal(kickupGoal);
    kickup.periodic();
    if (goal != lastGoal) {
      shotCalculator.clearShootingParameters();
      lastGoal = goal;
    }
    applyHoodSafetyControl(controlGoal);
    lastTurretRads = desiredTurretRads;

    boolean logNow = tuning_logFlag.get();
    if (logNow && !lastLogFlag) {
      Pose2d turretPose = RobotContainer.drivetrainS.getPose().transformBy(robotToTurret);
      double distToTarget = target.getDistance(turretPose.getTranslation()); // OUR FIELD iS .08M OFF (if we had 1, .92)
      double rpm = tuning_RPM.get();// flywheelInputs.velocityRadsPerSec * 60.0 / (2.0 * Math.PI);
      double hoodDeg = Math.toRadians(tuning_hoodDeg.get());// Math.toDegrees(hoodInputs.positionRads);
      double tof = tuning_TOF.get();
      String entry = String.format("dist=%.3fm  rpm=%.1f  hood=%.2frad  tof=%.3fs", distToTarget, rpm, hoodDeg, tof);
      loggedShots.add(entry);
      tuning_logFlag.set(false);
    }
    lastLogFlag = logNow;
    Logger.recordOutput(name + "/ShotTuning/LoggedShots", loggedShots.toArray(new String[0]));
    Logger.recordOutput(name + "/ShotTuning/ShotCount", loggedShots.size());
    // Live distance for tuning reference
    Pose2d currentTurretPose = RobotContainer.drivetrainS.getPose().transformBy(robotToTurret);
    Logger.recordOutput(name + "/ShotTuning/DistToTargetM",
        target.getDistance(currentTurretPose.getTranslation()));
    Logger.recordOutput(name + "/Goal", goal.toString());
    Logger.recordOutput(name + "/ControlGoal", controlGoal.toString());
    Logger.recordOutput("SuperStructure/" + name + "/Goal", goal.toString());
    Logger.recordOutput("SuperStructure/" + name + "/TargetName", activePreset.toString());
    Logger.recordOutput("SuperStructure/" + name + "/TargetPos", new Pose2d(target, new Rotation2d()));
    Logger.recordOutput(name + "/Preset", activePreset.toString());

    Logger.recordOutput(name + "/Targets/Target2d", target);

    Logger.recordOutput(name + "/Setpoints/TurretRads", desiredTurretRads);
    Logger.recordOutput(name + "/Setpoints/HoodRads", desiredHoodRads + Units.degreesToRadians(offsetHoodAngle.get()));
    Logger.recordOutput(name + "/Setpoints/FlywheelRadsPerSec",
        desiredFlywheelRadsPerSec + Units.rotationsPerMinuteToRadiansPerSecond(offsetRPM.get()));

    Logger.recordOutput(name + "/Errors/TurretRads", turretAngleErrorRads());
    Logger.recordOutput(name + "/Azimuth/ReferenceEncoderConnected", azimuthInputs.referenceEncoderConnected);
    Logger.recordOutput(name + "/Azimuth/Zeroed", azimuthInputs.zeroed);
    Logger.recordOutput(name + "/Azimuth/WantsZeroing", azimuthIO.wantsZeroing());
    Logger.recordOutput(name + "/BackupRobotAimingEnabled", backupRobotAimingEnabled);
    Logger.recordOutput(name + "/FlywheelOutputSuppressed", flywheelOutputSuppressed);
    Logger.recordOutput(name + "/BackupRobotHeadingErrorRads",
        backupRobotAimingEnabled ? backupRobotHeadingErrorRads() : 0.0);
    Logger.recordOutput(name + "/FixedShotDistanceOverrideMeters", fixedShotDistanceOverrideMeters);
    Logger.recordOutput(name + "/AtAimAngle", atAimAngle());
    Logger.recordOutput(name + "/AtShootSetpoints", atShootSetpoints());
    Logger.recordOutput(name + "/KickupGoal", kickupGoal.toString());
    Logger.recordOutput("SuperStructure/" + name + "/KickupGoal", kickupGoal.toString());
    previousControlGoal = controlGoal;
  }

  private Goal getShooterControlGoal() {
    if (goal != Goal.JACKHAMMER) {
      return goal;
    }
    return jackhammerBaseGoal == Goal.JACKHAMMER ? Goal.IDLE : jackhammerBaseGoal;
  }

  private Kickup.Goal getKickupGoal(Goal controlGoal, Goal previousControlGoal) {
    if (DriverStation.isDisabled()) {
      return Kickup.Goal.IDLING;
    }
    if (!azimuthInputs.zeroed || azimuthIO.wantsZeroing()) {
      return Kickup.Goal.IDLING;
    }
    if (!isAzimuthConnected()){
      return Kickup.Goal.VOMITING;
    }
    if (controlGoal == Goal.JACKHAMMER || goal == Goal.JACKHAMMER) {
      return Kickup.Goal.JACKHAMMER;
    }

    if (isShotTuningActive()) {
      kickupWaitStartSec = Double.NaN;
      return Kickup.Goal.TESTING;
    }

    if (!isShootLikeGoal(controlGoal)) {
      kickupWaitStartSec = Double.NaN;
      return Kickup.Goal.IDLING;
    }

    boolean enteredShootLike = !isShootLikeGoal(previousControlGoal);
    if (enteredShootLike) {
      kickupWaitStartSec = Timer.getFPGATimestamp();

      // If we just came from JACKHAMMER, satisfy the wait immediately.
      if (previousControlGoal == Goal.JACKHAMMER) {
        double waitSec = switch (controlGoal) {
          case SHOOTING -> RobotContainer.shootMaxWaitSec.get();
          case SHOOTING_CUSTOM, SHOOTING_FROM_HUB -> RobotContainer.shootMaxWaitSec.get();
          default -> 0.0;
        };
        kickupWaitStartSec -= waitSec;
      }
    }

    // SHOOTING can bypass timeout immediately once fully at setpoints.
    if ((controlGoal == Goal.SHOOTING && atShootSetpoints()) || RobotContainer.forceKickup) {
      return Kickup.Goal.SHOOTING;
    }

    double waitSec = switch (controlGoal) {
      case SHOOTING -> RobotContainer.shootMaxWaitSec.get();
      case SHOOTING_CUSTOM, SHOOTING_FROM_HUB -> RobotContainer.shootMaxWaitSec.get();
      default -> 0.0;
    };

    if (!Double.isFinite(kickupWaitStartSec)) {
      kickupWaitStartSec = Timer.getFPGATimestamp();
    }

    double elapsedSec = Timer.getFPGATimestamp() - kickupWaitStartSec;
    return elapsedSec >= waitSec ? Kickup.Goal.SHOOTING : Kickup.Goal.IDLING;
  }

  private boolean isShootLikeGoal(Goal goal) {
    return switch (goal) {
      case SHOOTING, SHOOTING_CUSTOM, SHOOTING_FROM_HUB -> true;
      default -> false;
    };
  }

  private boolean isHoodForcedDown(Goal controlGoal) {
    return manualHoodRezeroHeld || !isShootLikeGoal(controlGoal);
  }

  private void applyHoodSafetyControl(Goal controlGoal) {
    double commandedHoodRads = desiredHoodRads + Units.degreesToRadians(offsetHoodAngle.get());
    if (controlGoal == Goal.TUNING_HOOD || controlGoal == Goal.TUNING_SHOT) {
      hoodIO.setPosition(commandedHoodRads);
      return;
    }
    boolean forceDown = isHoodForcedDown(controlGoal);
    boolean trenchClamped = trenchHoodLock && isShootLikeGoal(controlGoal) && commandedHoodRads > SAFE_TRENCH_HOOD_RADS;
    if (forceDown) {
      hoodIO.setPosition(SAFE_HOOD_DOWN_RADS);
      /*
       * if (DriverStation.isEnabled()) {
       * double now = Timer.getFPGATimestamp();
       * if (manualHoodRezeroHeld) {
       * if (!hoodForcedDownLastLoop || now - lastManualRezeroSec >=
       * MANUAL_REZERO_INTERVAL_SEC) {
       * hoodIO.zero();
       * lastManualRezeroSec = now;
       * }
       * } else if (!hoodForcedDownLastLoop || now - lastAutoRezeroSec >=
       * AUTO_REZERO_INTERVAL_SEC) {
       * hoodIO.zero();
       * System.out.println("Auto-rezeroing hood due to safety lock");
       * lastAutoRezeroSec = now;
       * }
       * }
       */
    } else if (trenchClamped) {
      hoodIO.setPosition(SAFE_TRENCH_HOOD_RADS);
    } else {
      hoodIO.setPosition(commandedHoodRads);
    }
    hoodForcedDownLastLoop = forceDown || trenchClamped;
    Logger.recordOutput(name + "/Hood/ForceDown", forceDown);
    Logger.recordOutput(name + "/Hood/TrenchClamped", trenchClamped);
    Logger.recordOutput(name + "/Hood/TrenchLock", trenchHoodLock);
    Logger.recordOutput(name + "/Hood/ManualRezeroHeld", manualHoodRezeroHeld);
  }

  private static Translation2d getPresetTarget2d(PresetTarget preset) {
    return switch (preset) {
      case HUB_TOP_CENTER -> {
        Translation3d p = FieldConstants.Hub.topCenterPoint;
        yield GeomUtil.apply(new Translation2d(p.getX(), p.getY()));
      }

      case LEFT_TRENCH_CENTER -> {
        Translation3d a = FieldConstants.LeftTrench.openingTopLeft;
        Translation3d b = FieldConstants.LeftTrench.openingTopRight;
        Translation2d center = new Translation2d((a.getX() + b.getX()) * 0.5, (a.getY() + b.getY()) * 0.5);
        yield GeomUtil.apply(center);
      }

      case RIGHT_TRENCH_CENTER -> {
        Translation3d a = FieldConstants.RightTrench.openingTopLeft;
        Translation3d b = FieldConstants.RightTrench.openingTopRight;
        Translation2d center = new Translation2d((a.getX() + b.getX()) * 0.5, (a.getY() + b.getY()) * 0.5);
        yield GeomUtil.apply(center);
      }

      case OVER_NEUTRAL_ZONE -> {
        // same Y, but in our area X.
        yield new Translation2d(GeomUtil.applyX(3), RobotContainer.drivetrainS.getPose().getY());
      }
    };
  }

  private static ShotProfile getPresetTargetProfile(PresetTarget preset) {
    return switch (preset) {
      case HUB_TOP_CENTER -> ShotCalculator.HUB_PROFILE;
      case LEFT_TRENCH_CENTER, RIGHT_TRENCH_CENTER -> ShotCalculator.TRENCH_PROFILE;
      case OVER_NEUTRAL_ZONE -> ShotCalculator.NEUTRAL_ZONE_PROFILE;
    };
  }

  public HashMap<String, Double> getTemps() {
    HashMap<String, Double> tempMap = new HashMap<>();
    tempMap.put("Azimuth", azimuthInputs.tempCelsius);
    tempMap.put("Flywheel", flywheelInputs.tempCelsius);
    tempMap.put("Hood", hoodInputs.tempCelsius);
    tempMap.put("Kickup", kickup.getTemps().values().stream().findFirst().orElse(0.0));
    return tempMap;
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    List<ParentDevice> orchestra = new ArrayList<>();
    List<SelfChecking> hardware = new ArrayList<>();
    hardware.addAll(azimuthIO.getSelfCheckingHardware());
    hardware.addAll(flywheelIO.getSelfCheckingHardware());
    hardware.addAll(hoodIO.getSelfCheckingHardware());
    hardware.addAll(kickup.getHardware());
    for (SelfChecking motor : hardware) {
      if (motor.getHardware() instanceof TalonFX) {
        orchestra.add((TalonFX) motor.getHardware());
      }
    }
    return orchestra;
  }

  @Override
  public double getCurrent() {
    return Math.abs(azimuthInputs.supplyCurrentAmps)
        + Math.abs(flywheelInputs.supplyCurrentAmps)
        + Math.abs(hoodInputs.supplyCurrentAmps)
        + Math.abs(kickup.getCurrent());
  }

  @Override
  public void setCurrentLimit(int amps) {
    azimuthIO.setCurrentLimit(amps);
    kickup.setCurrentLimit(amps);
  }

  public void setCurrentLimit(int azimuthAmps, int flywheelAmps, int hoodAmps, int kickupAmps) {
    azimuthIO.setCurrentLimit(azimuthAmps);
    flywheelIO.setCurrentLimit(flywheelAmps);
    hoodIO.setCurrentLimit(hoodAmps);
    kickup.setCurrentLimit(kickupAmps);
  }

  @Override
  protected Command systemCheckCommand() {
    return runOnce(() -> {
      // Simple check logic
      if (isAzimuthConnected() && isFlywheelConnected() && isHoodConnected() && kickup.isConnected()) {
        Logger.recordOutput(name + "/SystemCheck/Connected", "GOOD");
      } else {
        Logger.recordOutput(name + "/SystemCheck/Connected", "BAD");
      }
    }).withName(name + "SystemCheck");
  }
}
