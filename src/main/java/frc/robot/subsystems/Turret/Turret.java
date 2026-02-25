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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.Turret.azimuth.AzimuthIOInputsAutoLogged;
import frc.robot.subsystems.Turret.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.subsystems.Turret.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.Turret.ShotCalculator.ShotProfile;
import frc.robot.subsystems.Turret.azimuth.AzimuthIO;
import frc.robot.subsystems.Turret.flywheel.FlywheelIO;
import frc.robot.subsystems.Turret.hood.HoodIO;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.selfCheck.SelfChecking;

import frc.robot.utils.GeomUtil;

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

  // Hood setPID(p,d,ks,kv)
  private final LoggableTunedNumber hood_kP;
  private final LoggableTunedNumber hood_kD;
  private final LoggableTunedNumber hood_kS;
  private final LoggableTunedNumber hood_kV;
  // Other tunables
  private final LoggableTunedNumber aimingFlywheelSpeedRadsPerSec;

  private final LoggableTunedNumber aimToleranceRads;
  private final LoggableTunedNumber hoodToleranceRads;
  private final LoggableTunedNumber flywheelToleranceRadsPerSec;

  private final AzimuthIO azimuthIO;
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;
  private final String name;

  private final Transform2d robotToTurret;

  private final AzimuthIOInputsAutoLogged azimuthInputs = new AzimuthIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  public enum Goal {
    AIMING, // hub top center + pre-spin
    SHOOTING, // hub top center using HUB profile
    IDLE,
    TUNING_FLYWHEEL,
    TUNING_AZIMUTH,
    TUNING_HOOD
  }

  public enum PresetTarget {
    HUB_TOP_CENTER,
    LEFT_TRENCH_CENTER,
    RIGHT_TRENCH_CENTER,
    OVER_NEUTRAL_ZONE,
  }

  private Goal goal = Goal.IDLE;
  private Goal lastGoal = Goal.IDLE;

  private PresetTarget activePreset = PresetTarget.HUB_TOP_CENTER;

  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  private ShotProfile profile = ShotCalculator.HUB_PROFILE;
  private Translation2d target = new Translation2d();

  private double desiredTurretRads = 0.0;
  private double desiredHoodRads = 0.0;
  private double desiredFlywheelRadsPerSec = 0.0;

  public Turret(AzimuthIO azimuthIO, FlywheelIO flywheelIO, HoodIO hoodIO, Transform2d robotToTurret, String name) {
    this.azimuthIO = azimuthIO;
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    this.robotToTurret = robotToTurret;
    this.name = name;

    azimuth_kP = new LoggableTunedNumber(name + "/Azimuth/kP", 0.0, true); //75
    azimuth_kI = new LoggableTunedNumber(name + "/Azimuth/kI", 0.0, true);
    azimuth_kD = new LoggableTunedNumber(name + "/Azimuth/kD", 0.0, true); //.25
    azimuth_kS = new LoggableTunedNumber(name + "/Azimuth/kS", 0.0, true);
    azimuth_kV = new LoggableTunedNumber(name + "/Azimuth/kV", 0.0, true);
    azimuth_kA = new LoggableTunedNumber(name + "/Azimuth/kA", 0.0, true);
    azimuth_velMax = new LoggableTunedNumber(name + "/Azimuth/velMaxRadPerSec", 18.0, true);
    azimuth_accelMax = new LoggableTunedNumber(name + "/Azimuth/accelMaxRadPerSec2", 40.0, true);
    azimuth_ramp = new LoggableTunedNumber(name + "/Azimuth/ramp", 0.25, true);

    flywheel_kP = new LoggableTunedNumber(name + "/Flywheel/kP", 0, true); //3
    flywheel_kD = new LoggableTunedNumber(name + "/Flywheel/kD", 0.0, true);
    flywheel_kS = new LoggableTunedNumber(name + "/Flywheel/kS", 0.0, true);
    flywheel_kV = new LoggableTunedNumber(name + "/Flywheel/kV", 0.0, true); //.098
    flywheel_kA = new LoggableTunedNumber(name + "/Flywheel/kA", 0.0, true);
    flywheel_ramp = new LoggableTunedNumber(name + "/Flywheel/Ramp", 0.25, true);

    hood_kP = new LoggableTunedNumber(name + "/Hood/kP", 0, true); //15
    hood_kD = new LoggableTunedNumber(name + "/Hood/kD", 0, true); //1.5
    hood_kS = new LoggableTunedNumber(name + "/Hood/kS", 0.0, true);
    hood_kV = new LoggableTunedNumber(name + "/Hood/kV", 0.0, true);

    aimingFlywheelSpeedRadsPerSec = new LoggableTunedNumber(name + "/Aiming/FlywheelSpeedRadsPerSec", 125.0, true);

    aimToleranceRads = new LoggableTunedNumber(name + "/Tolerance/AimRads", Math.toRadians(1.5), true);
    hoodToleranceRads = new LoggableTunedNumber(name + "/Tolerance/HoodRads", Math.toRadians(1.0), true);
    flywheelToleranceRadsPerSec = new LoggableTunedNumber(name + "/Tolerance/FlywheelRadsPerSec", 10.0, true);

    // Apply initial PIDs once
    applyAllPIDs();

    // Initialize targets
    target = getPresetTarget2d(PresetTarget.HUB_TOP_CENTER);
  }

  public void setGoal(Goal goal) {
    if (goal != null)
      this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public void setPresetTarget(PresetTarget preset) {
    if (preset == null)
      return;
    this.activePreset = preset;
    this.target = getPresetTarget2d(preset);
    this.profile = getPresetTargetProfile(preset);
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
        azimuth_kP, azimuth_kI, azimuth_kD, azimuth_kS, azimuth_kV, azimuth_kA, azimuth_velMax, azimuth_accelMax, azimuth_ramp);

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
    return azimuthInputs.motorConnected && azimuthInputs.bothEncodersConnected;
  }

  private boolean isFlywheelConnected() {
    return flywheelInputs.connected;
  }

  private boolean isHoodConnected() {
    return hoodInputs.connected;
  }

  private double turretAngleErrorRads() {
    return MathUtil.angleModulus(desiredTurretRads - azimuthInputs.turretPositionRads);
  }

  public boolean atAimAngle() {
    return isAzimuthConnected() && Math.abs(turretAngleErrorRads()) < aimToleranceRads.get();
  }
  public void setCharRPM(double rpm) {
    goal = Goal.TUNING_FLYWHEEL;
    desiredFlywheelRadsPerSec = rpm * 2.0 * Math.PI / 60.0;
    flywheelIO.setVelocity(desiredFlywheelRadsPerSec);
  }
  public void setCharTurretPos(double radians) {
    goal = Goal.TUNING_AZIMUTH;
    desiredTurretRads = radians;
    azimuthIO.setDesiredPosition(desiredTurretRads);
  }
  public void setCharHoodPos(double radians) {
    //goal = Goal.TUNING_HOOD;
    desiredHoodRads = radians;
    hoodIO.setPosition(desiredHoodRads);
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

  public double turretAngle() {
    return azimuthInputs.turretPositionRads;
  }

  public boolean atShootSetpoints() {
    return isAzimuthConnected()
        && isHoodConnected()
        && isFlywheelConnected()
        && Math.abs(turretAngleErrorRads()) < aimToleranceRads.get()
        && Math.abs(hoodInputs.positionRads - desiredHoodRads) < hoodToleranceRads.get()
        && Math.abs(flywheelInputs.velocityRadsPerSec - desiredFlywheelRadsPerSec) < flywheelToleranceRadsPerSec.get();
  }

  @Override
  public void periodic() {
    azimuthIO.updateInputs(azimuthInputs);
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);

    Logger.processInputs(name + "/Azimuth", azimuthInputs);
    Logger.processInputs(name + "/Flywheel", flywheelInputs);
    Logger.processInputs(name + "/Hood", hoodInputs);
    updateTunablePIDs(); // way cleaner than last year lol

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;
    }

    shotCalculator.clearShootingParameters();

    if (goal != lastGoal) {
      shotCalculator.clearShootingParameters();
      lastGoal = goal;
    }

    switch (goal) {
      case IDLE -> {
        if (activePreset == PresetTarget.OVER_NEUTRAL_ZONE) {
          target = getPresetTarget2d(PresetTarget.OVER_NEUTRAL_ZONE); // update for the robot pos, since target moves
        }
        desiredFlywheelRadsPerSec = 0.0;

        azimuthIO.stop();
        hoodIO.stop();
        flywheelIO.stop();
        desiredTurretRads = azimuthInputs.turretPositionRads;
        desiredHoodRads = hoodInputs.positionRads;
      }

      case AIMING -> {
        // Use HUB profile for turret + hood, but override flywheel speed to a constant
        // pre-spin
        if (activePreset == PresetTarget.OVER_NEUTRAL_ZONE) {
          target = getPresetTarget2d(PresetTarget.OVER_NEUTRAL_ZONE); // update for the robot pos, since target moves
        }
        var params = shotCalculator.getParameters(target, robotToTurret, ShotCalculator.HUB_PROFILE);

        desiredTurretRads = params.turretAngle().getRadians();
        //desiredHoodRads = params.hoodAngle();
        desiredFlywheelRadsPerSec = aimingFlywheelSpeedRadsPerSec.get();

        azimuthIO.setDesiredPosition(desiredTurretRads);
        hoodIO.setPosition(desiredHoodRads);
        flywheelIO.setVelocity(desiredFlywheelRadsPerSec);
      }

      case SHOOTING -> {
        if (activePreset == PresetTarget.OVER_NEUTRAL_ZONE) {
          target = getPresetTarget2d(PresetTarget.OVER_NEUTRAL_ZONE); // update for the robot pos, since target moves
        }
        var params = shotCalculator.getParameters(target, robotToTurret, profile);

        desiredTurretRads = params.turretAngle().getRadians();
        //desiredHoodRads = params.hoodAngle();
        desiredFlywheelRadsPerSec = params.flywheelSpeed();

        azimuthIO.setDesiredPosition(desiredTurretRads);
        hoodIO.setPosition(desiredHoodRads);
        flywheelIO.setVelocity(desiredFlywheelRadsPerSec);
      }
      case TUNING_FLYWHEEL -> {
        flywheelIO.setVelocity(desiredFlywheelRadsPerSec);
      }
      case TUNING_AZIMUTH -> {
        azimuthIO.setDesiredPosition(desiredTurretRads);
      }
      case TUNING_HOOD -> {
        hoodIO.setPosition(desiredHoodRads);
      }
    }

    // Logging
    Logger.recordOutput(name + "/Goal", goal.toString());
    Logger.recordOutput("SuperStructure/" + name + "/Goal", goal.toString());
    Logger.recordOutput("SuperStructure/" + name + "/TargetName", activePreset.toString());
    Logger.recordOutput("SuperStructure/" + name + "/TargetPos", new Pose2d(target, new Rotation2d()));
    Logger.recordOutput(name + "/Preset", activePreset.toString());

    Logger.recordOutput(name + "/Targets/Target2d", target);

    Logger.recordOutput(name + "/Setpoints/TurretRads", desiredTurretRads);
    Logger.recordOutput(name + "/Setpoints/HoodRads", desiredHoodRads);
    Logger.recordOutput(name + "/Setpoints/FlywheelRadsPerSec", desiredFlywheelRadsPerSec);

    Logger.recordOutput(name + "/Errors/TurretRads", turretAngleErrorRads());
    Logger.recordOutput(name + "/AtAimAngle", atAimAngle());
    Logger.recordOutput(name + "/AtShootSetpoints", atShootSetpoints());
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
    return tempMap;
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    List<ParentDevice> orchestra = new ArrayList<>();
    List<SelfChecking> hardware = new ArrayList<>();
    hardware.addAll(azimuthIO.getSelfCheckingHardware());
    hardware.addAll(flywheelIO.getSelfCheckingHardware());
    hardware.addAll(hoodIO.getSelfCheckingHardware());
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
        + Math.abs(hoodInputs.supplyCurrentAmps);
  }

  @Override
  public void setCurrentLimit(int amps) {
    azimuthIO.setCurrentLimit(amps);
  }

  public void setCurrentLimit(int azimuthAmps, int flywheelAmps, int hoodAmps, int kickupAmps) {
    azimuthIO.setCurrentLimit(azimuthAmps);
    flywheelIO.setCurrentLimit(flywheelAmps);
    hoodIO.setCurrentLimit(hoodAmps);
  }

  @Override
  protected Command systemCheckCommand() {
    return runOnce(() -> {
            // Simple check logic
            if (isAzimuthConnected() && isFlywheelConnected() && isHoodConnected()) {
                Logger.recordOutput(name + "/SystemCheck/Connected", "GOOD");
            } else {
                Logger.recordOutput(name + "/SystemCheck/Connected", "BAD");
            }
        }).withName(name+"SystemCheck");
  }
}
