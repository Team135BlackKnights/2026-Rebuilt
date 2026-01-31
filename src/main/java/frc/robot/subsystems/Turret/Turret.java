package frc.robot.subsystems.Turret;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
  private static final LoggableTunedNumber azimuth_kP =
      new LoggableTunedNumber("Turret/Azimuth/kP", 0.0, true);
  private static final LoggableTunedNumber azimuth_kI =
      new LoggableTunedNumber("Turret/Azimuth/kI", 0.0, true);
  private static final LoggableTunedNumber azimuth_kD =
      new LoggableTunedNumber("Turret/Azimuth/kD", 0.0, true);
  private static final LoggableTunedNumber azimuth_kS =
      new LoggableTunedNumber("Turret/Azimuth/kS", 0.0, true);
  private static final LoggableTunedNumber azimuth_kV =
      new LoggableTunedNumber("Turret/Azimuth/kV", 0.0, true);
  private static final LoggableTunedNumber azimuth_kA =
      new LoggableTunedNumber("Turret/Azimuth/kA", 0.0, true);
  private static final LoggableTunedNumber azimuth_velMax =
      new LoggableTunedNumber("Turret/Azimuth/velMaxRadPerSec", 12.0, true);
  private static final LoggableTunedNumber azimuth_accelMax =
      new LoggableTunedNumber("Turret/Azimuth/accelMaxRadPerSec2", 40.0, true);

  // Flywheel setPID(p,d,ks,kv)
  private static final LoggableTunedNumber flywheel_kP =
      new LoggableTunedNumber("Turret/Flywheel/kP", 0.0, true);
  private static final LoggableTunedNumber flywheel_kD =
      new LoggableTunedNumber("Turret/Flywheel/kD", 0.0, true);
  private static final LoggableTunedNumber flywheel_kS =
      new LoggableTunedNumber("Turret/Flywheel/kS", 0.0, true);
  private static final LoggableTunedNumber flywheel_kV =
      new LoggableTunedNumber("Turret/Flywheel/kV", 0.0, true);

  // Hood setPID(p,d,ks,kv)
  private static final LoggableTunedNumber hood_kP =
      new LoggableTunedNumber("Turret/Hood/kP", 0.0, true);
  private static final LoggableTunedNumber hood_kD =
      new LoggableTunedNumber("Turret/Hood/kD", 0.0, true);
  private static final LoggableTunedNumber hood_kS =
      new LoggableTunedNumber("Turret/Hood/kS", 0.0, true);
  private static final LoggableTunedNumber hood_kV =
      new LoggableTunedNumber("Turret/Hood/kV", 0.0, true);

  // Other tunables
  private static final LoggableTunedNumber aimingFlywheelSpeedRadsPerSec =
      new LoggableTunedNumber("Turret/Aiming/FlywheelSpeedRadsPerSec", 125.0, true);

  private static final LoggableTunedNumber aimToleranceRads =
      new LoggableTunedNumber("Turret/Tolerance/AimRads", Math.toRadians(1.5), true);
  private static final LoggableTunedNumber hoodToleranceRads =
      new LoggableTunedNumber("Turret/Tolerance/HoodRads", Math.toRadians(1.0), true);
  private static final LoggableTunedNumber flywheelToleranceRadsPerSec =
      new LoggableTunedNumber("Turret/Tolerance/FlywheelRadsPerSec", 10.0, true);

  private final AzimuthIO azimuthIO;
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;
  private final String name;

  private final Transform2d robotToTurret;

  private final AzimuthIOInputsAutoLogged azimuthInputs = new AzimuthIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  public enum TurretGoalType {
    AIMING,
    SHOOTING,
    IDLE
  }

  public enum Goal {
    AIMING,          // hub top center + pre-spin
    SHOOTING,        // hub top center using HUB profile
    SHOOTING_OTHER,  // preset/translation using TRENCH profile
    IDLE 
  }

  public enum PresetTarget {
    HUB_TOP_CENTER,
    LEFT_TRENCH_CENTER,
    RIGHT_TRENCH_CENTER
  }

  private Goal goal = Goal.IDLE;
  private Goal lastGoal = Goal.IDLE;
  private TurretGoalType currentGoalType = TurretGoalType.IDLE;

  private PresetTarget activeOtherPreset = PresetTarget.LEFT_TRENCH_CENTER;

  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();

  private Translation2d hubTarget2d = new Translation2d();
  private Translation2d otherTarget2d = new Translation2d();

  private double desiredTurretRads = 0.0;
  private double desiredHoodRads = 0.0;
  private double desiredFlywheelRadsPerSec = 0.0;

  public Turret(AzimuthIO azimuthIO, FlywheelIO flywheelIO, HoodIO hoodIO, Transform2d robotToTurret, String name) {
    this.azimuthIO = azimuthIO;
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;
    this.robotToTurret = robotToTurret;
    this.name = name;

    // Apply initial PIDs once
    applyAllPIDs();

    // Initialize targets
    hubTarget2d = getPresetTarget2d(PresetTarget.HUB_TOP_CENTER);
    otherTarget2d = getPresetTarget2d(PresetTarget.LEFT_TRENCH_CENTER);
  }


  public void setGoal(Goal goal) {
    if (goal != null) this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public TurretGoalType getCurrentGoalType() {
    return currentGoalType;
  }

  /** Choose which preset the "OTHER" mode shoots to. */
  public void setPresetTarget(PresetTarget preset) {
    if (preset == null) return;
    this.activeOtherPreset = preset;
    this.otherTarget2d = getPresetTarget2d(preset);
  }

  private void applyAllPIDs() {
    azimuthIO.setPID(
        azimuth_kP.get(), azimuth_kI.get(), azimuth_kD.get(),
        azimuth_kS.get(), azimuth_kV.get(), azimuth_kA.get(),
        azimuth_velMax.get(), azimuth_accelMax.get());

    flywheelIO.setPID(
        flywheel_kP.get(), flywheel_kD.get(),
        flywheel_kS.get(), flywheel_kV.get());

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
            azimuth_velMax.get(), azimuth_accelMax.get()),
        azimuth_kP, azimuth_kI, azimuth_kD, azimuth_kS, azimuth_kV, azimuth_kA, azimuth_velMax, azimuth_accelMax);

    LoggableTunedNumber.ifChanged(
        hashCode(),
        () -> flywheelIO.setPID(
            flywheel_kP.get(), flywheel_kD.get(),
            flywheel_kS.get(), flywheel_kV.get()),
        flywheel_kP, flywheel_kD, flywheel_kS, flywheel_kV);

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

    updateTunablePIDs(); //way cleaner than last year lol

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;
    }

    hubTarget2d = getPresetTarget2d(PresetTarget.HUB_TOP_CENTER);

    shotCalculator.clearShootingParameters();

    if (goal != lastGoal) {
      shotCalculator.clearShootingParameters();
      lastGoal = goal;
    }

    switch (goal) {
      case IDLE -> {
        currentGoalType = TurretGoalType.IDLE;

        desiredFlywheelRadsPerSec = 0.0;

        azimuthIO.stop();
        hoodIO.stop();
        flywheelIO.stop();

        desiredTurretRads = azimuthInputs.turretPositionRads;
        desiredHoodRads = hoodInputs.positionRads;
      }

      case AIMING -> {
        currentGoalType = TurretGoalType.AIMING;

        // Use HUB profile for turret + hood, but override flywheel speed to a constant pre-spin
        var params = shotCalculator.getParameters(hubTarget2d, robotToTurret, ShotCalculator.HUB_PROFILE);

        desiredTurretRads = params.turretAngle().getRadians();
        desiredHoodRads = params.hoodAngle();
        desiredFlywheelRadsPerSec = aimingFlywheelSpeedRadsPerSec.get();

        azimuthIO.setDesiredPosition(desiredTurretRads);
        hoodIO.setPosition(desiredHoodRads);
        flywheelIO.setVelocity(desiredFlywheelRadsPerSec);
      }

      case SHOOTING -> {
        currentGoalType = TurretGoalType.SHOOTING;

        var params = shotCalculator.getParameters(hubTarget2d, robotToTurret, ShotCalculator.HUB_PROFILE);

        desiredTurretRads = params.turretAngle().getRadians();
        desiredHoodRads = params.hoodAngle();
        desiredFlywheelRadsPerSec = params.flywheelSpeed();

        azimuthIO.setDesiredPosition(desiredTurretRads);
        hoodIO.setPosition(desiredHoodRads);
        flywheelIO.setVelocity(desiredFlywheelRadsPerSec);
      }

      case SHOOTING_OTHER -> {
        currentGoalType = TurretGoalType.SHOOTING;


        ShotProfile profile = ShotCalculator.TRENCH_PROFILE; //might want more...

        var params = shotCalculator.getParameters(otherTarget2d, robotToTurret, profile);

        desiredTurretRads = params.turretAngle().getRadians();
        desiredHoodRads = params.hoodAngle();
        desiredFlywheelRadsPerSec = params.flywheelSpeed();

        azimuthIO.setDesiredPosition(desiredTurretRads);
        hoodIO.setPosition(desiredHoodRads);
        flywheelIO.setVelocity(desiredFlywheelRadsPerSec);
      }
    }

    // Logging
    Logger.recordOutput(name + "/Goal", goal.toString());
    Logger.recordOutput(name + "/GoalType", currentGoalType.toString());
    Logger.recordOutput(name + "/Preset", activeOtherPreset.toString());

    Logger.recordOutput(name + "/Targets/Hub2d", hubTarget2d);
    Logger.recordOutput(name + "/Targets/Other2d", otherTarget2d);

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

  public void setCurrentLimit(int azimuthAmps, int flywheelAmps, int hoodAmps) {
    azimuthIO.setCurrentLimit(azimuthAmps);
    flywheelIO.setCurrentLimit(flywheelAmps);
    hoodIO.setCurrentLimit(hoodAmps);
  }

  @Override
  protected Command systemCheckCommand() {
    return runOnce(() -> {
      Logger.recordOutput(name + "/SystemCheck/AzimuthConnected", isAzimuthConnected());
      Logger.recordOutput(name + "/SystemCheck/FlywheelConnected", isFlywheelConnected());
      Logger.recordOutput(name + "/SystemCheck/HoodConnected", isHoodConnected());
    });
  }
}
