package frc.robot.subsystems.simpleMechanisms.roller;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;

/**
 * Generic roller IO implementation for a roller or series of rollers using a
 * SPARK Base.
 */
public abstract class GenericRollerSystemIOSparkBase implements GenericRollerSystemIO {
  private final SparkBase motor;
  private final RelativeEncoder encoder;
  private SparkBaseConfig config;
  private final double reduction;
  private final String name;

  public GenericRollerSystemIOSparkBase(
      int id, String name, int currentLimitAmps, boolean invert, boolean brake, boolean isSparkMax, double reduction) {
    this(id, name, currentLimitAmps, invert, brake, isSparkMax, reduction, true);
  }

  public GenericRollerSystemIOSparkBase(
      int id,
      String name,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      boolean isSparkMax,
      double reduction,
      boolean currentLimitEnabled) {
    this.reduction = reduction;
    if (!name.equals("CoralMotor")){
      if (isSparkMax) {
        motor = new SparkMax(id, SparkBase.MotorType.kBrushless);
        config = new SparkMaxConfig();
      } else {
        motor = new SparkFlex(id, SparkBase.MotorType.kBrushless);
        config = new SparkFlexConfig();
      }
      this.name = name;
      if (currentLimitEnabled) {
        config = config.smartCurrentLimit(currentLimitAmps);
      }
      config = config.voltageCompensation(12);
      config = config.inverted(invert).idleMode(IdleMode.kCoast);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      encoder = motor.getEncoder();
    }else{
      if (isSparkMax) {
        motor = new SparkMax(id, SparkBase.MotorType.kBrushless);
        config = new SparkMaxConfig();
      } else {
        motor = new SparkFlex(id, SparkBase.MotorType.kBrushless);
        config = new SparkFlexConfig();
      }
      this.name = name;
      if (currentLimitEnabled) {
        config = config.smartCurrentLimit(currentLimitAmps);
      }
      config = config.voltageCompensation(12);
      config = config.inverted(invert).idleMode(IdleMode.kCoast);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      encoder = motor.getEncoder();
    }
    
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.name = name;

    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setCurrentLimit(double amps) {
    config = config.smartCurrentLimit((int) amps);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public List<SelfChecking> getSelfCheckingHardware() {
    List<SelfChecking> hardware = new ArrayList<SelfChecking>();
    hardware.add(new SelfCheckingSparkBase(name, motor));
    return hardware;
  }
}
