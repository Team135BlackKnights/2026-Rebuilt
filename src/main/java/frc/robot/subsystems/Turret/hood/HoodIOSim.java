package frc.robot.subsystems.Turret.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.YAMS.GearBox;
import frc.robot.utils.YAMS.MechanismGearing;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.SmartMotorControllerConfig;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;
import frc.robot.utils.YAMS.TalonFXSWrapper;
import frc.robot.utils.YAMS.mechanisms.ArmConfig;
import frc.robot.utils.YAMS.mechanisms.Containers.Arm;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

public class HoodIOSim implements HoodIO {
  private final String name;

  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final ArmConfig hoodConfig;
  private final Arm hood;

  private final double minAngleRads;
  private final double maxAngleRads;

  public HoodIOSim(CANBus bus, int motorID, String name, double minAngleRads, double maxAngleRads) {
    this.name = name;
    this.minAngleRads = minAngleRads;
    this.maxAngleRads = maxAngleRads;
    // Matches your old SingleJointedArmSim gearing term:
    final double rotorPerMechanism =
        AdvancedMechanismConstants.Turret.hoodEncoderToHoodArmRatio
            * AdvancedMechanismConstants.Turret.hoodMotorToHoodEncoderRatio;

    motorConfig =
        new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                0.0, 0.0, 0.0, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440))
            .withSimClosedLoopController(
                0.2, 0.0, 0.0, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440))
            .withFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
            .withSimFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(rotorPerMechanism)))
            //.withSoftLimit(Radians.of(minAngleRads), Radians.of(maxAngleRads))
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(AdvancedMechanismConstants.Turret.invertHood)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    // Dummy CANBus + device ID for sim
    TalonFXS talon = new TalonFXS(motorID, bus);
    motor = new TalonFXSWrapper(talon, DCMotor.getMinion(1), motorConfig);

    hoodConfig =
        new ArmConfig(motor)
            .withLength(Inches.of(7))
            .withMOI(AdvancedMechanismConstants.Turret.hoodMOI)
            .withHardLimit(Radians.of(minAngleRads-Units.degreesToRadians(12)), Radians.of(maxAngleRads+Units.degreesToRadians(12)))
            .withStartingPosition(Radians.of(minAngleRads));

    hood = new Arm(hoodConfig);
  }

  public HoodIOSim() {
    this(
        new CANBus("rio"),
        0,
        "HoodIOSim",
        AdvancedMechanismConstants.Turret.leftMinHoodAngle,
        AdvancedMechanismConstants.Turret.leftMaxHoodAngle);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hood.simIterate();

    inputs.connected = true;
    inputs.positionRads = hood.getAngle().in(Radians);
    inputs.velocityRadsPerSec = motor.getMechanismVelocity().in(RadiansPerSecond);
    inputs.appliedVoltage = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getStatorCurrent().in(Amps);
  }

  @Override
  public void setPosition(double positionRads) {
    double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
    hood.setMechanismPositionSetpoint(Radians.of(clamped));
  }

  @Override
  public void runVolts(double volts) {
    hood.setVoltage(Volts.of(volts));
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }

  @Override
  public void setPID(double p, double d, double ks, double kv) {
    motor.setFeedback(p, 0.0, d);
    motor.setFeedforward(ks, kv, 0.0, 0.0);
  }
}
