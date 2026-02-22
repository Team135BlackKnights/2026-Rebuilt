package frc.robot.utils.YAMS.mechanisms.Containers;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.YAMS.DCMotorSimSupplier;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.mechanisms.MechanismPositionConfig;
import frc.robot.utils.YAMS.mechanisms.PivotConfig;
import frc.robot.utils.YAMS.mechanisms.SmartPositionalMechanism;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Pivot mechanism.
 */
public class Pivot extends SmartPositionalMechanism
{

  /**
   * Pivot config.
   */
  private final PivotConfig          m_config;
  /**
   * Simulation for the Pivot.
   */
  private       Optional<DCMotorSim> m_dcmotorSim = Optional.empty();
  /**
   * Mechanism ligament for the setpoint.
   */
  private MechanismLigament2d m_setpointLigament = null;

  /**
   * Construct the Pivot class
   *
   * @param config Pivot configuration.
   */
  public Pivot(PivotConfig config)
  {
    m_config = config;
    m_smc = config.getMotor();
    DCMotor                    dcMotor     = m_smc.getDCMotor();
    // Seed the relative encoder
    if (m_smc.getConfig().getExternalEncoder().isPresent())
    {
      m_smc.seedRelativeEncoder();
    }
    config.applyConfig();

    if (RobotBase.isSimulation())
    {
      SmartMotorController smc = config.getMotor();
      if (config.getLowerHardLimit().isEmpty())
      {
        throw new IllegalArgumentException("Pivot lower hard limit is empty");
      }
      if (config.getUpperHardLimit().isEmpty())
      {
        throw new IllegalArgumentException("Pivot upper hard limit is empty");
      }
      if (config.getStartingAngle().isEmpty())
      {
        throw new IllegalArgumentException("Pivot starting angle is empty");    
      }
      m_dcmotorSim = Optional.of(new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor,
                                                                                   config.getMOI(),
                                                                                   smc.getConfig().getGearing()
                                                                                      .getMechanismToRotorRatio()),
                                                dcMotor));

      m_smc.setSimSupplier(new DCMotorSimSupplier(m_dcmotorSim.get(), smc));
      Distance pivotLength = Inches.of(36);
      m_mechanismWindow = new Mechanism2d(pivotLength.in(Meters) * 2,
                                          pivotLength.in(Meters) * 2);
      m_mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root",
                                                  pivotLength.in(Meters), pivotLength.in(Meters));
      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(getName(),
                                                                           pivotLength.in(Meters),
                                                                           config.getStartingAngle().get().in(Degrees),
                                                                           6,
                                                                           config.getSimColor()));
      m_setpointLigament = m_mechanismRoot.append(new MechanismLigament2d("Setpoint",
                                                                          pivotLength.in(Meters),
                                                                          config.getStartingAngle().get()
                                                                                .in(Degrees),
                                                                          3,
                                                                          new Color8Bit(Color.kWhite)));
      m_mechanismRoot.append(new MechanismLigament2d("MaxHard",
                                                     Inch.of(3).in(Meters),
                                                     config.getUpperHardLimit().get()
                                                           .in(Degrees),
                                                     4,
                                                     new Color8Bit(Color.kLimeGreen)));
      m_mechanismRoot.append(new MechanismLigament2d("MinHard", Inch.of(3).in(Meters),
                                                     config.getLowerHardLimit().get()
                                                           .in(Degrees),
                                                     4, new Color8Bit(Color.kRed)));
      if (smc.getConfig().getMechanismLowerLimit().isPresent() &&
          smc.getConfig().getMechanismUpperLimit().isPresent())
      {
        m_mechanismRoot.append(new MechanismLigament2d("MaxSoft",
                                                       Inch.of(3).in(Meters),
                                                       smc.getConfig().getMechanismUpperLimit().get()
                                                          .in(Degrees),
                                                       4,
                                                       new Color8Bit(Color.kHotPink)));
        m_mechanismRoot.append(new MechanismLigament2d("MinSoft", Inch.of(3).in(Meters),
                                                       smc.getConfig().getMechanismLowerLimit().get()
                                                          .in(Degrees),
                                                       4, new Color8Bit(Color.kYellow)));
      }
      SmartDashboard.putData(getName() + "/mechanism",
                             m_mechanismWindow);
    }
  }

  /**
   * Between two angles.
   *
   * @param start Start angle.
   * @param end   End angle
   * @return {@link Trigger}
   */
  public Trigger between(Angle start, Angle end)
  {
    return gte(start).and(lte(end));
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return {@link Trigger} for Pivot.
   */
  public Trigger gte(Angle angle)
  {
    return new Trigger(() -> getAngle().gte(angle));
  }

  /**
   * Less than or equal to angle
   *
   * @param angle {@link Angle} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Angle angle)
  {
    return new Trigger(() -> getAngle().lte(angle));
  }

  /**
   * Get the {@link SmartMotorController} Mechanism Position representing the pivot.
   *
   * @return Pivot {@link Angle}
   */
  public Angle getAngle()
  {
    return m_smc.getMechanismPosition();
  }

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command setAngle(Angle angle)
  {
    return run(angle).withName(" SetAngle");
  }

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command setAngle(Supplier<Angle> angle)
  {
    return run(angle).withName(" SetAngle Supplier");
  }

  /**
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command run(Angle angle)
  {
    return Commands.run(() -> m_smc.setPosition(angle)).withName(" SetAngle");
  }

  /**
   * Set the pivot to the given angle via a supplier.
   *
   * @param angle Supplier for the pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
   */
  public Command run(Supplier<Angle> angle)
  {
    return Commands.run(() -> m_smc.setPosition(angle.get())).withName(" RunAngle Supplier");
  }

  /**
   * Set the pivot to the given {@link Angle} then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the pivot to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on the Subsystem.
   */
  public Command runTo(Angle angle, Angle tolerance)
  {
    return Commands.runOnce(() -> m_smc.setPosition(angle))
                   .andThen(Commands.waitUntil(isNear(angle, tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(" RunTo Angle");
  }

  /**
   * Set the pivot to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the pivot to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on the Subsystem.
   */
  public Command runTo(Supplier<Angle> angle, Angle tolerance)
  {
    return Commands.runOnce(() -> m_smc.setPosition(angle.get()))
                   .andThen(Commands.waitUntil(isNear(angle.get(), tolerance).debounce(0.1, DebounceType.kRising)))
                   .withName(" RunTo Angle Supplier");
  }

  /**
   * Pivot is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return {@link Trigger} on when the pivot is near another angle.
   */
  public Trigger isNear(Angle angle, Angle within)
  {
    return new Trigger(() -> getAngle().isNear(angle, within));
  }

  @Override
  public Trigger max()
  {
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent())
    {
      return new Trigger(gte(m_smc.getConfig().getMechanismUpperLimit().get()));
    }
    if (m_config.getUpperHardLimit().isEmpty())
    {
      return gte(m_config.getUpperHardLimit().get());
    }
    throw new IllegalArgumentException("Pivot upper hard and motor controller soft limit is empty");
  }

  @Override
  public Trigger min()
  {
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent())
    {
      return new Trigger(gte(m_smc.getConfig().getMechanismLowerLimit().get()));
    }
    if (m_config.getLowerHardLimit().isEmpty())
    {
      return gte(m_config.getLowerHardLimit().get());
    }
    throw new IllegalArgumentException("Pivot lower hard and motor controller soft limit is empty");
  }
/* 
  @Override
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    SysIdRoutine routine = m_smc.sysId(maximumVoltage, step, duration);
    Angle        max;
    Angle        min;
    if (m_smc.getConfig().getMechanismUpperLimit().isPresent())
    {
      max = m_smc.getConfig().getMechanismUpperLimit().get().minus(Degrees.of(1));
    } else if (m_config.getUpperHardLimit().isPresent())
    {
      max = m_config.getUpperHardLimit().get().minus(Degrees.of(1));
    } else
    {
      throw new IllegalArgumentException("Pivot upper hard and motor controller soft limit is empty");
    }
    if (m_smc.getConfig().getMechanismLowerLimit().isPresent())
    {
      min = m_smc.getConfig().getMechanismLowerLimit().get().plus(Degrees.of(1));
    } else if (m_config.getLowerHardLimit().isPresent())
    {
      min = m_config.getLowerHardLimit().get().plus(Degrees.of(1));
    } else
    {
      throw new IllegalArgumentException("Pivot lower hard and motor controller soft limit is empty");
    }
    Trigger maxTrigger = gte(max);
    Trigger minTrigger = lte(min);

    Command group = Commands.print("Starting SysId")
                            .andThen(Commands.runOnce(m_smc::stopClosedLoopController))
                            .andThen(routine.dynamic(Direction.kForward).until(maxTrigger))
                            .andThen(routine.dynamic(Direction.kReverse).until(minTrigger))
                            .andThen(routine.quasistatic(Direction.kForward).until(maxTrigger))
                            .andThen(routine.quasistatic(Direction.kReverse).until(minTrigger))
                            .finallyDo(m_smc::startClosedLoopController);
    if (m_config.getTelemetryName().isPresent())
    {
      group = group.andThen(Commands.print(getName() + " SysId test done."));
    }
    return group.withName(m_subsystem.getName() + " SysId");
  }*/

  @Override
  public void simIterate()
  {
    if (m_dcmotorSim.isPresent() && m_smc.getSimSupplier().isPresent())
    {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();
      if (m_config.getLowerHardLimit().isPresent() && m_dcmotorSim.get().getAngularVelocityRadPerSec() < 0 &&
          m_smc.getMechanismPosition().lt(m_config.getLowerHardLimit().get()))
      {
        m_smc.setEncoderPosition(m_config.getLowerHardLimit().get());
        // Stop the motor from moving further in the direction of the hard limit
        m_dcmotorSim.get().setAngularVelocity(0);
        m_smc.setDutyCycle(0);
      }
      if (m_config.getUpperHardLimit().isPresent() && m_dcmotorSim.get().getAngularVelocityRadPerSec() > 0 &&
          m_smc.getMechanismPosition().gt(m_config.getUpperHardLimit().get()))
      {
        m_smc.setEncoderPosition(m_config.getUpperHardLimit().get());
        // Stop the motor from moving further in the direction of the hard limit
        m_dcmotorSim.get().setAngularVelocity(0);
        m_smc.setDutyCycle(0);
      }
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_dcmotorSim.get()
                                                                                           .getCurrentDrawAmps()));
      visualizationUpdate();
    }
  }

  /**
   * Updates the angle of the mechanism ligament to match the current angle of the pivot.
   */
  @Override
  public void visualizationUpdate()
  {
// TODO: Add setpoint ligament
    m_mechanismLigament.setAngle(getAngle().in(Degrees));
    m_setpointLigament.setAngle(m_smc.getMechanismPositionSetpoint().orElse(getAngle()).in(Degrees));
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined in the
   * {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition()
  {
    Translation3d mechanismTranslation = new Translation3d(m_mechanismLigament.getLength(),
                                                           new Rotation3d(0, 0, m_mechanismLigament.getAngle()));
    if (m_config.getMechanismPositionConfig().getRelativePosition().isPresent())
    {
      return m_config.getMechanismPositionConfig().getRelativePosition().get()
                     .plus(mechanismTranslation);
    }
    return mechanismTranslation;
  }

  @Override
  public String getName()
  {
    return m_config.getTelemetryName().orElse("Pivot");
  }

  /**
   * Get the {@link PivotConfig} object for this {@link Pivot}
   *
   * @return The {@link PivotConfig} object for this {@link Pivot}
   */
  public PivotConfig getPivotConfig()
  {
    return m_config;
  }


  @Override
  @Deprecated
  public void setMeasurementVelocitySetpoint(LinearVelocity velocity)
  {
    throw new RuntimeException("Unimplemented");
  }

  @Override
  @Deprecated
  public void setMeasurementPositionSetpoint(Distance distance)
  {
    throw new RuntimeException("Unimplemented");
  }
}