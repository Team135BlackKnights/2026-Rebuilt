package frc.robot.subsystems.simpleMechanisms.roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

public abstract class GenericRollerSystem<G extends GenericRollerSystem.RollGoalSupplier> extends SubsystemChecker {
  public interface RollGoalSupplier {
    BooleanSupplier getIsVoltageSupplier();

    DoubleSupplier getValueSupplier();

    Optional<Double> getTimeout();
  }

  public abstract G getGoal();

  protected final String name;
  protected final GenericRollerSystemIO io;
  protected final GenericRollerSystemIOInputsAutoLogged inputs = new GenericRollerSystemIOInputsAutoLogged();
  protected final Timer stateTimer = new Timer();
  protected G lastGoal;
  protected boolean lastTimeout = false;
  protected boolean hasTimeout = false;
  protected double radsBeforeLock = 0;
  protected double integratedRadians = 0;

  public GenericRollerSystem(String name, GenericRollerSystemIO io) {
    this.name = name;
    this.setName(name);
    this.io = io;

    stateTimer.start();
    registerSelfCheckHardware();
    if (name != "Kickup") {
      CommandScheduler.getInstance().unregisterSubsystem(this);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }
    if (getGoal().getIsVoltageSupplier().getAsBoolean()) {
      if (getGoal().getTimeout().isPresent()) {
        double t = Timer.getFPGATimestamp() % (getGoal().getTimeout().get() * 2);
        if (t < getGoal().getTimeout().get()) {
          io.runVolts(-getGoal().getValueSupplier().getAsDouble());
        } else {
          io.runVolts(getGoal().getValueSupplier().getAsDouble());
        }
      }
      io.runVolts(getGoal().getValueSupplier().getAsDouble());
      /*
       * if (integratedRadians <= radsBeforeLock){
       * io.runVolts(getGoal().getValueSupplier().getAsDouble());
       * }else{
       * io.runVolts(0);
       * }
       */
    } else {
      io.runCurrent(getGoal().getValueSupplier().getAsDouble());
    }
    Logger.recordOutput("SuperStructure/" + name + "Goal", getGoal().toString());
    Logger.recordOutput("SuperStructure/" + name + "stateTimer", stateTimer.get());
  }

  public HashMap<String, Double> getTemps() {
    HashMap<String, Double> tempMap = new HashMap<>();
    tempMap.put(inputs.name, inputs.tempCelsius);
    return tempMap;
  }

  public double getRadians() {
    return inputs.positionRads;
  }

  public double getAppliedVolts() {
    return inputs.appliedVoltage;
  }

  private void registerSelfCheckHardware() {
    super.registerAllHardware(io.getSelfCheckingHardware());
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    List<ParentDevice> orchestra = new ArrayList<>();
    List<SelfChecking> hardware = io.getSelfCheckingHardware();
    for (SelfChecking motor : hardware) {
      if (motor.getHardware() instanceof TalonFX) {
        orchestra.add((TalonFX) motor.getHardware());
      }
    }
    return orchestra;
  }

  @Override
  public double getCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  @Override
  public void setCurrentLimit(int amps) {
    io.setCurrentLimit(amps);
  }

  public List<SelfChecking> getHardware() {
    return io.getSelfCheckingHardware();
  }
}