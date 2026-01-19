package frc.robot.subsystems.simpleMechanisms.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GenericRollerSystemIOSim implements GenericRollerSystemIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;
  private double appliedCurrent = 0.0;
  private final String name;

  public GenericRollerSystemIOSim(DCMotor motorModel, String name, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel, .1, .1);
    this.name = name;
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    sim.update(.02);
    inputs.name = name;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    if (appliedCurrent == 0){
      appliedCurrent = sim.getCurrentDrawAmps();
    }
    inputs.torqueCurrentAmps = appliedCurrent;
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
    appliedCurrent = 0;
  }
  @Override
  public void runCurrent(double amperes){
    double volts = (sim.getGearbox().rOhms * amperes) + (sim.getGearbox().KvRadPerSecPerVolt * sim.getAngularVelocityRadPerSec());
    appliedVoltage =  MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
    appliedCurrent = amperes;
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}