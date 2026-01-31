package frc.robot.subsystems.Shooter.flywheel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final FlywheelSim sim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(motorModel, .025, 1), motorModel);

  private PIDController controller = new PIDController(0, 0, 0, 0.02);
  private double currentOutput = 0.0;
  private double currentOutputAsVolt = 0.0;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    currentOutputAsVolt = motorModel.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
    appliedVolts = currentOutputAsVolt;

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(.02);

    inputs.connected = true;
    inputs.accelRadsPerSec2 = sim.getAngularAccelerationRadPerSecSq();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
    inputs.name = "FlywheelIOSim";
  }
    @Override
    public void setVelocity(double velocityRadsPerSec) {
        double pidOutput = controller.calculate(sim.getAngularVelocityRadPerSec(), velocityRadsPerSec);
        currentOutput = pidOutput;
    }
    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        currentOutput = motorModel.getCurrent(appliedVolts, sim.getAngularVelocityRadPerSec());
    }
    @Override
    public void stop() {
        runVolts(0.0);
    }
    @Override
    public void setPID(
        double p,
        double d,
        double ks,
        double kv) {
        controller.setP(p);
        controller.setD(d);
    }
    @Override
    public void setCurrentLimit(double amps) {
        // Not implemented in sim
    }
}
