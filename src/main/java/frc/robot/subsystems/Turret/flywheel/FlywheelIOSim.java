package frc.robot.subsystems.Turret.flywheel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = AdvancedMechanismConstants.Turret.flywheelMotor;
  private static final FlywheelSim sim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(motorModel, AdvancedMechanismConstants.Turret.flywheelMOI, AdvancedMechanismConstants.Turret.flywheelRatio), motorModel);

  private PIDController controller = new PIDController(10, 0, 0, 0.02);
  private double currentOutput = 0.0;
  private double currentOutputAsVolt = 0.0;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

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
        appliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);;
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
        System.out.println("Updating PID P to " + p);
    }
    @Override
    public void setCurrentLimit(double amps) {
        // Not implemented in sim
    }
}
