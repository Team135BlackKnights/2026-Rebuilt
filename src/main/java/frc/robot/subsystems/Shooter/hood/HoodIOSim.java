package frc.robot.subsystems.Shooter.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
    // Single jointed arm siom
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getMinion(1), 50.0 / 12.0 * 14 / 12.0, 0.05,
            Units.inchesToMeters(7), Units.degreesToRadians(12), Units.degreesToRadians(50), false, Units.degreesToRadians(12));
    private double appliedVoltage = 0.0;
    private final String name = "HoodIOSim";
    private PIDController controller = new PIDController(0, 0, 0, 0.02);
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.setInputVoltage(appliedVoltage);
        sim.update(0.02);
        inputs.connected = true;
        inputs.name = name;
        inputs.positionRads = sim.getAngleRads();
        inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.appliedVoltage = appliedVoltage;
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    }
    @Override
    public void setPosition(double positionRads) {
        double pidOutput = controller.calculate(sim.getAngleRads(), positionRads);
        appliedVoltage = Math.max(-12.0, Math.min(12.0, pidOutput));
    }

    @Override
    public void runVolts(double volts) {
        appliedVoltage = Math.max(-12.0, Math.min(12.0, volts));
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
}
