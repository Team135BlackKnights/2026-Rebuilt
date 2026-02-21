package frc.robot.subsystems.Turret.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

public class HoodIOSim implements HoodIO {
    // Single jointed arm siom
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getMinion(1), AdvancedMechanismConstants.Turret.hoodEncoderToHoodArmRatio * AdvancedMechanismConstants.Turret.hoodMotorToHoodEncoderRatio, AdvancedMechanismConstants.Turret.hoodMOI,
            Units.inchesToMeters(7), AdvancedMechanismConstants.Turret.minHoodAngle, AdvancedMechanismConstants.Turret.maxHoodAngle, false, AdvancedMechanismConstants.Turret.minHoodAngle);
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
