package frc.robot.subsystems.hang.wedgeArm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

public class WedgeArmIOSim implements WedgeArmIO {
    // Single jointed arm siom
    private final SingleJointedArmSim sim = new SingleJointedArmSim(SimpleMechanismConstants.Climber.wedgeArmMotor, SimpleMechanismConstants.Climber.wedgeReduction, SimpleMechanismConstants.Climber.wedgeMOI,
            Units.inchesToMeters(7),SimpleMechanismConstants.Climber.wedgeMinAngleRads, SimpleMechanismConstants.Climber.wedgeMaxAngleRads, false, SimpleMechanismConstants.Climber.wedgeMinAngleRads);
    private double appliedVoltage = 0.0;
    private final String name = "WedgeArmIOSim";
    private PIDController controller = new PIDController(0, 0, 0, 0.02);
    @Override
    public void updateInputs(WedgeArmIOInputs inputs) {
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
    public void setVoltage(double volts) {
        appliedVoltage = Math.max(-12.0, Math.min(12.0, volts));
    }
    @Override
    public void stop() {
        setVoltage(0.0);
    }
    @Override
    public void setPID(
        double p,
        double i,
        double d,
        double ks,
        double kv) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
    }
}
