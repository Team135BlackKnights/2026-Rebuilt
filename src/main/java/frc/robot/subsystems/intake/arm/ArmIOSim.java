package frc.robot.subsystems.intake.arm;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.IntakeConstants;

public class ArmIOSim extends ArmIOKrakenFOC {

    private final SingleJointedArmSim armSim;
    private final PIDController simPID = new PIDController(12.0, 0.0, 1.85);

    public ArmIOSim(CANBus bus, int motorID, String name) {
        super(
                bus,
                motorID,
                name,
                IntakeConstants.intakeCurrentLimit,
                IntakeConstants.intakeInverted,
                true,
                IntakeConstants.intakeArmReduction);

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX44Foc(1),
                IntakeConstants.intakeArmReduction,
                IntakeConstants.armMOI,
                Units.inchesToMeters(7),
                IntakeConstants.armMinAngleRads,
                IntakeConstants.armMaxAngleRads,
                false,
                IntakeConstants.armMaxAngleRads);
    }

    public ArmIOSim() {
        this(new CANBus("rio"), IntakeConstants.intakeMotorID, "IntakeArmIOSim");
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(0.02);
        super.updateInputs(inputs);
        // Override position/velocity from sim
        inputs.positionRads = armSim.getAngleRads();
        inputs.velocityRadsPerSec = armSim.getVelocityRadPerSec();
    }

    @Override
    public void setPosition(double positionRads) {
        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
        double volts = simPID.calculate(armSim.getAngleRads(), clamped);
        armSim.setInputVoltage(volts);
    }

    @Override
    public void setVoltage(double volts) {
        super.setVoltage(volts);
        armSim.setInputVoltage(volts);
    }
}
