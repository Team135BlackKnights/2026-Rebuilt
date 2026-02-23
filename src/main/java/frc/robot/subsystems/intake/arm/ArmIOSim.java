package frc.robot.subsystems.intake.arm;

import com.ctre.phoenix6.CANBus;

import frc.robot.utils.IntakeConstants;

public class ArmIOSim extends ArmIOKrakenFOC {

    public ArmIOSim(CANBus bus, int motorID, String name) {
        super(
                bus,
                motorID,
                name,
                IntakeConstants.intakeCurrentLimit,
                IntakeConstants.intakeInverted,
                true,
                IntakeConstants.intakeArmReduction);
    }

    public ArmIOSim() {
        this(new CANBus("rio"), IntakeConstants.intakeMotorID, "IntakeArmIOSim");
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        arm.simIterate();
        super.updateInputs(inputs);
    }
}
