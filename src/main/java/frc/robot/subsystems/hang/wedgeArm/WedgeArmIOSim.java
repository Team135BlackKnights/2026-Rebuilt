package frc.robot.subsystems.hang.wedgeArm;

import com.ctre.phoenix6.CANBus;

import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

public class WedgeArmIOSim extends WedgeArmIOKrakenFOC {
    public WedgeArmIOSim(CANBus bus, int motorID, int servoID, String name) {
        super(
                bus,
                motorID,
                servoID,
                name,
                SimpleMechanismConstants.Climber.wedgeArmCurrentLimit,
                SimpleMechanismConstants.Climber.wedgeArmInverted,
                true,
                SimpleMechanismConstants.Climber.wedgeReduction);
    }

    public WedgeArmIOSim() {
        this(new CANBus("rio"), SimpleMechanismConstants.Climber.wedgeArmId, SimpleMechanismConstants.Climber.wedgeArmServoId, "WedgeArmIOSim");
    }

    @Override
    public void updateInputs(WedgeArmIOInputs inputs) {
        wedgeArm.simIterate();
        super.updateInputs(inputs);
    }
}
