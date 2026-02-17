package frc.robot.subsystems.intake.frontRollers;

import com.ctre.phoenix6.CANBus;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOKrakenFOC;

public class FrontRollersIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements FrontRollersIO {
    public FrontRollersIOKrakenFOC(int motorID, CANBus bus, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, bus, name, currentLimitAmps, invert, brake, reduction);
    }
}
