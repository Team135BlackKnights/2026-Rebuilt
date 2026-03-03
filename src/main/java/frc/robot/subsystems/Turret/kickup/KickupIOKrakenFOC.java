package frc.robot.subsystems.Turret.kickup;

import com.ctre.phoenix6.CANBus;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOKrakenFOC;

public class KickupIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements KickupIO {
    public KickupIOKrakenFOC(int motorID, CANBus bus, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, bus, name, currentLimitAmps, invert, brake, reduction);
    }
}

