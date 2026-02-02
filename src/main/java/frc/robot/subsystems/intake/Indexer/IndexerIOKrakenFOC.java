package frc.robot.subsystems.intake.Indexer;

import com.ctre.phoenix6.CANBus;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOKrakenFOC;

public class IndexerIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements IndexerIO {
    public IndexerIOKrakenFOC(int motorID, CANBus bus, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, bus, name, currentLimitAmps, invert, brake, reduction);
    }
}
