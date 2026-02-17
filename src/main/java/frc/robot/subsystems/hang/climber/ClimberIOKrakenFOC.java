package frc.robot.subsystems.hang.climber;

import com.ctre.phoenix6.CANBus;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOKrakenFOC;

public class ClimberIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements ClimberIO {
    public ClimberIOKrakenFOC(int motorID, CANBus bus, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, bus, name, currentLimitAmps, invert, brake, reduction);
    }
}
