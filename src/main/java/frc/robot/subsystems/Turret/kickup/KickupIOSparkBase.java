package frc.robot.subsystems.Turret.kickup;

import com.ctre.phoenix6.CANBus;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSparkBase;

public class KickupIOSparkBase extends GenericRollerSystemIOSparkBase implements KickupIO {
    public KickupIOSparkBase(int motorID, CANBus bus, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, name, currentLimitAmps, invert, brake, true, reduction);
    }
}