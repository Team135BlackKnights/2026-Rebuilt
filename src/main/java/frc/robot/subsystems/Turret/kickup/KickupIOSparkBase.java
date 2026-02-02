package frc.robot.subsystems.Turret.kickup;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSparkBase;

public class KickupIOSparkBase extends GenericRollerSystemIOSparkBase implements KickupIO {
    public KickupIOSparkBase(int motorID, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, name, currentLimitAmps, invert, brake, true, reduction);
    }
}