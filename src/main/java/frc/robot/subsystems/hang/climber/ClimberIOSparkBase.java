package frc.robot.subsystems.hang.climber;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSparkBase;

public class ClimberIOSparkBase extends GenericRollerSystemIOSparkBase implements ClimberIO {
    public ClimberIOSparkBase(int motorID, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, name, currentLimitAmps, invert, brake, true, reduction);
    }
}