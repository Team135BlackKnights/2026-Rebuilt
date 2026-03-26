package frc.robot.subsystems.intake.frontRollers;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSparkBase;

public class FrontRollersIOSparkBase extends GenericRollerSystemIOSparkBase implements FrontRollersIO {
    public FrontRollersIOSparkBase(int motorID, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, name, currentLimitAmps, invert, brake, true, reduction, false);
    }
}
