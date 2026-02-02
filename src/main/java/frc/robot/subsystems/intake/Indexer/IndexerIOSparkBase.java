package frc.robot.subsystems.intake.Indexer;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSparkBase;

public class IndexerIOSparkBase extends GenericRollerSystemIOSparkBase implements IndexerIO {
    public IndexerIOSparkBase(int motorID, String name, int currentLimitAmps, boolean invert, boolean brake,
            double reduction) {
        super(motorID, name, currentLimitAmps, invert, brake, true, reduction);
    }
}