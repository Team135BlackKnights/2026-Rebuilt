package frc.robot.subsystems.intake.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;

public class IndexerIOSim extends GenericRollerSystemIOSim implements IndexerIO {
    public IndexerIOSim(DCMotor motorModel, String name, double reduction, double moi) {
        super(motorModel, name, reduction, moi);
    }

}
