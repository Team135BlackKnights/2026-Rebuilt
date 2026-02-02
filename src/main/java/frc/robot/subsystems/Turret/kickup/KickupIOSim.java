package frc.robot.subsystems.Turret.kickup;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;

public class KickupIOSim extends GenericRollerSystemIOSim implements KickupIO {
    public KickupIOSim(DCMotor motorModel, String name, double reduction, double moi) {
        super(motorModel, name, reduction, moi);
    }

}
