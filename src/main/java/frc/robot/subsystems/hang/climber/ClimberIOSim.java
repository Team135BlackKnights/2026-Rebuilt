package frc.robot.subsystems.hang.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;

public class ClimberIOSim extends GenericRollerSystemIOSim implements ClimberIO {
    public ClimberIOSim(DCMotor motorModel, String name, double reduction, double moi) {
        super(motorModel, name, reduction, moi);
    }

}
