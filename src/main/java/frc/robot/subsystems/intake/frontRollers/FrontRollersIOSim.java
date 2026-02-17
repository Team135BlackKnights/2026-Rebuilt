package frc.robot.subsystems.intake.frontRollers;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;

public class FrontRollersIOSim extends GenericRollerSystemIOSim implements FrontRollersIO {
    public FrontRollersIOSim(DCMotor motorModel, String name, double reduction, double moi) {
        super(motorModel, name, reduction, moi);
    }

}
