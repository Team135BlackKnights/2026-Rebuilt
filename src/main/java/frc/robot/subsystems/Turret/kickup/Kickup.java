package frc.robot.subsystems.Turret.kickup;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.LoggableTunedNumber;
import lombok.Getter;
import lombok.Setter;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystem;

@Getter
@Setter
public class Kickup extends GenericRollerSystem<Kickup.Goal> {
    @Getter
    public enum Goal implements GenericRollerSystem.RollGoalSupplier {
        IDLING(new LoggableTunedNumber("Kickup/IdlingVoltage", 2.0, Constants.TuningConstants.isTuningShooter)),
        SHOOTING(new LoggableTunedNumber("Kickup/ShootingVoltage", 5.0, Constants.TuningConstants.isTuningShooter));
        
        private final DoubleSupplier valueSupplier;
        private final BooleanSupplier isVoltageSupplier;

        Goal(DoubleSupplier valueSupplier) {
            this.valueSupplier = valueSupplier;
            this.isVoltageSupplier = () -> true;
        }

        Goal(LoggableTunedNumber valueSupplier) {
            this.valueSupplier = valueSupplier::get;
            this.isVoltageSupplier = () -> true;
        }

        Goal(LoggableTunedNumber valueSupplier, BooleanSupplier isVoltageSupplier) {
            this.valueSupplier = valueSupplier::get;
            this.isVoltageSupplier = isVoltageSupplier::getAsBoolean;
        }
    }

    private Goal goal = Goal.IDLING;

    public Kickup(KickupIO io) {
        super("Kickup", io);
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    /**
     * A command which sets to idle, ejects, and then sets to idle again.
     */
    protected Command systemCheckCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> goal = Goal.IDLING),
                Commands.run(() -> goal = Goal.SHOOTING).withTimeout(1),
                Commands.runOnce(() -> {
                    if (Math.abs(getAppliedVolts() - Goal.SHOOTING.valueSupplier.getAsDouble()) < .5) {
                        addFault(
                                "[System Check] idling voltage not reached for subsystem:"
                                        + getName(),
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.IDLING));
    }
}