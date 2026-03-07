package frc.robot.subsystems.Turret.kickup;

import java.util.Optional;
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
        IDLING(new LoggableTunedNumber("Kickup/IdlingVoltage", 0.0, Constants.TuningConstants.isTuningShooter)),
        JACKHAMMER(new LoggableTunedNumber("Kickup/JackHammer", 12.0, Constants.TuningConstants.isTuningShooter),.25),
        SHOOTING(new LoggableTunedNumber("Kickup/ShootingVoltage", 11.0, Constants.TuningConstants.isTuningShooter)),
        VOMITING(new LoggableTunedNumber("Kickup/VomitingVoltage", -8, Constants.TuningConstants.isTuningShooter)); 
        private final DoubleSupplier valueSupplier;
        private final BooleanSupplier isVoltageSupplier;
        private final Optional<Double> timeout;

        Goal(DoubleSupplier valueSupplier) {
            this.valueSupplier = valueSupplier;
            this.isVoltageSupplier = () -> true;
            this.timeout = Optional.empty();
        }
        Goal(LoggableTunedNumber valueSupplier) {
            this.valueSupplier = valueSupplier::get;
            this.isVoltageSupplier = () -> true;
            this.timeout = Optional.empty();
        }
        Goal(LoggableTunedNumber valueSupplier, double timeout) {
            this.valueSupplier = valueSupplier::get;
            this.isVoltageSupplier = () -> true;
            this.timeout = Optional.of(timeout);
        }

        Goal(LoggableTunedNumber valueSupplier, BooleanSupplier isVoltageSupplier, double timeout) {
            this.valueSupplier = valueSupplier::get;
            this.isVoltageSupplier = isVoltageSupplier::getAsBoolean;
            this.timeout = Optional.of(timeout);
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