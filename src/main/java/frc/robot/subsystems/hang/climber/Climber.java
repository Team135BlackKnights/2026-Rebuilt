package frc.robot.subsystems.hang.climber;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.LoggableTunedNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystem;

@Getter
@Setter
public class Climber extends GenericRollerSystem<Climber.Goal> {
    @RequiredArgsConstructor
    @Getter
    public enum Goal implements GenericRollerSystem.RollGoalSupplier {
        STOPPED(() -> 0),
        CLIMBING(
                new LoggableTunedNumber("Climber/ClimbingVoltage", 5.0, Constants.TuningConstants.isTuningClimber)),
        DROPPING(new LoggableTunedNumber("Climber/DroppingVoltage", -2.0, Constants.TuningConstants.isTuningClimber));

                
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

        Goal(LoggableTunedNumber valueSupplier, BooleanSupplier isVoltageSupplier) {
            this.valueSupplier = valueSupplier::get;
            this.isVoltageSupplier = isVoltageSupplier::getAsBoolean;
            this.timeout = Optional.empty();
        }
    }

    private Goal goal = Goal.STOPPED;

    public Climber(ClimberIO io) {
        super("Hang/Climber", io);
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
                Commands.runOnce(() -> goal = Goal.STOPPED),
                Commands.run(() -> goal = Goal.DROPPING).withTimeout(1),
                Commands.runOnce(() -> {
                    if (Math.abs(getAppliedVolts() - Goal.DROPPING.valueSupplier.getAsDouble()) < .5) {
                        addFault(
                                "[System Check] Dropping voltage not reached for subsystem:"
                                        + getName(),
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.STOPPED));
    }
}