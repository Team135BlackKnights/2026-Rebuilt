package frc.robot.subsystems.intake.Indexer;

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
public class Indexer extends GenericRollerSystem<Indexer.Goal> {
    @RequiredArgsConstructor
    @Getter
    public enum Goal implements GenericRollerSystem.RollGoalSupplier {
        STOPPED(() -> 0),
        IDLING(new LoggableTunedNumber("IdlingVoltage", 2.0, Constants.TuningConstants.isTuningIntake)),
        INTAKING(
                new LoggableTunedNumber("IntakingVoltage", 6.0, Constants.TuningConstants.isTuningIntake)),
        VOMITING(new LoggableTunedNumber("VomitingVoltage",-6.0,Constants.TuningConstants.isTuningIntake)),
        JACKHAMMER_IN(new LoggableTunedNumber("JackhammerInAmps", 20.0, Constants.TuningConstants.isTuningIntake),
                () -> false),
        JACKHAMMER_OUT(new LoggableTunedNumber("JackhammerOutAmps", -20.0, Constants.TuningConstants.isTuningIntake),
                () -> false);
                
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

    private Goal goal = Goal.INTAKING; //should always intake or vomit

    public Indexer(IndexerIO io) {
        super("Indexer", io);
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
                Commands.run(() -> goal = Goal.INTAKING).withTimeout(1),
                Commands.runOnce(() -> {
                    if (Math.abs(getAppliedVolts() - Goal.INTAKING.valueSupplier.getAsDouble()) < .5) {
                        addFault(
                                "[System Check] Ejecting voltage not reached for subsystem:"
                                        + getName(),
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.STOPPED));
    }
}