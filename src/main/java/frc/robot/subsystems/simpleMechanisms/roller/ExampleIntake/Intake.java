package frc.robot.subsystems.simpleMechanisms.roller.ExampleIntake;

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
public class Intake extends GenericRollerSystem<Intake.Goal> {
    @RequiredArgsConstructor
    @Getter
    public enum Goal implements GenericRollerSystem.RollGoalSupplier {
        IDLING(() -> 0),
        AMP_SCORING(
                new LoggableTunedNumber("Intake/AmpScoringVoltage", 12.0, Constants.TuningConstants.isTuningIntake)),
        TRAP_SCORING(
                new LoggableTunedNumber("Intake/TrapScoringVoltage", 8.0, Constants.TuningConstants.isTuningIntake)),
        TRAP_JACKHAMMER_OUT(
                new LoggableTunedNumber("Intake/JackhammerOutVoltage", 8.0, Constants.TuningConstants.isTuningIntake)),
        TRAP_JACKHAMMER_IN(
                new LoggableTunedNumber("Intake/JackhammerInVoltage", -2.0, Constants.TuningConstants.isTuningIntake)),
        EJECTING(new LoggableTunedNumber("Intake/EjectingVoltage", -12.0, Constants.TuningConstants.isTuningIntake));

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

    public Intake(IntakeIO io) {
        super("Intake", io);
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
                Commands.run(() -> goal = Goal.EJECTING).withTimeout(1),
                Commands.runOnce(() -> {
                    if (Math.abs(getAppliedVolts() - Goal.EJECTING.valueSupplier.getAsDouble()) < .5) {
                        addFault(
                                "[System Check] Ejecting voltage not reached for subsystem:"
                                        + getName(),
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.IDLING));
    }
}