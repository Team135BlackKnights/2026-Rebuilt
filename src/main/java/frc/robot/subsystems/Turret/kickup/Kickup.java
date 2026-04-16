package frc.robot.subsystems.Turret.kickup;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
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
        IDLING(0.0, true),
        JACKHAMMER(9.0, true, 0.25),
        SHOOTING(9.0, true),
        TESTING(9.0, true),
        VOMITING(-8.0, true);

        private final double defaultValue;
        private final boolean isVoltage;
        private final Optional<Double> timeout;

        Goal(double defaultValue, boolean isVoltage) {
            this.defaultValue = defaultValue;
            this.isVoltage = isVoltage;
            this.timeout = Optional.empty();
        }
        Goal(double defaultValue, boolean isVoltage, double timeout) {
            this.defaultValue = defaultValue;
            this.isVoltage = isVoltage;
            this.timeout = Optional.of(timeout);
        }

        @Override
        public DoubleSupplier getValueSupplier() {
            return () -> defaultValue;
        }

        @Override
        public BooleanSupplier getIsVoltageSupplier() {
            return () -> isVoltage;
        }
    }

    private final LoggableTunedNumber idlingVoltage;
    private final LoggableTunedNumber jackhammerVoltage;
    private final LoggableTunedNumber shootingVoltage;
    private final LoggableTunedNumber vomitingVoltage;
    private final LoggableTunedNumber testingVoltage;
    private Goal goal = Goal.IDLING;

    public Kickup(KickupIO io) {
        this("Kickup", "Kickup", io);
    }

    public Kickup(String name, String tuningPrefix, KickupIO io) {
        super(name, io);
        idlingVoltage = new LoggableTunedNumber(
                tuningPrefix + "/IdlingVoltage", Goal.IDLING.getValueSupplier().getAsDouble(),
                Constants.TuningConstants.isTuningShooter);
        jackhammerVoltage = new LoggableTunedNumber(
                tuningPrefix + "/JackHammer", Goal.JACKHAMMER.getValueSupplier().getAsDouble(),
                Constants.TuningConstants.isTuningShooter);
        shootingVoltage = new LoggableTunedNumber(
                tuningPrefix + "/ShootingVoltage", Goal.SHOOTING.getValueSupplier().getAsDouble(),
                Constants.TuningConstants.isTuningShooter);
        vomitingVoltage = new LoggableTunedNumber(
                tuningPrefix + "/VomitingVoltage", Goal.VOMITING.getValueSupplier().getAsDouble(),
                Constants.TuningConstants.isTuningShooter);
        testingVoltage = new LoggableTunedNumber(
                tuningPrefix + "/TestingVoltage", Goal.TESTING.getValueSupplier().getAsDouble(),
                Constants.TuningConstants.isTuningShooter);
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    protected void subsystemPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        if (getGoal() != lastGoal) {
            stateTimer.reset();
            lastGoal = getGoal();
        }

        double output = getGoalValue();
        if (getGoal().getIsVoltageSupplier().getAsBoolean()) {
            if (getGoal().getTimeout().isPresent()) {
                double timeoutSec = getGoal().getTimeout().get();
                double cycleTimeSec = Timer.getFPGATimestamp() % (timeoutSec * 2.0);
                if (cycleTimeSec < timeoutSec) {
                    output = -output;
                }
            }
            io.runVolts(output);
        } else {
            io.runCurrent(output);
        }

        Logger.recordOutput("SuperStructure/" + name + "Goal", getGoal().toString());
        Logger.recordOutput("SuperStructure/" + name + "stateTimer", stateTimer.get());
    }

    private double getGoalValue() {
        return switch (goal) {
            case IDLING -> idlingVoltage.get();
            case JACKHAMMER -> jackhammerVoltage.get();
            case SHOOTING -> shootingVoltage.get();
            case VOMITING -> vomitingVoltage.get();
            case TESTING -> testingVoltage.get();
        };
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
                    if (Math.abs(getAppliedVolts() - shootingVoltage.get()) < .5) {
                        addFault(
                                "[System Check] idling voltage not reached for subsystem:"
                                        + getName(),
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.IDLING));
    }
}
