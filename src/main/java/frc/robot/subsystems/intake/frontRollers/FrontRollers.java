package frc.robot.subsystems.intake.frontRollers;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
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
public class FrontRollers extends GenericRollerSystem<FrontRollers.Goal> {
    private enum OvercurrentProtectionMode {
        NORMAL,
        PULSE_OFF,
        PULSE_ON
    }

    @RequiredArgsConstructor
    @Getter
    public enum Goal implements GenericRollerSystem.RollGoalSupplier {
        STOPPED(() -> 0),
        IDLING(new LoggableTunedNumber("Intake/FrontRollers/IdlingVoltage", 0.0,
                Constants.TuningConstants.isTuningIntake)),
        INTAKING(
                new LoggableTunedNumber("Intake/FrontRollers/IntakingVoltage", 9.1,
                        Constants.TuningConstants.isTuningIntake)),
        SHOOTING(new LoggableTunedNumber("Intake/FrontRollers/ShootingVoltage", 10.0,
                Constants.TuningConstants.isTuningIntake)),
        VOMITING(new LoggableTunedNumber("Intake/FrontRollers/VomitingVoltage", -6.0,
                Constants.TuningConstants.isTuningIntake)),
        JACKHAMMER_IN(
                new LoggableTunedNumber("Intake/FrontRollers/JackhammerInAmps", 20.0,
                        Constants.TuningConstants.isTuningIntake),
                () -> false),
        JACKHAMMER_OUT(
                new LoggableTunedNumber("Intake/FrontRollers/JackhammerOutAmps", -20.0,
                        Constants.TuningConstants.isTuningIntake),
                () -> false);

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

    private Goal goal = Goal.INTAKING; // should always intake or vomit
    private final LoggableTunedNumber overcurrentThresholdAmps;
    private final LoggableTunedNumber overcurrentDelaySec;
    private final LoggableTunedNumber overcurrentPulseOnSec;
    private final LoggableTunedNumber overcurrentPulseOffSec;
    private OvercurrentProtectionMode overcurrentProtectionMode = OvercurrentProtectionMode.NORMAL;
    private double overcurrentStartSec = Double.NaN;
    private double protectionPhaseEndSec = Double.NaN;

    public FrontRollers(FrontRollersIO io) {
        super("Intake/FrontRollers", io);
        overcurrentThresholdAmps = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentThresholdAmps", 90.0,
                Constants.TuningConstants.isTuningIntake);
        overcurrentDelaySec = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentDelaySec", 0.5,
                Constants.TuningConstants.isTuningIntake);
        overcurrentPulseOnSec = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentPulseOnSec", 0.08,
                Constants.TuningConstants.isTuningIntake);
        overcurrentPulseOffSec = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentPulseOffSec", 0.06,
                Constants.TuningConstants.isTuningIntake);
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        if (getGoal() != lastGoal) {
            stateTimer.reset();
            lastGoal = getGoal();
            resetOvercurrentProtection();
        }

        Goal activeGoal = getGoal();
        double requestedOutput = activeGoal.getValueSupplier().getAsDouble();
        double appliedOutput = requestedOutput;

        if (activeGoal.getIsVoltageSupplier().getAsBoolean()) {
            appliedOutput = applyVoltageProtection(requestedOutput);
            io.runVolts(appliedOutput);
        } else {
            resetOvercurrentProtection();
            io.runCurrent(requestedOutput);
        }

        Logger.recordOutput("SuperStructure/" + name + "Goal", activeGoal.toString());
        Logger.recordOutput("SuperStructure/" + name + "stateTimer", stateTimer.get());
        Logger.recordOutput(name + "/ObservedCurrentAmps", getObservedCurrentAmps());
        Logger.recordOutput(name + "/OvercurrentProtectionMode", overcurrentProtectionMode.toString());
        Logger.recordOutput(name + "/OvercurrentTrackingSec",
                Double.isNaN(overcurrentStartSec) ? 0.0 : Timer.getFPGATimestamp() - overcurrentStartSec);
        Logger.recordOutput(name + "/RequestedVolts", requestedOutput);
        Logger.recordOutput(name + "/AppliedVolts", appliedOutput);
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

    private double applyVoltageProtection(double requestedVolts) {
        if (Math.abs(requestedVolts) < 1e-3) {
            resetOvercurrentProtection();
            return 0.0;
        }

        double now = Timer.getFPGATimestamp();
        double thresholdAmps = overcurrentThresholdAmps.get();
        double observedCurrentAmps = getObservedCurrentAmps();

        return switch (overcurrentProtectionMode) {
            case NORMAL -> {
                if (observedCurrentAmps >= thresholdAmps) {
                    if (Double.isNaN(overcurrentStartSec)) {
                        overcurrentStartSec = now;
                    }
                    if ((now - overcurrentStartSec) >= overcurrentDelaySec.get()) {
                        overcurrentProtectionMode = OvercurrentProtectionMode.PULSE_OFF;
                        protectionPhaseEndSec = now + overcurrentPulseOffSec.get();
                        yield 0.0;
                    }
                } else {
                    overcurrentStartSec = Double.NaN;
                }
                yield requestedVolts;
            }
            case PULSE_OFF -> {
                if (now >= protectionPhaseEndSec) {
                    overcurrentProtectionMode = OvercurrentProtectionMode.PULSE_ON;
                    protectionPhaseEndSec = now + overcurrentPulseOnSec.get();
                    yield requestedVolts;
                }
                yield 0.0;
            }
            case PULSE_ON -> {
                if (now >= protectionPhaseEndSec) {
                    if (observedCurrentAmps >= thresholdAmps) {
                        overcurrentProtectionMode = OvercurrentProtectionMode.PULSE_OFF;
                        protectionPhaseEndSec = now + overcurrentPulseOffSec.get();
                        yield 0.0;
                    }
                    resetOvercurrentProtection();
                }
                yield requestedVolts;
            }
        };
    }

    private double getObservedCurrentAmps() {
        return Math.max(Math.abs(inputs.supplyCurrentAmps), Math.abs(inputs.torqueCurrentAmps));
    }

    private void resetOvercurrentProtection() {
        overcurrentProtectionMode = OvercurrentProtectionMode.NORMAL;
        overcurrentStartSec = Double.NaN;
        protectionPhaseEndSec = Double.NaN;
    }
}
