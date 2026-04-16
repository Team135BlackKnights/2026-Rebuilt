package frc.robot.subsystems.intake.frontRollers;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.IntakeConstants;
import frc.robot.utils.LoggableTunedNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystem;

@Getter
@Setter
public class FrontRollers extends GenericRollerSystem<FrontRollers.Goal> {
    private static final LoggableTunedNumber velocity_kP =
            new LoggableTunedNumber("Intake/FrontRollers/kP", 0.098,
                    Constants.TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber velocity_kD =
            new LoggableTunedNumber("Intake/FrontRollers/kD", 0.0,
                    Constants.TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber velocity_kS =
            new LoggableTunedNumber("Intake/FrontRollers/kS", 0.0,
                    Constants.TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber velocity_kV =
            new LoggableTunedNumber("Intake/FrontRollers/kV", 0.115,
                    Constants.TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber velocity_kA =
            new LoggableTunedNumber("Intake/FrontRollers/kA", 0.0,
                    Constants.TuningConstants.isTuningIntake);

    private enum OvercurrentProtectionMode {
        NORMAL,
        PULSE_OFF,
        PULSE_ON
    }

    @RequiredArgsConstructor
    @Getter
    public enum Goal implements GenericRollerSystem.RollGoalSupplier {
        STOPPED(new LoggableTunedNumber("Intake/FrontRollers/StoppedSpeedMps", legacyVoltageToSurfaceSpeedMps(0.0),
                Constants.TuningConstants.isTuningIntake)),
        IDLING(new LoggableTunedNumber("Intake/FrontRollers/IdlingSpeedMps", legacyVoltageToSurfaceSpeedMps(0.0),
                Constants.TuningConstants.isTuningIntake)),
        INTAKING(new LoggableTunedNumber("Intake/FrontRollers/IntakingSpeedMps", legacyVoltageToSurfaceSpeedMps(9.1),
                        Constants.TuningConstants.isTuningIntake)),
        SHOOTING(new LoggableTunedNumber("Intake/FrontRollers/ShootingSpeedMps", legacyVoltageToSurfaceSpeedMps(10.0),
                Constants.TuningConstants.isTuningIntake)),
        VOMITING(new LoggableTunedNumber("Intake/FrontRollers/VomitingSpeedMps", legacyVoltageToSurfaceSpeedMps(-6.0),
                Constants.TuningConstants.isTuningIntake)),
        JACKHAMMER_IN(new LoggableTunedNumber("Intake/FrontRollers/JackhammerInSpeedMps", legacyVoltageToSurfaceSpeedMps(6.0),
                Constants.TuningConstants.isTuningIntake)),
        JACKHAMMER_OUT(new LoggableTunedNumber("Intake/FrontRollers/JackhammerOutSpeedMps", legacyVoltageToSurfaceSpeedMps(-6.0),
                Constants.TuningConstants.isTuningIntake));

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
            this.isVoltageSupplier = () -> false;
            this.timeout = Optional.empty();
        }

        Goal(LoggableTunedNumber valueSupplier, BooleanSupplier isVoltageSupplier) {
            this.valueSupplier = valueSupplier::get;
            this.isVoltageSupplier = isVoltageSupplier::getAsBoolean;
            this.timeout = Optional.empty();
        }
    }

    private static double legacyVoltageToSurfaceSpeedMps(double volts) {
        double rollerFreeSpeedMps =
                (DCMotor.getKrakenX44Foc(1).freeSpeedRadPerSec / IntakeConstants.frontRollersReduction)
                        * (IntakeConstants.rollersDiameterMeters / 2.0);
        return rollerFreeSpeedMps * (volts / 12.0);
    }

    private Goal goal = Goal.INTAKING; // should always intake or vomit
    private final LoggableTunedNumber overcurrentThresholdAmps;
    private final LoggableTunedNumber overcurrentDelaySec;
    private final LoggableTunedNumber overcurrentPulseOnSec;
    private final LoggableTunedNumber overcurrentPulseOffSec;
    private final FrontRollersIO frontRollersIO;
    private OvercurrentProtectionMode overcurrentProtectionMode = OvercurrentProtectionMode.NORMAL;
    private double overcurrentStartSec = Double.NaN;
    private double protectionPhaseEndSec = Double.NaN;

    public FrontRollers(FrontRollersIO io) {
        super("Intake/FrontRollers", io);
        this.frontRollersIO = io;
        overcurrentThresholdAmps = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentThresholdAmps", 90.0,
                Constants.TuningConstants.isTuningIntake);
        overcurrentDelaySec = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentDelaySec", 0.5,
                Constants.TuningConstants.isTuningIntake);
        overcurrentPulseOnSec = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentPulseOnSec", 0.08,
                Constants.TuningConstants.isTuningIntake);
        overcurrentPulseOffSec = new LoggableTunedNumber("Intake/FrontRollers/OvercurrentPulseOffSec", 0.06,
                Constants.TuningConstants.isTuningIntake);
        updateTunableVelocityControl();
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    protected void subsystemPeriodic() {
        frontRollersIO.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        updateTunableVelocityControl();
        if (getGoal() != lastGoal) {
            stateTimer.reset();
            lastGoal = getGoal();
            resetOvercurrentProtection();
        }

        Goal activeGoal = getGoal();
        double requestedSurfaceSpeedMps = getRequestedSurfaceSpeedMps(activeGoal);
        double appliedSurfaceSpeedMps = applySurfaceSpeedProtection(requestedSurfaceSpeedMps);
        double requestedVelocityRadsPerSec = surfaceSpeedMpsToRollerRadPerSec(requestedSurfaceSpeedMps);
        double appliedVelocityRadsPerSec = surfaceSpeedMpsToRollerRadPerSec(appliedSurfaceSpeedMps);
        double fallbackAppliedVolts = 0.0;

        if (frontRollersIO.supportsVelocityControl()) {
            frontRollersIO.setVelocity(appliedVelocityRadsPerSec);
        } else {
            fallbackAppliedVolts = surfaceSpeedMpsToFallbackVolts(appliedSurfaceSpeedMps);
            frontRollersIO.runVolts(fallbackAppliedVolts);
        }

        Logger.recordOutput("SuperStructure/" + name + "Goal", activeGoal.toString());
        Logger.recordOutput("SuperStructure/" + name + "stateTimer", stateTimer.get());
        Logger.recordOutput(name + "/ObservedCurrentAmps", getObservedCurrentAmps());
        Logger.recordOutput(name + "/OvercurrentProtectionMode", overcurrentProtectionMode.toString());
        Logger.recordOutput(name + "/OvercurrentTrackingSec",
                Double.isNaN(overcurrentStartSec) ? 0.0 : Timer.getFPGATimestamp() - overcurrentStartSec);
        Logger.recordOutput(name + "/VelocityModeActive", frontRollersIO.supportsVelocityControl());
        Logger.recordOutput(name + "/RequestedVelocityRadsPerSec", requestedVelocityRadsPerSec);
        Logger.recordOutput(name + "/AppliedVelocityRadsPerSec", appliedVelocityRadsPerSec);
        Logger.recordOutput(name + "/ObservedVelocityRadsPerSec", inputs.velocityRadsPerSec);
        Logger.recordOutput(name + "/RequestedSurfaceSpeedMps", requestedSurfaceSpeedMps);
        Logger.recordOutput(name + "/AppliedSurfaceSpeedMps", appliedSurfaceSpeedMps);
        Logger.recordOutput(name + "/ObservedSurfaceSpeedMps", rollerRadPerSecToSurfaceSpeedMps(inputs.velocityRadsPerSec));
        Logger.recordOutput(name + "/FallbackAppliedVolts", fallbackAppliedVolts);
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
                    double requestedVelocity = surfaceSpeedMpsToRollerRadPerSec(getRequestedSurfaceSpeedMps(Goal.INTAKING));
                    if (Math.abs(inputs.velocityRadsPerSec) < Math.abs(requestedVelocity) * 0.5) {
                        addFault(
                                "[System Check] Intake roller velocity not reached for subsystem:"
                                        + getName(),
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.STOPPED));
    }

    private void updateTunableVelocityControl() {
        LoggableTunedNumber.ifChanged(hashCode(), () -> frontRollersIO.setPID(
                velocity_kP.get(),
                velocity_kD.get(),
                velocity_kS.get(),
                velocity_kV.get(),
                velocity_kA.get()),
                velocity_kP, velocity_kD, velocity_kS, velocity_kV, velocity_kA);
    }

    private double getRequestedSurfaceSpeedMps(Goal goal) {
        return goal.getValueSupplier().getAsDouble();
    }

    private double applySurfaceSpeedProtection(double requestedSurfaceSpeedMps) {
        if (Math.abs(requestedSurfaceSpeedMps) < 1e-3) {
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
                yield requestedSurfaceSpeedMps;
            }
            case PULSE_OFF -> {
                if (now >= protectionPhaseEndSec) {
                    overcurrentProtectionMode = OvercurrentProtectionMode.PULSE_ON;
                    protectionPhaseEndSec = now + overcurrentPulseOnSec.get();
                    yield requestedSurfaceSpeedMps;
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
                yield requestedSurfaceSpeedMps;
            }
        };
    }

    private static double surfaceSpeedMpsToRollerRadPerSec(double surfaceSpeedMps) {
        return surfaceSpeedMps / (IntakeConstants.rollersDiameterMeters / 2.0);
    }

    private static double surfaceSpeedMpsToFallbackVolts(double surfaceSpeedMps) {
        double requestedRadPerSec = surfaceSpeedMpsToRollerRadPerSec(surfaceSpeedMps);
        double maxRollerRadPerSec = surfaceSpeedMpsToRollerRadPerSec(12.0);
        if (maxRollerRadPerSec <= 1e-9) {
            return 0.0;
        }
        return 12.0 * (requestedRadPerSec / maxRollerRadPerSec);
    }

    private static double rollerRadPerSecToSurfaceSpeedMps(double rollerRadPerSec) {
        return rollerRadPerSec * (IntakeConstants.rollersDiameterMeters / 2.0);
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
