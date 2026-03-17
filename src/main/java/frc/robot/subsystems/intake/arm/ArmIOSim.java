package frc.robot.subsystems.intake.arm;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.IntakeConstants;

public class ArmIOSim extends ArmIOKrakenFOC {

    private static final double ZERO_POSITION_EPSILON_INCHES = 0.05;

    private final ElevatorSim slideSim;
    private final PIDController simPID = new PIDController(6.0, 0.0, 0.25);
    private double appliedVolts = 0.0;
    private double zeroSpikeStartTimeSec = Double.NaN;

    public ArmIOSim(CANBus bus, int motorID, String name) {
        super(
                bus,
                motorID,
                name,
                IntakeConstants.intakeCurrentLimit,
                IntakeConstants.intakeInverted,
                true,
                IntakeConstants.intakeArmReduction);

        slideSim = new ElevatorSim(
                DCMotor.getKrakenX44Foc(1),
                IntakeConstants.intakeArmReduction,
                IntakeConstants.slideCarriageMassKg,
                Units.inchesToMeters(IntakeConstants.slideInchesPerMechanismRotation / (2.0 * Math.PI)),
                Units.inchesToMeters(IntakeConstants.slideMinInches),
                Units.inchesToMeters(IntakeConstants.slideMaxInches),
                false,
                Units.inchesToMeters(IntakeConstants.slideMinInches));
    }

    public ArmIOSim() {
        this(new CANBus("rio"), IntakeConstants.intakeMotorID, "IntakeArmIOSim");
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (zeroingActive) {
            processSimZeroing();
        }

        slideSim.update(0.02);

        inputs.connected = true;
        inputs.name = name;
        inputs.zeroing = zeroingActive;
        inputs.positionInches = Units.metersToInches(slideSim.getPositionMeters());
        inputs.velocityInchesPerSec = Units.metersToInches(slideSim.getVelocityMetersPerSecond());
        inputs.appliedVoltage = appliedVolts;
        inputs.supplyCurrentAmps = Math.abs(slideSim.getCurrentDrawAmps());
        inputs.torqueCurrentAmps = Math.abs(slideSim.getCurrentDrawAmps());
        inputs.tempCelsius = 0.0;
    }

    @Override
    public void setPosition(double positionInches) {
        if (zeroingActive) {
            return;
        }

        openLoop = false;
        double clamped = MathUtil.clamp(positionInches, minPositionInches, maxPositionInches);
        appliedVolts = MathUtil.clamp(
                simPID.calculate(slideSim.getPositionMeters(), Units.inchesToMeters(clamped)),
                -12.0,
                12.0);
        slideSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setVoltage(double volts) {
        if (zeroingActive) {
            return;
        }

        openLoop = true;
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        slideSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
        slideSim.setInputVoltage(0.0);
    }

    @Override
    public void zero() {
        zeroingActive = true;
        zeroSpikeStartTimeSec = Double.NaN;
    }

    private void processSimZeroing() {
        double now = Timer.getFPGATimestamp();
        appliedVolts = ZERO_VOLTS.get();
        slideSim.setInputVoltage(appliedVolts);

        boolean atLowerHardstop = Units.metersToInches(slideSim.getPositionMeters())
                <= (IntakeConstants.slideMinInches + ZERO_POSITION_EPSILON_INCHES);
        if (atLowerHardstop) {
            if (Double.isNaN(zeroSpikeStartTimeSec)) {
                zeroSpikeStartTimeSec = now;
            }
        } else {
            zeroSpikeStartTimeSec = Double.NaN;
        }

        if (!Double.isNaN(zeroSpikeStartTimeSec) && (now - zeroSpikeStartTimeSec) >= ZERO_HOLD_SEC.get()) {
            slideSim.setState(Units.inchesToMeters(IntakeConstants.slideMinInches), 0.0);
            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
            openLoop = false;
            appliedVolts = 0.0;
            slideSim.setInputVoltage(0.0);
        }
    }
}
