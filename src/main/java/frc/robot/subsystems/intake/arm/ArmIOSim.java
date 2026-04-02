package frc.robot.subsystems.intake.arm;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.IntakeConstants;

public class ArmIOSim extends ArmIOKrakenFOC {

    private static final double ZERO_POSITION_EPSILON_RAD = Units.degreesToRadians(1.0);

    private final SingleJointedArmSim armSim;
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

        armSim = new SingleJointedArmSim(
                IntakeConstants.intakeArmMotor,
                IntakeConstants.intakeArmReduction,
                IntakeConstants.intakeMOI,
                IntakeConstants.intakeArmLengthMeters,
                IntakeConstants.armMinAngleRad,
                IntakeConstants.armMaxAngleRad,
                true,
                IntakeConstants.armMinAngleRad);
    }

    public ArmIOSim() {
        this(new CANBus("rio"), IntakeConstants.intakeMotorID, "IntakeArmIOSim");
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (zeroingActive) {
            processSimZeroing();
        }

        armSim.update(0.02);

        inputs.connected = true;
        inputs.name = name;
        inputs.zeroing = zeroingActive;
        inputs.positionRad = armSim.getAngleRads();
        inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.appliedVoltage = appliedVolts;
        inputs.supplyCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());
        inputs.torqueCurrentAmps = Math.abs(armSim.getCurrentDrawAmps());
        inputs.tempCelsius = 0.0;
    }

    @Override
    public void setPosition(double positionRad) {
        if (zeroingActive) {
            return;
        }

        openLoop = false;
        double clamped = MathUtil.clamp(positionRad, minPositionRad, maxPositionRad);
        double controlVolts = MathUtil.clamp(
                simPID.calculate(armSim.getAngleRads(), clamped),
                -12.0,
                12.0);
        appliedVolts = -controlVolts;
        armSim.setInputVoltage(controlVolts);
    }

    @Override
    public void setVoltage(double volts) {
        if (zeroingActive) {
            return;
        }

        openLoop = true;
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        armSim.setInputVoltage(-appliedVolts);
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
        armSim.setInputVoltage(0.0);
    }

    @Override
    public void zero() {
        super.zero();
        zeroSpikeStartTimeSec = Double.NaN;
        simPID.reset();
    }

    private void processSimZeroing() {
        double now = Timer.getFPGATimestamp();
        appliedVolts = getZeroingVoltage();
        armSim.setInputVoltage(-appliedVolts);

        boolean atLowerHardstop = armSim.getAngleRads()
                <= (IntakeConstants.armMinAngleRad + ZERO_POSITION_EPSILON_RAD);
        if (atLowerHardstop) {
            if (Double.isNaN(zeroSpikeStartTimeSec)) {
                zeroSpikeStartTimeSec = now;
            }
        } else {
            zeroSpikeStartTimeSec = Double.NaN;
        }

        if (!Double.isNaN(zeroSpikeStartTimeSec) && (now - zeroSpikeStartTimeSec) >= ZERO_HOLD_SEC.get()) {
            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
            fastRezeroActive = false;
            openLoop = false;
            appliedVolts = 0.0;
            armSim.setInputVoltage(0.0);
        }
    }
}
