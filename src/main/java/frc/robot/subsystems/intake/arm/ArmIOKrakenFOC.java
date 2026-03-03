package frc.robot.subsystems.intake.arm;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.IntakeConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class ArmIOKrakenFOC implements ArmIO {
    private static final LoggableTunedNumber ZERO_VOLTS = new LoggableTunedNumber("Intake/Arm/zeroVolts",-3.5,TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber ZERO_CURRENT_AMPS = new LoggableTunedNumber("Intake/Arm/zeroAmps",25,TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber ZERO_HOLD_SEC = new LoggableTunedNumber("Intake/Arm/zeroTime",0.15,TuningConstants.isTuningIntake);
    private static final double TWO_PI = 2.0 * Math.PI;

    protected final String name;
    protected final TalonFX talon;
    protected final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    /* Direct Phoenix 6 control requests — no YAMS wrapper */
    private final MotionMagicVoltage motionMagicRequest =
            new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false).withOverrideBrakeDurNeutral(false);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false).withIgnoreHardwareLimits(true).withIgnoreSoftwareLimits(true).withLimitReverseMotion(false).withLimitForwardMotion(false).withOverrideBrakeDurNeutral(true);

    /** Motor rotor rotations per mechanism rotation. */
    protected final double reduction;

    protected final double minAngleRads = IntakeConstants.armMinAngleRads;
    protected final double maxAngleRads = IntakeConstants.armMaxAngleRads;

    protected final StatusSignal<Angle> posAngle;
    protected final StatusSignal<AngularVelocity> velAngle;
    protected final StatusSignal<Voltage> appliedVoltage;
    protected final StatusSignal<Current> supplyCurrent;
    protected final StatusSignal<Current> statorCurrent;
    protected final StatusSignal<Current> torqueCurrent;
    protected final StatusSignal<Temperature> tempCelsius;

    protected boolean zeroingActive = false;
    protected boolean openLoop = false;
    private double zeroSpikeStartTimeSec = Double.NaN;

    public ArmIOKrakenFOC(
            CANBus bus,
            int motorID,
            String name,
            int currentLimitAmps,
            boolean invert,
            boolean brake,
            double reduction) {
        this.name = name;
        this.reduction = reduction;

        talon = new TalonFX(motorID, bus);

        talonConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        talonConfig.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;

        talonConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talonConfig.MotorOutput.DutyCycleNeutralDeadband = 0;
        talonConfig.MotorOutput.Inverted = invert
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        talonConfig.Feedback.SensorToMechanismRatio = reduction;

        talonConfig.Slot0.kP = 0.0;
        talonConfig.Slot0.kI = 0.0;
        talonConfig.Slot0.kD = 0.0;
        talonConfig.Slot0.kS = 0.0;
        talonConfig.Slot0.kV = 0.0;
        talonConfig.Slot0.kA = 0.0;
        talonConfig.Slot0.kG = 0.0;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;  
        talonConfig.MotionMagic.MotionMagicAcceleration = 0.0;    

        talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.125;
        talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;

        talon.getConfigurator().apply(talonConfig);

        posAngle = talon.getPosition();
        velAngle = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        statorCurrent = talon.getStatorCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, posAngle, velAngle, appliedVoltage, supplyCurrent, statorCurrent, torqueCurrent, tempCelsius);
        talon.optimizeBusUtilization(0, 1.0);

        zero();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        processZeroing();
        inputs.connected = BaseStatusSignal.refreshAll(
                posAngle, velAngle, appliedVoltage, supplyCurrent, statorCurrent, torqueCurrent, tempCelsius)
                .isOK();
        inputs.name = name;
        inputs.zeroing = zeroingActive;
        // posAngle is mechanism rotations thanks to SensorToMechanismRatio
        inputs.positionRads = Units.rotationsToRadians(posAngle.getValueAsDouble());
        inputs.velocityRadsPerSec = Units.rotationsToRadians(velAngle.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        Logger.recordOutput("Intake/ZERO", openLoop);
    }

    @Override
    public void setPosition(double positionRads) {
        if (zeroingActive) {
            return;
        }

        openLoop = false;
        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
        double desiredRotations = Units.radiansToRotations(clamped);
        talon.setControl(motionMagicRequest.withPosition(desiredRotations));
    }

    @Override
    public void setVoltage(double volts) {
        if (zeroingActive){
            return;
        }
        openLoop = true;
        talon.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        talon.setControl(voltageRequest.withOutput(0.0));
    }

    @Override
    public void zero() {
        zeroingActive = true;
        zeroSpikeStartTimeSec = Double.NaN;
    }

    @Override
    public void configureMotionMagic(double cruiseRadPerSec, double accelRadPerSec2, double neutralDeadband) {
        talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseRadPerSec / TWO_PI;
        talonConfig.MotionMagic.MotionMagicAcceleration = accelRadPerSec2 / TWO_PI;
        talonConfig.MotorOutput.DutyCycleNeutralDeadband = neutralDeadband;
        talon.getConfigurator().apply(talonConfig.MotionMagic);
        talon.getConfigurator().apply(talonConfig.MotorOutput);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv, double kg) {
        talonConfig.Slot0.kP = p;
        talonConfig.Slot0.kI = i;
        talonConfig.Slot0.kD = d;
        talonConfig.Slot0.kS = ks;
        talonConfig.Slot0.kV = kv;
        talonConfig.Slot0.kG = kg;
        talon.getConfigurator().apply(talonConfig.Slot0);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv, double kg,
                       double velocityMax, double accelerationMax, double neutralDeadband) {
        talonConfig.Slot0.kP = p;
        talonConfig.Slot0.kI = i;
        talonConfig.Slot0.kD = d;
        talonConfig.Slot0.kS = ks;
        talonConfig.Slot0.kV = kv;
        talonConfig.Slot0.kG = kg;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = velocityMax / TWO_PI;
        talonConfig.MotionMagic.MotionMagicAcceleration = accelerationMax / TWO_PI;

        talonConfig.MotorOutput.DutyCycleNeutralDeadband = neutralDeadband;
        talon.getConfigurator().apply(talonConfig.Slot0);
        talon.getConfigurator().apply(talonConfig.MotionMagic);
        talon.getConfigurator().apply(talonConfig.MotorOutput);
    }

    @Override
    public void setCurrentLimit(double amps) {
        talonConfig.CurrentLimits.StatorCurrentLimit = amps;
        talonConfig.CurrentLimits.SupplyCurrentLimit = amps;
        talon.getConfigurator().apply(talonConfig.CurrentLimits);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        talonConfig.MotorOutput.NeutralMode =
                brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talon.getConfigurator().apply(talonConfig.MotorOutput);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        return List.of(new SelfCheckingTalonFX(name, talon));
    }

    private void processZeroing() {
        if (!zeroingActive) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        talon.setControl(voltageRequest.withOutput(ZERO_VOLTS.get()));
        BaseStatusSignal.refreshAll(supplyCurrent, statorCurrent, torqueCurrent);
        double observedCurrentAmps = Math.max(
                Math.max(Math.abs(supplyCurrent.getValueAsDouble()), Math.abs(statorCurrent.getValueAsDouble())),
                Math.abs(torqueCurrent.getValueAsDouble()));

        if (observedCurrentAmps >= ZERO_CURRENT_AMPS.get()) {
            if (Double.isNaN(zeroSpikeStartTimeSec)) {
                zeroSpikeStartTimeSec = now;
            }
        } else {
            zeroSpikeStartTimeSec = Double.NaN;
        }

        if (!Double.isNaN(zeroSpikeStartTimeSec) && (now - zeroSpikeStartTimeSec) >= ZERO_HOLD_SEC.get()) {
            talon.setPosition(0.0);
            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
            openLoop = false;
        }
    }
}
