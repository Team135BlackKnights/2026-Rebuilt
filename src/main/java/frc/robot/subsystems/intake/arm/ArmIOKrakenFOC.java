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
    protected static final LoggableTunedNumber ZERO_VOLTS = new LoggableTunedNumber("Intake/Arm/zeroVolts",7,TuningConstants.isTuningIntake);
    protected static final LoggableTunedNumber FAST_REZERO_VOLTS = new LoggableTunedNumber("Intake/Arm/fastRezeroVolts",10,TuningConstants.isTuningIntake);
    protected static final LoggableTunedNumber ZERO_CURRENT_AMPS = new LoggableTunedNumber("Intake/Arm/zeroAmps",18,TuningConstants.isTuningIntake);
    protected static final LoggableTunedNumber ZERO_HOLD_SEC = new LoggableTunedNumber("Intake/Arm/zeroTime",.4,TuningConstants.isTuningIntake);
    protected static final LoggableTunedNumber FAST_REZERO_WINDOW_SEC = new LoggableTunedNumber("Intake/Arm/fastRezeroWindowSec",2.0,TuningConstants.isTuningIntake);

    protected final String name;
    protected final TalonFX talon;
    protected final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    /* Direct Phoenix 6 control requests — no YAMS wrapper */
    private final MotionMagicVoltage motionMagicRequest =
            new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false).withOverrideBrakeDurNeutral(false);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false).withIgnoreHardwareLimits(true).withIgnoreSoftwareLimits(true).withLimitReverseMotion(false).withLimitForwardMotion(false).withOverrideBrakeDurNeutral(true);

    /** Motor rotor rotations per mechanism rotation. */
    protected final double reduction;

    protected final double minPositionRad = IntakeConstants.armMinAngleRad;
    protected final double maxPositionRad = IntakeConstants.armMaxAngleRad;
    protected final double radiansPerMechanismRotation = IntakeConstants.armRadiansPerMechanismRotation;

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
    protected boolean fastRezeroActive = false;
    protected double lastZeroRequestTimeSec = Double.NEGATIVE_INFINITY;

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

        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
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
        startZeroing(Timer.getFPGATimestamp());
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        processZeroing();
        inputs.connected = BaseStatusSignal.refreshAll(
                posAngle, velAngle, appliedVoltage, supplyCurrent, statorCurrent, torqueCurrent, tempCelsius)
                .isOK();
        inputs.name = name;
        inputs.zeroing = zeroingActive;
        inputs.positionRad = mechanismRotationsToRadians(posAngle.getValueAsDouble());
        inputs.velocityRadPerSec = mechanismRotationsToRadians(velAngle.getValueAsDouble());
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        Logger.recordOutput("Intake/ZERO", openLoop);
        Logger.recordOutput("Intake/Arm/FastRezeroActive", fastRezeroActive);
        Logger.recordOutput("Intake/Arm/ZeroingVoltage", getZeroingVoltage());
    }

    @Override
    public void setPosition(double positionRad) {
        if (zeroingActive) {
            return;
        }

        openLoop = false;
        double clamped = MathUtil.clamp(positionRad, minPositionRad, maxPositionRad);
        double desiredRotations = radiansToMechanismRotations(clamped);
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
        startZeroing(Timer.getFPGATimestamp());
    }

    @Override
    public void configureMotionMagic(double cruiseRadPerSec, double accelRadPerSec2, double neutralDeadband) {
        talonConfig.MotionMagic.MotionMagicCruiseVelocity = radiansToMechanismRotationsMagnitude(cruiseRadPerSec);
        talonConfig.MotionMagic.MotionMagicAcceleration = radiansToMechanismRotationsMagnitude(accelRadPerSec2);
        talonConfig.MotorOutput.DutyCycleNeutralDeadband = neutralDeadband;
        talon.getConfigurator().apply(talonConfig.MotionMagic);
        talon.getConfigurator().apply(talonConfig.MotorOutput);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv, double kg) {
        talonConfig.Slot0.kP = scaleAngularGain(p);
        talonConfig.Slot0.kI = scaleAngularGain(i);
        talonConfig.Slot0.kD = scaleAngularGain(d);
        talonConfig.Slot0.kS = ks;
        talonConfig.Slot0.kV = scaleAngularVelocityGain(kv);
        talonConfig.Slot0.kG = kg;
        talon.getConfigurator().apply(talonConfig.Slot0);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv, double kg,
                       double velocityMax, double accelerationMax, double neutralDeadband) {
        talonConfig.Slot0.kP = scaleAngularGain(p);
        talonConfig.Slot0.kI = scaleAngularGain(i);
        talonConfig.Slot0.kD = scaleAngularGain(d);
        talonConfig.Slot0.kS = ks;
        talonConfig.Slot0.kV = scaleAngularVelocityGain(kv);
        talonConfig.Slot0.kG = kg;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = radiansToMechanismRotationsMagnitude(velocityMax);
        talonConfig.MotionMagic.MotionMagicAcceleration = radiansToMechanismRotationsMagnitude(accelerationMax);

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

    protected double mechanismRotationsToRadians(double mechanismRotations) {
        return -Units.rotationsToRadians(mechanismRotations);
    }

    protected double radiansToMechanismRotations(double radians) {
        return -Units.radiansToRotations(radians);
    }

    protected double radiansToMechanismRotationsMagnitude(double radians) {
        return Units.radiansToRotations(Math.abs(radians));
    }

    private double scaleAngularGain(double angularGain) {
        return angularGain * radiansPerMechanismRotation;
    }

    private double scaleAngularVelocityGain(double angularVelocityGain) {
        return angularVelocityGain * Units.rotationsToRadians(1.0);
    }

    // Avoid calling an overridable method from the constructor.
    private void startZeroing(double now) {
        fastRezeroActive =
                Double.isFinite(lastZeroRequestTimeSec)
                        && (now - lastZeroRequestTimeSec) <= FAST_REZERO_WINDOW_SEC.get();
        lastZeroRequestTimeSec = now;
        zeroingActive = true;
        zeroSpikeStartTimeSec = Double.NaN;
    }

    private void processZeroing() {
        if (!zeroingActive) {
            return;
        }
        /*talon.setPosition(radiansToMechanismRotations(minPositionRad));
            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
            fastRezeroActive = false;
            openLoop = false;
        return;*/
            double now = Timer.getFPGATimestamp();
        talon.setControl(voltageRequest.withOutput(getZeroingVoltage()));
        BaseStatusSignal.refreshAll(supplyCurrent, statorCurrent, torqueCurrent);
        double observedCurrentAmps = Math.abs(torqueCurrent.getValueAsDouble());

        if (observedCurrentAmps >= ZERO_CURRENT_AMPS.get()) {
            if (Double.isNaN(zeroSpikeStartTimeSec)) {
                zeroSpikeStartTimeSec = now;
            }
        } else {
            zeroSpikeStartTimeSec = Double.NaN;
        }

        if (!Double.isNaN(zeroSpikeStartTimeSec) && (now - zeroSpikeStartTimeSec) >= ZERO_HOLD_SEC.get()) {
            talon.setPosition(radiansToMechanismRotations(minPositionRad));
            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
            fastRezeroActive = false;
            openLoop = false;
        }
    }

    protected double getZeroingVoltage() {
        return fastRezeroActive ? FAST_REZERO_VOLTS.get() : ZERO_VOLTS.get();
    }
}
