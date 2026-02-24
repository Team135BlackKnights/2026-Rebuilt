package frc.robot.subsystems.intake.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.IntakeConstants;
import frc.robot.utils.YAMS.GearBox;
import frc.robot.utils.YAMS.MechanismGearing;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.SmartMotorControllerConfig;
import frc.robot.utils.YAMS.TalonFXWrapper;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;
import frc.robot.utils.YAMS.mechanisms.ArmConfig;
import frc.robot.utils.YAMS.mechanisms.Containers.Arm;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class ArmIOKrakenFOC implements ArmIO {
    private static final double ZERO_HOMING_VOLTAGE = -2.0;
    private static final double ZERO_SPIKE_CURRENT_AMPS = 20.0;
    private static final double ZERO_SPIKE_HOLD_TIME_SEC = 0.15;

    protected final String name;

    protected final TalonFX talon;
    protected final SmartMotorControllerConfig motorConfig;
    protected final SmartMotorController motor;
    protected final ArmConfig armConfig;
    protected final Arm arm;

    protected final double minAngleRads = IntakeConstants.armMinAngleRads;
    protected final double maxAngleRads = IntakeConstants.armMaxAngleRads;

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

        talon = new TalonFX(motorID, bus);

        motorConfig = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(
                        0.0, 0.0, 0.0, DegreesPerSecond.of(999), DegreesPerSecondPerSecond.of(999))
                .withSimClosedLoopController(
                        12, 0.0, 1.85, DegreesPerSecond.of(999), DegreesPerSecondPerSecond.of(999))
                .withFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
                .withSimFeedforward(new ArmFeedforward(0.0, 0.0, 0.0, 0.0))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(reduction)))
                .withMotorInverted(invert)
                .withIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(currentLimitAmps))
                .withSupplyCurrentLimit(Amps.of(currentLimitAmps))
                .withClosedLoopRampRate(Seconds.of(0.0))
                .withOpenLoopRampRate(Seconds.of(0.0));

        motor = new TalonFXWrapper(talon, DCMotor.getKrakenX44Foc(1), motorConfig);

        armConfig = new ArmConfig(motor)
                .withGravity(false)
                .withLength(Inches.of(7))
                .withMOI(KilogramSquareMeters.of(IntakeConstants.armMOI))
                .withHardLimit(Radians.of(minAngleRads), Radians.of(maxAngleRads))
                .withStartingPosition(Radians.of(IntakeConstants.armMaxAngleRads));
        arm = new Arm(armConfig);

        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        statorCurrent = talon.getStatorCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, appliedVoltage, supplyCurrent, statorCurrent, torqueCurrent, tempCelsius);
        talon.optimizeBusUtilization(0, 1.0);

        zero();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        processZeroing();
        inputs.connected = BaseStatusSignal.refreshAll(
                appliedVoltage, supplyCurrent, statorCurrent, torqueCurrent, tempCelsius)
                .isOK();
        inputs.name = name;
        inputs.zeroing = zeroingActive;
        inputs.positionRads = arm.getAngle().in(Radians);
        inputs.velocityRadsPerSec = motor.getMechanismVelocity().in(RadiansPerSecond);
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void setPosition(double positionRads) {
        if (zeroingActive || openLoop) {
            return;
        }
        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
        arm.setMechanismPositionSetpoint(Radians.of(clamped));
    }

    @Override
    public void setVoltage(double volts) {
        if (zeroingActive) {
            return;
        }
        openLoop = true;
        arm.setVolts(volts);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void zero() {
        zeroingActive = true;
        zeroSpikeStartTimeSec = Double.NaN;
    }

    @Override
    public void configureMotionMagic(double cruiseRadPerSec, double accelRadPerSec2, double jerkRadPerSec3) {
        motor.setMotionProfileMaxVelocity(RadiansPerSecond.of(cruiseRadPerSec));
        motor.setMotionProfileMaxAcceleration(RadiansPerSecondPerSecond.of(accelRadPerSec2));
        motor.setMotionProfileMaxJerk(RadiansPerSecondPerSecond.per(Seconds).of(jerkRadPerSec3));
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv, double kg) {
        motor.setFeedback(p, i, d);
        motor.setFeedforward(ks, kv, 0.0, kg);
    }

    @Override
    public void setCurrentLimit(double amps) {
        motor.setSupplyCurrentLimit(Amps.of(amps));
        motor.setStatorCurrentLimit(Amps.of(amps));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
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
        motor.setVoltage(Volts.of(ZERO_HOMING_VOLTAGE));
        BaseStatusSignal.refreshAll(supplyCurrent, statorCurrent, torqueCurrent);
        double observedCurrentAmps = Math.max(
                Math.max(Math.abs(supplyCurrent.getValueAsDouble()), Math.abs(statorCurrent.getValueAsDouble())),
                Math.abs(torqueCurrent.getValueAsDouble()));

        if (observedCurrentAmps >= ZERO_SPIKE_CURRENT_AMPS) {
            if (Double.isNaN(zeroSpikeStartTimeSec)) {
                zeroSpikeStartTimeSec = now;
            }
        } else {
            zeroSpikeStartTimeSec = Double.NaN;
        }

        if (!Double.isNaN(zeroSpikeStartTimeSec) && (now - zeroSpikeStartTimeSec) >= ZERO_SPIKE_HOLD_TIME_SEC) {
            motor.setEncoderPosition(Radians.zero());
            motor.setVoltage(Volts.zero());
            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
        }
    }
}
