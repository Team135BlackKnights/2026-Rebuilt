package frc.robot.subsystems.Turret.azimuth;

import java.util.ArrayList;
import java.util.List;

import com.andymark.jni.AM_CAN_Mag_Switch;
import com.andymark.jni.AM_CAN_Mag_Switch.AM_MagSwitchData;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class AzimuthIOKrakenFOC implements AzimuthIO {

    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double ENCODER_UPDATE_HZ = 100.0;
    private static final double MOTOR_UPDATE_HZ = 100.0;
    private static final double LEFT_HOME_DIRECTION = -1.0;
    private static final double RIGHT_HOME_DIRECTION = 1.0;
    private static final double MAG_SWITCH_RECENT_CONTACT_SEC = 2.0;

    private final TalonFX talon;
    private final CANcoder canCoderBig;
    private final AM_CAN_Mag_Switch magSwitch;


    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final StatusSignal<Angle> motorRotorRots;
    private final StatusSignal<AngularVelocity> motorRotorVelocityRotsPerSec;

    private final StatusSignal<Angle> bigAbsRots;
    private final StatusSignal<AngularVelocity> bigAbsRotsVel;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private boolean haveLock = false;
    private boolean allowMovement = true;
    private boolean zeroingRequested = true;
    private final String name;
    private final double minAngle;
    private final double maxAngle;

    private final double turretSign;
    private final boolean rightTurret;
    private final double motorToTurretRatio;
    private final double primaryEncoderOffsetRotations;
    private final double primaryEncoderRatio;
    private final double homingDirection;
    private final LoggableTunedNumber homingVolts;

    private double lastTurretAngleRads = 0.0;
    private double lastMagSwitchContactSec = Double.NEGATIVE_INFINITY;
    private String lastRezeroRequestResult = "NONE";

    public AzimuthIOKrakenFOC(
            CANBus bus,
            int ID,
            int canCoderBigID,
            int canCoderSmallID,
            int magID,
            String name,
            int currentLimitAmps,
            double minTurretAngle,
            double maxTurretAngle,
            double encoder1Offset,
            double encoder2Offset,
            double enc1GearTeeth,
            double enc2GearTeeth,
            double motorToTurretRatio) {

        this.name = name;
        this.minAngle = minTurretAngle;
        this.maxAngle = maxTurretAngle;

        this.rightTurret = canCoderSmallID == AdvancedMechanismConstants.Turret.rightAzimuthBigEncoderID;
        this.turretSign = rightTurret ? 1.0 : 1.0;
        this.motorToTurretRatio = motorToTurretRatio;
        this.homingDirection = rightTurret ? RIGHT_HOME_DIRECTION : LEFT_HOME_DIRECTION;

        final double turretToIdlerRatio = (double) AdvancedMechanismConstants.Turret.turretTeeth
                / (double) AdvancedMechanismConstants.Turret.idlerTeeth;
        this.primaryEncoderOffsetRotations = encoder1Offset;
        this.primaryEncoderRatio = -turretToIdlerRatio;
        this.homingVolts = new LoggableTunedNumber(
                name + "/Turret/HomingVolts",
                1,
                TuningConstants.isTuningShooter);

        talon = new TalonFX(ID, bus);
        canCoderBig = new CANcoder(canCoderBigID, bus);
        magSwitch = new AM_CAN_Mag_Switch(magID);
        magSwitch.setReportPeriod(5);

        /* ---- CANcoder configs ---- */
        /*
         * CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
         * CANcoderConfiguration encoder2Config = new CANcoderConfiguration();
         * 
         * encoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
         * encoder2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
         * 
         * //encoder1Config.MagnetSensor.MagnetOffset = encoder1Offset;
         * //encoder2Config.MagnetSensor.MagnetOffset = encoder2Offset;
         * 
         * encoder1Config.MagnetSensor.SensorDirection =
         * SensorDirectionValue.Clockwise_Positive;
         * //encoder2Config.MagnetSensor.SensorDirection =
         * SensorDirectionValue.Clockwise_Positive;
         * 
         * canCoderBig.getConfigurator().apply(encoder1Config);
         * canCoderSmall.getConfigurator().apply(encoder2Config);
         */

        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfig.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;

        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        talonConfig.Slot0.kP = 0.0;
        talonConfig.Slot0.kI = 0.0;
        talonConfig.Slot0.kD = 0.0;
        talonConfig.Slot0.kS = 0.0;
        talonConfig.Slot0.kV = 0.0;
        talonConfig.Slot0.kA = 0.0;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = 12.0 * motorToTurretRatio / TWO_PI;
        talonConfig.MotionMagic.MotionMagicAcceleration = 40.0 * motorToTurretRatio / TWO_PI;
        talonConfig.MotionMagic.MotionMagicJerk = 0; // trapezoidal

        talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
        talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;

        // No gear ratio on kraken we command in raw rotor rotations cuz we can
        talonConfig.Feedback.SensorToMechanismRatio = 1.0;

        talon.getConfigurator().apply(talonConfig);

        motorRotorRots = talon.getRotorPosition();
        motorRotorVelocityRotsPerSec = talon.getRotorVelocity();

        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        bigAbsRots = canCoderBig.getAbsolutePosition();
        bigAbsRotsVel = canCoderBig.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                ENCODER_UPDATE_HZ, bigAbsRots, bigAbsRotsVel);
        BaseStatusSignal.setUpdateFrequencyForAll(
                MOTOR_UPDATE_HZ,
                motorRotorRots,
                motorRotorVelocityRotsPerSec,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius);

        talon.optimizeBusUtilization(0, 1.0);
    }

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
        final double nowSec = Timer.getFPGATimestamp();

        inputs.motorConnected = BaseStatusSignal.refreshAll(
                motorRotorRots,
                motorRotorVelocityRotsPerSec,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius).isOK();

        inputs.referenceEncoderConnected = BaseStatusSignal.refreshAll(bigAbsRots, bigAbsRotsVel).isOK();
        inputs.bothEncodersConnected = inputs.referenceEncoderConnected;
        inputs.name = name;
        AM_MagSwitchData data = magSwitch.getData(1);
        final boolean magSwitchDetected = data.magnetDetected;
        final int magSwitchTime = data.timeStamp;
        lastMagSwitchContactSec = magSwitchTime;
        final boolean magSwitchRecentlySeen = (nowSec - lastMagSwitchContactSec) <= MAG_SWITCH_RECENT_CONTACT_SEC;
        final boolean effectiveMagSwitchDetected = magSwitchDetected;
        inputs.magSwitchDetected = magSwitchDetected;
        final double currentRotorRots = motorRotorRots.getValueAsDouble();
        inputs.motorPositionRads = Units.rotationsToRadians(currentRotorRots);
        inputs.motorVelocityRadsPerSec = Units.rotationsToRadians(motorRotorVelocityRotsPerSec.getValueAsDouble());

        double bigRawRads = MathUtil.inputModulus(
                Units.rotationsToRadians(bigAbsRots.getValueAsDouble()), 0.0, TWO_PI);
     
        inputs.bigEncoderRads = bigRawRads;

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        final double motorVelRotsPerSec = motorRotorVelocityRotsPerSec.getValueAsDouble();
        final double turretVelRadsPerSec = Units.rotationsToRadians(motorVelRotsPerSec) * turretSign
                / motorToTurretRatio;
        final boolean wantsZeroing = zeroingRequested || !haveLock;
        final boolean zeroingReady = wantsZeroing
                && inputs.motorConnected
                && inputs.referenceEncoderConnected;
        final boolean shouldUseMagRangeReference = inputs.referenceEncoderConnected
                && effectiveMagSwitchDetected;
        final boolean shouldHomeToMag = zeroingReady
                && DriverStation.isEnabled()
                && !haveLock
                && !magSwitchDetected;
        final boolean shouldSolveFromEncoder = zeroingReady
                && !haveLock
                && shouldUseMagRangeReference;
        final double homingCommandVolts = homingVolts.get() * homingDirection;
        if (shouldUseMagRangeReference) {
            final double solvedRad = MathUtil.clamp(
                    primaryEncoderAngleToTurretRads(bigAbsRots.getValueAsDouble()), minAngle, maxAngle);
            final double seededRotorRots = turretToRotorRotations(solvedRad);
            talon.setPosition(seededRotorRots);
            lastTurretAngleRads = solvedRad;
            haveLock = true;
            if (shouldSolveFromEncoder) {
                zeroingRequested = false;
                lastRezeroRequestResult = "SOLVED";
            }
        } else if (haveLock) {
            lastTurretAngleRads = rotorRotationsToTurretRads(currentRotorRots);
        }

        inputs.turretPositionRads = lastTurretAngleRads;
        inputs.turretVelocityRadsPerSec = turretVelRadsPerSec;
        inputs.zeroed = haveLock;
        inputs.zeroingState =
                shouldSolveFromEncoder
                        ? "MAG_ENCODER_SOLVE"
                        : (shouldUseMagRangeReference
                                ? "MAG_ENCODER_TRACK"
                                : (shouldHomeToMag
                                        ? "HOMING_TO_MAG"
                                        : (haveLock
                                                ? "MOTOR_ONLY"
                                                : (!wantsZeroing
                                                        ? "ZEROING_DISABLED"
                                                        : (!inputs.motorConnected
                                                                ? "WAITING_FOR_MOTOR"
                                                                : (!inputs.referenceEncoderConnected
                                                                        ? "WAITING_FOR_ENCODER"
                                                                        : (DriverStation.isDisabled()
                                                                                ? "WAITING_FOR_ENABLE"
                                                                                : "WAITING_FOR_MAG_SWITCH")))))));
        if (shouldHomeToMag) {
            talon.setControl(voltageRequest.withOutput(homingCommandVolts));
        } else if (!allowMovement) {
            stop();
        }
    }

    /**
     * The desired turret angle (radians) is converted to an absolute motor rotor
     * position (rotations) using the offset established at lock.
     */
    @Override
    public void setDesiredPosition(double turretRads) {
        if (!haveLock) {
            return;
        }

        double lower = maxAngle - TWO_PI;
        double upper = maxAngle;

        double wrappedCmd = MathUtil.inputModulus(turretRads, lower, upper);
        double desiredTurret = MathUtil.clamp(wrappedCmd, minAngle, maxAngle);

        double desiredRotorRots = turretToRotorRotations(desiredTurret);
        if (allowMovement)
            talon.setControl(motionMagicRequest.withPosition(desiredRotorRots));
    }

    @Override
    public void stop() {
        talon.setControl(voltageRequest.withOutput(0.0));
    }

    @Override
    public void runVolts(double volts) {
        if (allowMovement)
            talon.setControl(voltageRequest.withOutput(volts * turretSign));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        talonConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talon.getConfigurator().apply(talonConfig.MotorOutput);
    }

    @Override
    public void setCurrentLimit(double amps) {
        talonConfig.CurrentLimits.StatorCurrentLimit = amps;
        talonConfig.CurrentLimits.SupplyCurrentLimit = amps;
        talon.getConfigurator().apply(talonConfig.CurrentLimits);
    }

    @Override
    public void setPID(
            double p,
            double i,
            double d,
            double ks,
            double kv,
            double ka,
            double velocityMax,
            double accelerationMax,
            double rampRate) {
        talonConfig.Slot0.kP = p;
        talonConfig.Slot0.kI = i;
        talonConfig.Slot0.kD = d;
        talonConfig.Slot0.kS = ks;
        talonConfig.Slot0.kV = kv;
        talonConfig.Slot0.kA = ka;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = velocityMax * motorToTurretRatio / TWO_PI;
        talonConfig.MotionMagic.MotionMagicAcceleration = accelerationMax * motorToTurretRatio / TWO_PI;

        talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;
        talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate;

        talon.getConfigurator().apply(talonConfig.Slot0);
        talon.getConfigurator().apply(talonConfig.MotionMagic);
        talon.getConfigurator().apply(talonConfig.ClosedLoopRamps);
        talon.getConfigurator().apply(talonConfig.OpenLoopRamps);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.add(new SelfCheckingTalonFX(name + "_talon", talon));
        hardware.add(new SelfCheckingCANCoder(name + "_CANcoderBig", canCoderBig));
        return hardware;
    }

    @Override
    public void disable() {
        allowMovement = false;
    }

    @Override
    public void enable() {
        allowMovement = true;
    }

    @Override
    public void requestRezero() {
        final boolean magSwitchRecentlySeen =
                (Timer.getFPGATimestamp() - lastMagSwitchContactSec) <= MAG_SWITCH_RECENT_CONTACT_SEC;
        if (haveLock && !magSwitchRecentlySeen) {
            lastRezeroRequestResult = "IGNORED_MAG_SWITCH_DISCONNECTED";
            return;
        }
        zeroingRequested = true;
        haveLock = false;
        lastMagSwitchContactSec = Double.NEGATIVE_INFINITY;
        lastRezeroRequestResult = "QUEUED";
        stop();
    }

    @Override
    public boolean wantsZeroing() {
        return zeroingRequested || !haveLock;
    }

    private double primaryEncoderAngleToTurretRads(double rawEncoderRotations) {
        double encoderDeltaRotations = MathUtil.inputModulus(rawEncoderRotations, -0.5, 0.5);
        return Units.rotationsToRadians(encoderDeltaRotations / primaryEncoderRatio);
    }

    private double turretToRotorRotations(double turretRads) {
        return (turretRads / TWO_PI) * motorToTurretRatio * turretSign;
    }

    private double rotorRotationsToTurretRads(double rotorRotations) {
        return Units.rotationsToRadians(rotorRotations / (motorToTurretRatio * turretSign));
    }

    private double normalizeAbsoluteRadians(double rawRadians) {
        return MathUtil.inputModulus(rawRadians * turretSign, 0.0, TWO_PI);
    }
}
