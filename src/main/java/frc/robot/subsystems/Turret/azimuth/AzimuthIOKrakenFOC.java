package frc.robot.subsystems.Turret.azimuth;

import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRT;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRT.CRTStatus;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRTConfig;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class AzimuthIOKrakenFOC implements AzimuthIO {

    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double ENCODER_UPDATE_HZ = 100.0;
    private static final double MOTOR_UPDATE_HZ = 100.0;
    private static final double ENCODER_ERROR = 1 / 7.7;

    /** Motor rotor rotations per turret rotation (36/10 * 77/10 = 27.72). */
    private static final double MOTOR_TO_TURRET_RATIO =
            AdvancedMechanismConstants.Turret.motorRadPerTurretRad;

    private static final double BASE_TOLERANCE_ROT = Units.degreesToRadians(3.0) / TWO_PI;
    private static final LoggableTunedNumber SPEED_CUT =
            new LoggableTunedNumber("Turrets/SPEED_CUT", 0.05, TuningConstants.isTuningShooter);

    private final TalonFX talon;
    private final CANcoder canCoderBig;
    private final CANcoder canCoderSmall;

    /* Direct Phoenix 6 control requests — no YAMS wrapper */
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final MotionMagicVoltage motionMagicRequest =
            new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final StatusSignal<Angle> motorRotorRots;
    private final StatusSignal<AngularVelocity> motorRotorVelocityRotsPerSec;

    private final StatusSignal<Angle> bigAbsRots;
    private final StatusSignal<AngularVelocity> bigAbsRotsVel;
    private final StatusSignal<Angle> smallAbsRots;
    private final StatusSignal<AngularVelocity> smallAbsRotsVel;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private boolean haveLock = false;

    private final String name;
    private final double minAngle;
    private final double maxAngle;

    private final double turretSign;
    private final boolean rightTurret;

    private double lastTurretAngleRads = 0.0;
    private double lastUpdateTimeSec = -1.0;

    /**
     * The motor rotor position (in rotations) that corresponds to turret angle = 0.
     * Computed on ANY CRT lock:  offset = currentRotorRots - turretRads/(2π) * ratio * sign
     */
    private double motorRotorOffsetRots = Double.NaN;

    private final EasyCRT easyCrt;

    public AzimuthIOKrakenFOC(
            CANBus bus,
            int ID,
            int canCoderBigID,
            int canCoderSmallID,
            String name,
            int currentLimitAmps,
            double minTurretAngle,
            double maxTurretAngle,
            double encoder1Offset,
            double encoder2Offset,
            double enc1GearTeeth,
            double enc2GearTeeth) {

        this.name = name;
        this.minAngle = minTurretAngle;
        this.maxAngle = maxTurretAngle;

        this.rightTurret =
                canCoderSmallID == AdvancedMechanismConstants.Turret.rightAzimuthBigEncoderID;
        this.turretSign = rightTurret ? 1.0 : 1.0;

        final double turretToIdlerRatio =
                (double) AdvancedMechanismConstants.Turret.turretTeeth
                        / (double) AdvancedMechanismConstants.Turret.idlerTeeth;

        /*
         * We normalize the encoder readings into turret-space before feeding EasyCRT,
         * so both turrets can use the SAME effective ratios here.
         */
        final double encoder1Ratio = -turretToIdlerRatio;
        final double encoder2Ratio = turretToIdlerRatio * (enc1GearTeeth / enc2GearTeeth);

        talon = new TalonFX(ID, bus);
        canCoderBig = new CANcoder(canCoderBigID, bus);
        canCoderSmall = new CANcoder(canCoderSmallID, bus);

        /* ---- CANcoder configs ---- */
        CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
        CANcoderConfiguration encoder2Config = new CANcoderConfiguration();

        encoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoder2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        encoder1Config.MagnetSensor.MagnetOffset = encoder1Offset;
        encoder2Config.MagnetSensor.MagnetOffset = encoder2Offset;

        encoder1Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder2Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        canCoderBig.getConfigurator().apply(encoder1Config);
        canCoderSmall.getConfigurator().apply(encoder2Config);

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

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = 12.0 * MOTOR_TO_TURRET_RATIO / TWO_PI;
        talonConfig.MotionMagic.MotionMagicAcceleration = 40.0 * MOTOR_TO_TURRET_RATIO / TWO_PI;
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
        smallAbsRots = canCoderSmall.getAbsolutePosition();
        smallAbsRotsVel = canCoderSmall.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                ENCODER_UPDATE_HZ, bigAbsRots, smallAbsRots, bigAbsRotsVel, smallAbsRotsVel);
        BaseStatusSignal.setUpdateFrequencyForAll(
                MOTOR_UPDATE_HZ,
                motorRotorRots,
                motorRotorVelocityRotsPerSec,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius);

        talon.optimizeBusUtilization(0, 1.0);

        EasyCRTConfig crtConfig = new EasyCRTConfig(
                (Supplier<Pair<Angle, Angle>>) () -> {
                    return Pair.of(
                            Rotations.of(normalizeAbsoluteRotations(bigAbsRots.getValueAsDouble())),
                            Rotations.of(normalizeAbsoluteRotations(smallAbsRots.getValueAsDouble())));
                })
                .withEncoderRatios(encoder1Ratio, encoder2Ratio)
                .withMechanismRange(
                        Rotations.of(minTurretAngle / TWO_PI - 0.75),
                        Rotations.of(maxTurretAngle / TWO_PI + 0.75))
                .withMatchTolerance(Rotations.of(BASE_TOLERANCE_ROT));

        easyCrt = new EasyCRT(crtConfig);
    }

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                motorRotorRots,
                motorRotorVelocityRotsPerSec,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius).isOK();

        inputs.bothEncodersConnected = BaseStatusSignal.refreshAll(
                bigAbsRots, smallAbsRots, bigAbsRotsVel, smallAbsRotsVel).isOK();
        inputs.name = name;

        inputs.motorPositionRads = Units.rotationsToRadians(motorRotorRots.getValueAsDouble());
        inputs.motorVelocityRadsPerSec =
                Units.rotationsToRadians(motorRotorVelocityRotsPerSec.getValueAsDouble());

        double bigRawRads = MathUtil.inputModulus(
                Units.rotationsToRadians(bigAbsRots.getValueAsDouble()), 0.0, TWO_PI);
        double smallRawRads = MathUtil.inputModulus(
                Units.rotationsToRadians(smallAbsRots.getValueAsDouble()), 0.0, TWO_PI);

        inputs.bigEncoderRads = bigRawRads;
        inputs.smallEncoderRads = smallRawRads;

        final double bigSolveRads = normalizeAbsoluteRadians(bigRawRads);
        final double smallSolveRads = normalizeAbsoluteRadians(smallRawRads);

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        final double motorVelRotsPerSec = motorRotorVelocityRotsPerSec.getValueAsDouble();
        final double turretVelRadsPerSec =
                Units.rotationsToRadians(motorVelRotsPerSec) * turretSign / MOTOR_TO_TURRET_RATIO;

        /*
         * Velocity-integrate lastTurretAngleRads every cycle so the position
         * estimate stays alive even when CRT drops out at high speed.
         */
        final double now = Timer.getFPGATimestamp();
        final double dt = (lastUpdateTimeSec > 0) ? (now - lastUpdateTimeSec) : 0.0;
        lastUpdateTimeSec = now;

        double predictedTurretRad = lastTurretAngleRads + turretVelRadsPerSec * dt;

        double solvedRad = Double.NaN;
        boolean solved = false;
        if (Math.abs(turretVelRadsPerSec) < SPEED_CUT.get()) {

            Optional<Angle> crtResult = easyCrt.getAngleOptional();
            CRTStatus crtStatus = easyCrt.getLastStatus();

            if (crtResult.isPresent()) {
                solvedRad = crtResult.get().in(edu.wpi.first.units.Units.Radians);
                if (solvedRad - lastTurretAngleRads >= ENCODER_ERROR) {
                    solvedRad += ENCODER_ERROR;
                } else if (solvedRad - lastTurretAngleRads <= -ENCODER_ERROR) {
                    solvedRad -= ENCODER_ERROR;
                }
                solved = true;
            }
            Logger.recordOutput(name + "/Turret/SolveStatusRaw", crtStatus);
            Logger.recordOutput(name + "/Turret/SolveStatus", crtStatus.name());
            Logger.recordOutput(name + "/Turret/SolveAngle", solvedRad);
        } else {
            Logger.recordOutput(name + "/Turret/SolveStatus", "IGNORED");
        }

        Logger.recordOutput(name + "/Turret/IsRightTurret", rightTurret);
        Logger.recordOutput(name + "/Turret/TurretSign", turretSign);
        Logger.recordOutput(name + "/Turret/CrtLastErrorRot", easyCrt.getLastErrorRotations());

        Logger.recordOutput(name + "/Turret/EncoderBigRadsRaw", bigRawRads);
        Logger.recordOutput(name + "/Turret/EncoderSmallRadsRaw", smallRawRads);
        Logger.recordOutput(name + "/Turret/EncoderBigRadsNormalized", bigSolveRads);
        Logger.recordOutput(name + "/Turret/EncoderSmallRadsNormalized", smallSolveRads);

        Logger.recordOutput(name + "/Turret/TurretVelRadsPerSec", turretVelRadsPerSec);
        Logger.recordOutput(name + "/Turret/PredictedTurretRad", predictedTurretRad);
        Logger.recordOutput(name + "/Turret/Dt", dt);

        if (solved) {
            lastTurretAngleRads = solvedRad;
            haveLock = true;

            /*
             * (Re-)compute the rotor offset every time CRT solves.
             * offset = currentRotorRots - turretRads/(2π) * ratio * sign
             * This keeps the mapping from turret-space to rotor-space accurate
             * even if there is any mechanical slip throughout a rotation. Maybe disable this/ max 1 per sec (Sync Cancoder essentially)
             */
            double currentRotorRots = motorRotorRots.getValueAsDouble();
            motorRotorOffsetRots = currentRotorRots
                    - (solvedRad / TWO_PI) * MOTOR_TO_TURRET_RATIO * turretSign;
            Logger.recordOutput(name + "/Turret/MotorRotorOffsetRots", motorRotorOffsetRots);
        } else if (haveLock) {
            lastTurretAngleRads = predictedTurretRad;
        }

        inputs.turretPositionRads = lastTurretAngleRads;
        inputs.turretVelocityRadsPerSec = turretVelRadsPerSec;
    }

    /**
     * The desired turret angle (radians) is converted to an absolute motor rotor
     * position (rotations) using the offset established at lock.
     */
    @Override
    public void setDesiredPosition(double turretRads) {
        if (!haveLock || Double.isNaN(motorRotorOffsetRots)) {
            Logger.recordOutput(name + "/Turret/DesStatus", "NO_LOCK");
            return;
        }

        double lower = maxAngle - TWO_PI;
        double upper = maxAngle;

        double wrappedCmd = MathUtil.inputModulus(turretRads, lower, upper);
        double desiredTurret = MathUtil.clamp(wrappedCmd, minAngle, maxAngle);

        // Convert desired turret-space angle to absolute motor rotor rotations
        double desiredRotorRots = motorRotorOffsetRots
                + (desiredTurret / TWO_PI) * MOTOR_TO_TURRET_RATIO * turretSign;

        Logger.recordOutput(name + "/Turret/DesMotorSpot", desiredRotorRots);
        Logger.recordOutput(name + "/Turret/DesiredTurretRads", desiredTurret);
        Logger.recordOutput(name + "/Turret/DesStatus", "OK");

        talon.setControl(motionMagicRequest.withPosition(desiredRotorRots));
    }

    @Override
    public void stop() {
        talon.setControl(voltageRequest.withOutput(0.0));
    }

    @Override
    public void runVolts(double volts) {
        talon.setControl(voltageRequest.withOutput(volts * turretSign));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        talonConfig.MotorOutput.NeutralMode =
                brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
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

        talonConfig.MotionMagic.MotionMagicCruiseVelocity =
                velocityMax * MOTOR_TO_TURRET_RATIO / TWO_PI;
        talonConfig.MotionMagic.MotionMagicAcceleration =
                accelerationMax * MOTOR_TO_TURRET_RATIO / TWO_PI;

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
        hardware.add(new SelfCheckingCANCoder(name + "_CANcoderSmall", canCoderSmall));
        return hardware;
    }

    private double normalizeAbsoluteRotations(double rawRotations) {
        return MathUtil.inputModulus(rawRotations * turretSign, 0.0, 1.0);
    }

    private double normalizeAbsoluteRadians(double rawRadians) {
        return MathUtil.inputModulus(rawRadians * turretSign, 0.0, TWO_PI);
    }
}