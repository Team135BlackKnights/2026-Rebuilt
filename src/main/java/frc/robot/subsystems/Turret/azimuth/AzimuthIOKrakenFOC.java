package frc.robot.subsystems.Turret.azimuth;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRT;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRT.CRTStatus;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRTConfig;
import frc.robot.utils.YAMS.GearBox;
import frc.robot.utils.YAMS.MechanismGearing;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.SmartMotorControllerConfig;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;
import frc.robot.utils.YAMS.TalonFXWrapper;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class AzimuthIOKrakenFOC implements AzimuthIO {

    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double ENCODER_UPDATE_HZ = 200.0;
    private static final double MOTOR_UPDATE_HZ = 200.0;
    private static final double ENCODER_ERROR = 1/7.7;

    private static final double BASE_TOLERANCE_ROT = Units.degreesToRadians(3.0) / TWO_PI;
    /** Extra tolerance per rad/s of turret velocity to absorb CAN timestamp skew. */
    private static final double TOLERANCE_PER_RADPS_ROT = Units.degreesToRadians(1) / TWO_PI;

    private final TalonFX talon;
    private final CANcoder canCoderBig;
    private final CANcoder canCoderSmall;

    private final SmartMotorControllerConfig motorConfig;
    private final SmartMotorController motorController;

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
    private final double enc1GearTeeth;
    private final double enc2GearTeeth;

    /** +1 for left turret, -1 for right turret. All turret-space math uses this sign convention. */
    private final double turretSign;
    private final boolean rightTurret;

    private double lastTurretAngleRads = 0.0;
    /** FPGA timestamp of the last updateInputs call, for velocity integration. */
    private double lastUpdateTimeSec = -1.0;

    private final EasyCRT easyCrt;
    private final EasyCRTConfig crtConfig;

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
        this.enc1GearTeeth = enc1GearTeeth;
        this.enc2GearTeeth = enc2GearTeeth;

        this.rightTurret =
                canCoderBigID == AdvancedMechanismConstants.Turret.rightAzimuthBigEncoderID;
        this.turretSign = rightTurret ? -1.0 : 1.0;

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

        CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
        CANcoderConfiguration encoder2Config = new CANcoderConfiguration();

        encoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoder2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        encoder1Config.MagnetSensor.MagnetOffset = encoder1Offset;
        encoder2Config.MagnetSensor.MagnetOffset = encoder2Offset;

        /*
         * Leave both physical sensors configured the same way and do side mirroring here in software.
         */
        encoder1Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoder2Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        canCoderBig.getConfigurator().apply(encoder1Config);
        canCoderSmall.getConfigurator().apply(encoder2Config);

        motorConfig = new SmartMotorControllerConfig()
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(
                        0.0, 0.0, 0.0, RadiansPerSecond.of(12.0), RadiansPerSecondPerSecond.of(40.0))
                .withSimClosedLoopController(
                        0.0, 0.0, 0.0, RadiansPerSecond.of(12.0), RadiansPerSecondPerSecond.of(40.0))
                .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0))
                .withSimFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0))
                .withGearing(new MechanismGearing(
                        GearBox.fromReductionStages(AdvancedMechanismConstants.Turret.motorRadPerTurretRad)))
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(currentLimitAmps))
                .withSupplyCurrentLimit(Amps.of(currentLimitAmps))
                .withClosedLoopRampRate(Seconds.of(0.0))
                .withClosedLoopControlPeriod(Seconds.of(1.0 / MOTOR_UPDATE_HZ))
                .withOpenLoopRampRate(Seconds.of(0.0));

        motorController = new TalonFXWrapper(talon, DCMotor.getKrakenX44Foc(1), motorConfig);

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

        BaseStatusSignal.setUpdateFrequencyForAll(ENCODER_UPDATE_HZ, bigAbsRots, smallAbsRots, bigAbsRotsVel, smallAbsRotsVel);
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
                (Supplier<Pair<Angle, Angle>>)() -> {
                    return Pair.of(Rotations.of(normalizeAbsoluteRotations(bigAbsRots.getValueAsDouble())),
                            Rotations.of(normalizeAbsoluteRotations(smallAbsRots.getValueAsDouble())));
                })
                .withEncoderRatios(encoder1Ratio, encoder2Ratio)
                .withMechanismRange(
                        Rotations.of(minTurretAngle / TWO_PI - 0.75),
                        Rotations.of(maxTurretAngle / TWO_PI + 0.75))
                .withMatchTolerance(Rotations.of(BASE_TOLERANCE_ROT));

        easyCrt = new EasyCRT(crtConfig);
        this.crtConfig = crtConfig;
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

        /*
         * Keep raw values in the normal telemetry fields so you can still inspect exactly what the hardware reports.
         * Use normalized turret-space values for all solving logic.
         */
        inputs.bigEncoderRads = bigRawRads;
        inputs.smallEncoderRads = smallRawRads;

        final double bigSolveRads = normalizeAbsoluteRadians(bigRawRads);
        final double smallSolveRads = normalizeAbsoluteRadians(smallRawRads);

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        final double mechanismPosRadTurretSpace =
                toTurretSpaceMechanism(motorController.getMechanismPosition().in(Radians));
        final double mechanismVelRadPerSecTurretSpace =
                toTurretSpaceMechanism(motorController.getMechanismVelocity().in(RadiansPerSecond));

        /*
         * Velocity-integrate lastTurretAngleRads every cycle so the position
         * estimate stays alive even when CRT drops out at high speed.
         */
        final double now = Timer.getFPGATimestamp();
        final double dt = (lastUpdateTimeSec > 0) ? (now - lastUpdateTimeSec) : 0.0;
        lastUpdateTimeSec = now;

        // Predict where we are NOW based on motor velocity since last cycle
        double predictedTurretRad = lastTurretAngleRads
                + mechanismVelRadPerSecTurretSpace * dt;

        /*
         * Dynamically widen the CRT tolerance based on turret velocity.
         * At high speed the two CANcoders sample at slightly different CAN timestamps,
         * so the angular mismatch grows proportionally to velocity.
         * This prevents NO_SOLUTION during fast motion.
         */
        double dynamicTolRot = BASE_TOLERANCE_ROT
                + TOLERANCE_PER_RADPS_ROT * Math.abs(mechanismVelRadPerSecTurretSpace);
        crtConfig.withMatchTolerance(Rotations.of(dynamicTolRot));

        // --- Inline CRT solve (single-threaded, no race conditions) ---
        Optional<Angle> crtResult = easyCrt.getAngleOptional();
        CRTStatus crtStatus = easyCrt.getLastStatus();

        double solvedRad = Double.NaN;
        if (crtResult.isPresent()) {
            solvedRad = crtResult.get().in(Radians);
            if (solvedRad - lastTurretAngleRads >= ENCODER_ERROR){
                solvedRad += ENCODER_ERROR;
            }else if (solvedRad - lastTurretAngleRads <= -ENCODER_ERROR){
                solvedRad -= ENCODER_ERROR;
            }
        }

        Logger.recordOutput(name + "/Turret/IsRightTurret", rightTurret);
        Logger.recordOutput(name + "/Turret/TurretSign", turretSign);
        Logger.recordOutput(name + "/Turret/DynamicToleranceRot", dynamicTolRot);
        Logger.recordOutput(name + "/Turret/CrtLastErrorRot", easyCrt.getLastErrorRotations());

        Logger.recordOutput(name + "/Turret/SolveStatusRaw", crtStatus);
        Logger.recordOutput(
                name + "/Turret/SolveStatus",
                 crtStatus.name());
        Logger.recordOutput(name + "/Turret/SolveAngle", solvedRad);

        Logger.recordOutput(name + "/Turret/EncoderBigRadsRaw", bigRawRads);
        Logger.recordOutput(name + "/Turret/EncoderSmallRadsRaw", smallRawRads);
        Logger.recordOutput(name + "/Turret/EncoderBigRadsNormalized", bigSolveRads);
        Logger.recordOutput(name + "/Turret/EncoderSmallRadsNormalized", smallSolveRads);

        Logger.recordOutput(name + "/Turret/MechanismPosTurretSpace", mechanismPosRadTurretSpace);
        Logger.recordOutput(name + "/Turret/MechanismVelTurretSpace", mechanismVelRadPerSecTurretSpace);
        Logger.recordOutput(name + "/Turret/PredictedTurretRad", predictedTurretRad);
        Logger.recordOutput(name + "/Turret/Dt", dt);

        if (Double.isFinite(solvedRad)) {
            // CRT solved — use the absolute answer and mark lock
            lastTurretAngleRads = solvedRad;
            haveLock = true;
        } else if (haveLock) {
            // CRT dropout — coast on velocity-integrated prediction
            lastTurretAngleRads = predictedTurretRad;
        } /*else {
            // No lock yet — use raw mechanism position as best guess
            /lastTurretAngleRads = mechanismPosRadTurretSpace;
        }*/

        inputs.turretPositionRads = lastTurretAngleRads;
        inputs.turretVelocityRadsPerSec = mechanismVelRadPerSecTurretSpace;
    }

    @Override
    public void setDesiredPosition(double turretRads) {
        double lower = maxAngle - TWO_PI;
        double upper = maxAngle;

        double wrappedCmd = MathUtil.inputModulus(turretRads, lower, upper);
        double desiredTurret = MathUtil.clamp(wrappedCmd, minAngle, maxAngle);

        double mechanismPosRadTurretSpace =
                toTurretSpaceMechanism(motorController.getMechanismPosition().in(Radians));
        double mechanismTargetRadTurretSpace =
                mechanismPosRadTurretSpace + (desiredTurret - lastTurretAngleRads);

        if (!motorController.isClosedLoopRunning()) {
            motorController.startClosedLoopController();
            System.out.println("starting closed loop for azimuth!");
        }

        motorController.setPosition(
                Radians.of(fromTurretSpaceMechanism(mechanismTargetRadTurretSpace)));
    }

    @Override
    public void stop() {
        motorController.setVoltage(Volts.zero());
    }

    @Override
    public void runVolts(double volts) {
        /*
         * Positive open-loop volts should correspond to positive turret-space motion on BOTH turrets.
         */
        motorController.setVoltage(Volts.of(volts * turretSign));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motorController.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
    }

    @Override
    public void setCurrentLimit(double amps) {
        motorController.setSupplyCurrentLimit(Amps.of(amps));
        motorController.setStatorCurrentLimit(Amps.of(amps));
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
        motorController.setFeedback(p, i, d);
        motorController.setFeedforward(ks, kv, ka, 0.0);
        motorController.setMotionProfileMaxVelocity(RadiansPerSecond.of(velocityMax));
        motorController.setMotionProfileMaxAcceleration(RadiansPerSecondPerSecond.of(accelerationMax));
        motorController.setClosedLoopRampRate(Seconds.of(rampRate));
        motorController.setOpenLoopRampRate(Seconds.of(rampRate));
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

    private double toTurretSpaceMechanism(double rawMechanismRadians) {
        return rawMechanismRadians * turretSign;
    }

    private double fromTurretSpaceMechanism(double turretSpaceMechanismRadians) {
        return turretSpaceMechanismRadians * turretSign;
    }
}