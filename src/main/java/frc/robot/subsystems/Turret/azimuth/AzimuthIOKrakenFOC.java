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

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRT;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRTConfig;
import frc.robot.utils.YAMS.GearBox;
import frc.robot.utils.YAMS.MechanismGearing;
import frc.robot.utils.YAMS.SmartMotorController;
import frc.robot.utils.YAMS.SmartMotorControllerConfig;
import frc.robot.utils.YAMS.TalonFXWrapper;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.ControlMode;
import frc.robot.utils.YAMS.SmartMotorControllerConfig.MotorMode;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class AzimuthIOKrakenFOC implements AzimuthIO {

    private final TalonFX talon;
    private final CANcoder canCoderBig;
    private final CANcoder canCoderSmall;

    private final SmartMotorControllerConfig motorConfig;
    private final SmartMotorController motorController;

    private final StatusSignal<Angle> motorRotorRots;
    private final StatusSignal<AngularVelocity> motorRotorVelocityRotsPerSec;

    private final StatusSignal<Angle> bigAbsRots;
    private final StatusSignal<Angle> smallAbsRots;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private boolean haveLock = false;
    private double motorPosRadAtLock = 0.0;
    private double turretRadAtLock = 0.0;

    private final String name;
    private final double minAngle;
    private final double maxAngle;

    private double lastTurretAngleRads = 0.0;

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
            double encoder2Offset) {

        this.name = name;
        this.minAngle = minTurretAngle;
        this.maxAngle = maxTurretAngle;

        double turretToIdlerRatio = (double) AdvancedMechanismConstants.Turret.turretTeeth
                / (double) AdvancedMechanismConstants.Turret.idlerTeeth;
        double encoder1Ratio = -turretToIdlerRatio;
        double encoder2Ratio = turretToIdlerRatio
                * (AdvancedMechanismConstants.Turret.enc1GearTeeth / AdvancedMechanismConstants.Turret.enc2GearTeeth);

        talon = new TalonFX(ID, bus);
        canCoderBig = new CANcoder(canCoderBigID, bus);
        canCoderSmall = new CANcoder(canCoderSmallID, bus);

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
                .withIdleMode(MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(currentLimitAmps))
                .withSupplyCurrentLimit(Amps.of(currentLimitAmps))
                .withClosedLoopRampRate(Seconds.of(0.0))
                .withOpenLoopRampRate(Seconds.of(0.0));
        motorController = new TalonFXWrapper(talon, DCMotor.getKrakenX44Foc(1), motorConfig);

        motorRotorRots = talon.getRotorPosition();
        motorRotorVelocityRotsPerSec = talon.getRotorVelocity();

        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        bigAbsRots = canCoderBig.getAbsolutePosition();
        smallAbsRots = canCoderSmall.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50, bigAbsRots, smallAbsRots);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                motorRotorRots,
                motorRotorVelocityRotsPerSec,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius);

        talon.optimizeBusUtilization(0, 1.0);

        EasyCRTConfig crtConfig = new EasyCRTConfig(
                () -> bigAbsRots.getValue(),
                () -> smallAbsRots.getValue())
                .withEncoderRatios(encoder1Ratio, encoder2Ratio)
                .withMechanismRange(
                        Rotations.of(minTurretAngle / (2.0 * Math.PI) - .75),
                        Rotations.of(maxTurretAngle / (2.0 * Math.PI) + .75))
                .withMatchTolerance(Rotations.of(Units.degreesToRadians(5.0) / (2.0 * Math.PI)));

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

        inputs.bothEncodersConnected = BaseStatusSignal.refreshAll(bigAbsRots, smallAbsRots).isOK();
        inputs.name = name;

        inputs.motorPositionRads = Units.rotationsToRadians(motorRotorRots.getValueAsDouble());
        inputs.motorVelocityRadsPerSec = Units.rotationsToRadians(motorRotorVelocityRotsPerSec.getValueAsDouble());

        double bigRads = MathUtil.inputModulus(Units.rotationsToRadians(bigAbsRots.getValueAsDouble()), 0, 2 * Math.PI);
        double smallRads = MathUtil.inputModulus(Units.rotationsToRadians(smallAbsRots.getValueAsDouble()), 0, 2 * Math.PI);
        inputs.bigEncoderRads = bigRads;
        inputs.smallEncoderRads = smallRads;

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        double mechanismPosRad = motorController.getMechanismPosition().in(Radians);
        double turretRef = haveLock
                ? turretRadAtLock + (mechanismPosRad - motorPosRadAtLock)
                : mechanismPosRad;

        Optional<Angle> solvedAngle = easyCrt.getAngleOptional();
        double fallbackSolvedRad = TurretMathematics.TurretMath.turretAngleFromEncodersRad(
                bigRads, smallRads, turretRef, Units.degreesToRadians(8.0));
        boolean usedFallbackSolve = false;
        double solvedRad = Double.NaN;
        if (solvedAngle.isPresent()) {
            solvedRad = solvedAngle.get().in(Radians);
        } else if (Double.isFinite(fallbackSolvedRad)) {
            solvedRad = fallbackSolvedRad;
            usedFallbackSolve = true;
        }

        Logger.recordOutput(name + "/Turret/SolveStatusRaw", easyCrt.getLastStatus());
        Logger.recordOutput(
                name + "/Turret/SolveStatus",
                Double.isFinite(solvedRad) ? (usedFallbackSolve ? "FALLBACK_OK" : "OK") : easyCrt.getLastStatus().name());
        Logger.recordOutput(name + "/Turret/SolveUsedFallback", usedFallbackSolve);
        Logger.recordOutput(name + "/Turret/SolveAngle", solvedRad);
        Logger.recordOutput(name + "/Turret/EncoderBigRads", bigRads);
        Logger.recordOutput(name + "/Turret/EncoderSmallRads", smallRads);

        if (Double.isFinite(solvedRad)) {
            lastTurretAngleRads = solvedRad;
            haveLock = true;
            turretRadAtLock = solvedRad;
            motorPosRadAtLock = mechanismPosRad;
        } else if (haveLock) {
            lastTurretAngleRads = turretRef;
        } else {
            lastTurretAngleRads = mechanismPosRad;
        }

        inputs.turretPositionRads = lastTurretAngleRads;
        inputs.turretVelocityRadsPerSec = motorController.getMechanismVelocity().in(RadiansPerSecond);
    }

    @Override
    public void setDesiredPosition(double turretRads) {
        final double TWO_PI = 2.0 * Math.PI;
        double lower = maxAngle - TWO_PI;
        double upper = maxAngle;
        double wrappedCmd = MathUtil.inputModulus(turretRads, lower, upper);
        double desiredTurret = MathUtil.clamp(wrappedCmd, minAngle, maxAngle);

        double mechanismPosRad = motorController.getMechanismPosition().in(Radians);
        double mechanismTargetRad = mechanismPosRad + (desiredTurret - lastTurretAngleRads);
        motorController.setPosition(Radians.of(mechanismTargetRad));
    }

    @Override
    public void stop() {
        motorController.setVoltage(Volts.zero());
    }

    @Override
    public void runVolts(double volts) {
        motorController.setVoltage(Volts.of(volts));
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
            double p, double i, double d, double ks, double kv, double ka, double velocityMax, double accelerationMax, double rampRate) {
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
}
