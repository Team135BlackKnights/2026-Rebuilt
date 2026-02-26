package frc.robot.subsystems.Turret.azimuth;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
import com.ctre.phoenix6.signals.MagnetHealthValue;
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
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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

public class AzimuthIOSim implements AzimuthIO {

    private static final double TURRET_RATIO = (double) AdvancedMechanismConstants.Turret.turretTeeth
            / (double) AdvancedMechanismConstants.Turret.idlerTeeth;

    private final double enc1ToEnc2Ratio;
    private final double combinedRatio;

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
    private final double minAngleRad;
    private final double maxAngleRad;
    private final double enc1GearTeeth;
    private final double enc2GearTeeth;

    private double lastTurretAngleRads = 0.0;

    private final EasyCRT easyCrt;

    public AzimuthIOSim(
            CANBus bus,
            int motorID,
            int canCoderBigID,
            int canCoderSmallID,
            String name,
            int currentLimitAmps,
            double minTurretAngleRad,
            double maxTurretAngleRad,
            double enc1GearTeeth,
            double enc2GearTeeth) {
        this.name = name;
        this.minAngleRad = minTurretAngleRad;
        this.maxAngleRad = maxTurretAngleRad;
        this.enc1GearTeeth = enc1GearTeeth;
        this.enc2GearTeeth = enc2GearTeeth;
        this.enc1ToEnc2Ratio = enc1GearTeeth / enc2GearTeeth;
        this.combinedRatio = TURRET_RATIO * enc1ToEnc2Ratio;

        double encoder1Ratio = -TURRET_RATIO;
        double encoder2Ratio = TURRET_RATIO
                * (enc1GearTeeth / enc2GearTeeth);

        talon = new TalonFX(motorID, bus);
        canCoderBig = new CANcoder(canCoderBigID, bus);
        canCoderSmall = new CANcoder(canCoderSmallID, bus);

        CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
        CANcoderConfiguration encoder2Config = new CANcoderConfiguration();

        encoder1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoder2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        encoder1Config.MagnetSensor.MagnetOffset = -.404541;
        encoder2Config.MagnetSensor.MagnetOffset = 0.088867;

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
                .withOpenLoopRampRate(Seconds.of(0.0))
                .withMomentOfInertia(KilogramSquareMeters.of(AdvancedMechanismConstants.Turret.azimuthMOI));
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
                        Rotations.of(minTurretAngleRad / (2.0 * Math.PI) - .75),
                        Rotations.of(maxTurretAngleRad / (2.0 * Math.PI) +.75))
                .withMatchTolerance(Rotations.of(Units.degreesToRadians(5.0) / (2.0 * Math.PI)));

        easyCrt = new EasyCRT(crtConfig);
    }

    public AzimuthIOSim(double minTurretAngleRad, double maxTurretAngleRad) {
        this(
                new CANBus("rio"),
                0,
                1,
                2,
                "AzimuthIOSim",
                AdvancedMechanismConstants.Turret.currentLimitAzimuth,
                minTurretAngleRad,
                maxTurretAngleRad,
                AdvancedMechanismConstants.Turret.enc1GearTeethRight,
                AdvancedMechanismConstants.Turret.enc2GearTeethRight);
    }

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
        motorController.simIterate();

        double turretPosRad = motorController.getMechanismPosition().in(Radians);
        double turretVelRadPerSec = motorController.getMechanismVelocity().in(RadiansPerSecond);

        double bigPositionRot = (TURRET_RATIO * turretPosRad) / (2.0 * Math.PI);
        double smallPositionRot = (-combinedRatio * turretPosRad) / (2.0 * Math.PI);
        double bigVelocityRotPerSec = (TURRET_RATIO * turretVelRadPerSec) / (2.0 * Math.PI);
        double smallVelocityRotPerSec = (-combinedRatio * turretVelRadPerSec) / (2.0 * Math.PI);

        var bigSim = canCoderBig.getSimState();
        bigSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
        bigSim.setRawPosition(Rotations.of(bigPositionRot));
        bigSim.setVelocity(RotationsPerSecond.of(bigVelocityRotPerSec));
        bigSim.setMagnetHealth(MagnetHealthValue.Magnet_Green);

        var smallSim = canCoderSmall.getSimState();
        smallSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
        smallSim.setRawPosition(Rotations.of(smallPositionRot));
        smallSim.setVelocity(RotationsPerSecond.of(smallVelocityRotPerSec));
        smallSim.setMagnetHealth(MagnetHealthValue.Magnet_Green);

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

        Optional<Angle> solved = easyCrt.getAngleOptional();
        double fallbackSolvedRad = TurretMathematics.TurretMath.turretAngleFromEncodersRad(
                bigRads, smallRads, turretRef, Units.degreesToRadians(8.0), enc1GearTeeth, enc2GearTeeth);
        boolean usedFallbackSolve = false;
        double solvedRad = Double.NaN;
        if (solved.isPresent()) {
            solvedRad = solved.get().in(Radians);
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
        inputs.turretVelocityRadsPerSec = turretVelRadPerSec;
    }

    @Override
    public void setDesiredPosition(double turretRads) {
        final double TWO_PI = 2.0 * Math.PI;
        double lower = maxAngleRad - TWO_PI;
        double upper = maxAngleRad;
        double wrappedCmd = MathUtil.inputModulus(turretRads, lower, upper);
        double desiredTurret = MathUtil.clamp(wrappedCmd, minAngleRad, maxAngleRad);

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
    public void setCurrentLimit(double amps) {
        motorController.setSupplyCurrentLimit(Amps.of(amps));
        motorController.setStatorCurrentLimit(Amps.of(amps));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motorController.setIdleMode(brake ? MotorMode.BRAKE : MotorMode.COAST);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<>();
    }
}
