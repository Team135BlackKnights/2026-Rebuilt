package frc.robot.subsystems.Turret.azimuth;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRT;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRTConfig;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class AzimuthIOKrakenFOC implements AzimuthIO {

    private final TalonFX talon;
    private final CANcoder canCoderBig;
    private final CANcoder canCoderSmall;

    private final StatusSignal<Angle> motorRots;
    private final StatusSignal<AngularVelocity> motorVelocityRotsPerSec;

    // IMPORTANT: absolute signals
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
    private TalonFXConfiguration config;

    private final MotionMagicTorqueCurrentFOC positionControl = new MotionMagicTorqueCurrentFOC(0.0);
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

    private double lastTurretAngleRads = 0.0;

    private final EasyCRT easyCrt;

    private final double encoder1Ratio;
    private final double encoder2Ratio;

    private final double minAngle;
    private final double maxAngle;

    public AzimuthIOKrakenFOC(
            CANBus bus,
            int ID,
            int canCoderBigID,
            int canCoderSmallID,
            String name,
            int currentLimitAmps,
            double minTurretAngle,
            double maxTurretAngle) {

        this.name = name;
        this.minAngle = minTurretAngle;
        this.maxAngle = maxTurretAngle;

        double turretToIdlerRatio = (double) AdvancedMechanismConstants.Turret.turretTeeth
                / (double) AdvancedMechanismConstants.Turret.idlerTeeth;
        this.encoder1Ratio = -turretToIdlerRatio; // big encoder spins opposite turret
        this.encoder2Ratio = turretToIdlerRatio
                * (AdvancedMechanismConstants.Turret.enc1GearTeeth / AdvancedMechanismConstants.Turret.enc2GearTeeth);

        this.talon = new TalonFX(ID, bus);
        this.canCoderBig = new CANcoder(canCoderBigID, bus);
        this.canCoderSmall = new CANcoder(canCoderSmallID, bus);

        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        talon.getConfigurator().apply(config);

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

        motorRots = talon.getPosition();
        motorVelocityRotsPerSec = talon.getVelocity();

        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        // absolute
        bigAbsRots = canCoderBig.getAbsolutePosition();
        smallAbsRots = canCoderSmall.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50, bigAbsRots, smallAbsRots);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50, motorRots, motorVelocityRotsPerSec, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);

        talon.optimizeBusUtilization(0, 1.0);

        // Allow CRT to resolve across wraps; motion commands still clamp to
        // minAngle/maxAngle.
        EasyCRTConfig crtConfig = new EasyCRTConfig(
                () -> bigAbsRots.getValue(),
                () -> smallAbsRots.getValue())
                .withEncoderRatios(encoder1Ratio, encoder2Ratio)
                .withMechanismRange(Rotations.of(minTurretAngle / (2.0 * Math.PI)-.75),
                        Rotations.of(maxTurretAngle / (2.0 * Math.PI)-.75))
                .withMatchTolerance(Rotations.of(Units.degreesToRadians(5.0) / (2.0 * Math.PI)));

        this.easyCrt = new EasyCRT(crtConfig);
    }

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                motorRots, motorVelocityRotsPerSec, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK();

        inputs.bothEncodersConnected = BaseStatusSignal.refreshAll(bigAbsRots, smallAbsRots).isOK();

        inputs.name = name;

        inputs.motorPositionRads = Units.rotationsToRadians(motorRots.getValueAsDouble());
        inputs.motorVelocityRadsPerSec = Units.rotationsToRadians(motorVelocityRotsPerSec.getValueAsDouble());

        double bigRads = MathUtil.inputModulus(Units.rotationsToRadians(bigAbsRots.getValueAsDouble()), 0, 2 * Math.PI);
        double smallRads = MathUtil.inputModulus(Units.rotationsToRadians(smallAbsRots.getValueAsDouble()), 0,
                2 * Math.PI);

        inputs.bigEncoderRads = bigRads;
        inputs.smallEncoderRads = smallRads;

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        double motorPosRad = inputs.motorPositionRads;
        // Continuous reference from motor DELTA since last lock (works even if motor
        // "zero" is arbitrary)
        double turretRef = haveLock
                ? turretRadAtLock
                        + (motorPosRad - motorPosRadAtLock) / AdvancedMechanismConstants.Turret.motorRadPerTurretRad
                : 0.0;

        Optional<Angle> solvedAngle = easyCrt.getAngleOptional();
        Logger.recordOutput(name + "/Turret/SolveStatus", easyCrt.getLastStatus());
        Logger.recordOutput(name + "/Turret/SolveAngle", solvedAngle.isPresent() ? solvedAngle.get().in(Radians) : Double.NaN);
        if (solvedAngle.isPresent()) {
            double solvedRad = solvedAngle.get().in(Radians);
            lastTurretAngleRads = solvedRad;
            // lock the motor reference to this solved angle
            haveLock = true;
            turretRadAtLock = solvedRad;
            motorPosRadAtLock = motorPosRad;
        } else if (haveLock) {
            lastTurretAngleRads = turretRef;
        } else {
            lastTurretAngleRads = 0.0;
        }

        inputs.turretPositionRads = lastTurretAngleRads;

        inputs.turretVelocityRadsPerSec = Units.rotationsToRadians(motorVelocityRotsPerSec.getValueAsDouble())
                / AdvancedMechanismConstants.Turret.motorRadPerTurretRad;
    }

@Override
public void setDesiredPosition(double turretRads) {
    final double TWO_PI = 2.0 * Math.PI;
    double lower = maxAngle - TWO_PI;
    double upper = maxAngle;
    double wrappedCmd = MathUtil.inputModulus(turretRads, lower, upper);

    double desiredTurret = MathUtil.clamp(wrappedCmd, minAngle, maxAngle);

    double motorPosRad = Units.rotationsToRadians(motorRots.getValueAsDouble());
    double motorTargetRad = motorPosRad
        + (desiredTurret - lastTurretAngleRads) * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;

    positionControl.Position = Units.radiansToRotations(motorTargetRad);
    talon.setControl(positionControl);
}
    @Override
    public void stop() {
        talon.setControl(voltageOut.withOutput(0));
    }

    @Override
    public void runVolts(double volts) {
        talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talon.getConfigurator().apply(config);
    }

    @Override
    public void setCurrentLimit(double amps) {
        config.CurrentLimits.SupplyCurrentLimit = amps;
        talon.getConfigurator().apply(config);
    }

    @Override
    public void setPID(
            double p, double i, double d, double ks, double kv, double ka, double velocityMax, double accelerationMax) {
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        config.Slot0.kS = ks;
        config.Slot0.kV = kv;
        config.Slot0.kA = ka;
        config.MotionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(velocityMax);
        config.MotionMagic.MotionMagicAcceleration = Units.radiansToRotations(accelerationMax);
        talon.getConfigurator().apply(config);
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