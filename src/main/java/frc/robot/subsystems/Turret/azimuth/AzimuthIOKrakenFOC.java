package frc.robot.subsystems.Turret.azimuth;

import java.util.ArrayList;
import java.util.List;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class AzimuthIOKrakenFOC implements AzimuthIO {

    private final TalonFX talon;
    private final CANcoder canCoderBig;
    private final CANcoder canCoderSmall;
    private final StatusSignal<Angle> motorRots;
    private final StatusSignal<AngularVelocity> motorVelocityRotsPerSec;
    private final StatusSignal<Angle> bigRots;
    private final StatusSignal<Angle> smallRots;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;
    private final String name;
    private TalonFXConfiguration config;
    private final MotionMagicTorqueCurrentFOC positionControl = new MotionMagicTorqueCurrentFOC(0.0);
    private final VoltageOut   voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private double lastTurretAngleRads = 0;
    private final double minAngle = -Math.PI;
    private final double maxAngle = Math.PI;
    public AzimuthIOKrakenFOC(CANBus bus, int ID, int canCoderBigID, int canCoderSmallID, String name,
            int currentLimitAmps,
            double minTurretAngle, double maxTurretAngle) {
        this.name = name;
        this.talon = new TalonFX(ID, bus);
        this.canCoderBig = new CANcoder(canCoderBigID, bus);
        this.canCoderSmall = new CANcoder(canCoderSmallID, bus);
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        talon.getConfigurator().apply(config);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        // zero is set via phoenix tuner
        canCoderBig.getConfigurator().apply(encoderConfig);
        canCoderSmall.getConfigurator().apply(encoderConfig);

        motorRots = talon.getPosition();
        motorVelocityRotsPerSec = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();
        // Encoders
        bigRots = canCoderBig.getPosition();
        smallRots = canCoderSmall.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50, bigRots, smallRots); // see if we can inc this later
        BaseStatusSignal.setUpdateFrequencyForAll(50, motorRots, motorVelocityRotsPerSec, appliedVoltage, supplyCurrent,
                torqueCurrent, tempCelsius);
        talon.optimizeBusUtilization(0, 1.0);

    }

    public void updateInputs(AzimuthIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                motorRots, motorVelocityRotsPerSec, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius).isOK();
        inputs.bothEncodersConnected = BaseStatusSignal.refreshAll(bigRots, smallRots).isOK();
        inputs.name = name;
        inputs.motorPositionRads = Units.rotationsToRadians(motorRots.getValueAsDouble());
        inputs.motorVelocityRadsPerSec = Units.rotationsToRadians(motorVelocityRotsPerSec.getValueAsDouble());
        double bigRads = Units.rotationsToRadians(bigRots.getValueAsDouble());
        double smallRads = Units.rotationsToRadians(smallRots.getValueAsDouble());
        inputs.bigEncoderRads = bigRads;
        inputs.smallEncoderRads = smallRads;
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        // Calculate turret position from encoders here using chinese remainder theorem
        lastTurretAngleRads = calculateTurretAngleRads(bigRads, smallRads);
        inputs.turretPositionRads = lastTurretAngleRads;
        // do velocity at some point if needd
        inputs.turretVelocityRadsPerSec = 0;
    }

    double calculateTurretAngleRads(double bigRads, double smallRads) {
        // this is in 0 - 2 pi range for both encoders.
        Rotation2d bigAngle = new Rotation2d(bigRads);
        Rotation2d smallAngle = new Rotation2d(smallRads);
        return TurretMathematics.TurretMath.turretAngleFromEncoders(bigAngle, smallAngle);
    }

  @Override
    public void setDesiredPosition(double turretRads) {

        // Clamp the desired turret command to mechanical limits
        double desiredTurret = MathUtil.clamp(turretRads, minAngle, maxAngle);

        double motorPosRad = Units.rotationsToRadians(motorRots.getValueAsDouble());

        // Compute motor position setpoint in continuous radians
        double motorTargetRad = TurretMathematics.TurretMath.motorSetpointForTurretAngle(
                lastTurretAngleRads,
                motorPosRad,
                desiredTurret,
                minAngle,
                maxAngle
        );

        positionControl.Position = Units.radiansToRotations(motorTargetRad);
        //talon.setControl(positionControl);
    }

    @Override
    public void stop() {
        talon.setControl(voltageOut.withOutput(0));
    }

    @Override
    public void runVolts(double volts) {
       // talon.setControl(voltageOut.withOutput(volts));
    }
    @Override
    public void setBrakeMode(boolean brake) {
        if (brake) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        talon.getConfigurator().apply(config);
    }
    @Override
    public void setCurrentLimit(double amps) {
        config.CurrentLimits.SupplyCurrentLimit = amps;
        talon.getConfigurator().apply(config);
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv, double ka, double velocityMax, double accelerationMax) {
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
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        hardware.add(new SelfCheckingTalonFX(name + "_talon", talon));
        hardware.add(new SelfCheckingCANCoder(name + "_CANcoderBig", canCoderBig));
        hardware.add(new SelfCheckingCANCoder(name + "_CANcoderSmall", canCoderSmall));
        return hardware;
    }
}