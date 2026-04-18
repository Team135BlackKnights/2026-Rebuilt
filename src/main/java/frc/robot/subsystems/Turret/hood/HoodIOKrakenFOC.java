package frc.robot.subsystems.Turret.hood;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import com.ctre.phoenix6.signals.TempSensorRequiredValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFXS;

public class HoodIOKrakenFOC implements HoodIO {

    private static final double START_ANGLE_DEG_LEFT = 12.0;
    private static final double START_ANGLE_DEG_RIGHT = 13.0;

    private static final LoggableTunedNumber END_ANGLE_DEG_LEFT =
        new LoggableTunedNumber("Hood/Left/EndAngleDeg", 45.0, TuningConstants.isTuningIntake);

    private static final LoggableTunedNumber END_EXTENSION_IN_LEFT =
        new LoggableTunedNumber("Hood/Left/EndExtensionIn", 2.0, TuningConstants.isTuningIntake);

    private static final LoggableTunedNumber END_ANGLE_DEG_RIGHT =
        new LoggableTunedNumber("Hood/Right/EndAngleDeg", 46.0, TuningConstants.isTuningIntake);

    private static final LoggableTunedNumber END_EXTENSION_IN_RIGHT =
        new LoggableTunedNumber("Hood/Right/EndExtensionIn", 2.0, TuningConstants.isTuningIntake);

    // Leadscrew
    private static final double LEAD_MM_PER_REV = 8.0;
    private static final double LEAD_IN_PER_REV = LEAD_MM_PER_REV / 25.4;
    private static final double REV_PER_INCH = 1.0 / LEAD_IN_PER_REV;

    public static final double BOOT_ANGLE_RAD_LEFT = Math.toRadians(START_ANGLE_DEG_LEFT);
    public static final double BOOT_ANGLE_RAD_RIGHT = Math.toRadians(START_ANGLE_DEG_RIGHT);

    private static final int QUAD_EDGES_PER_OUTPUT_REV = 178;

    private final String name;
    private final TalonFXS talon;
    private final TalonFXSConfiguration cfg = new TalonFXSConfiguration();
    private PIDController controller = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
    private final VoltageOut voltageOut = new VoltageOut(0);

    private final double minAngleRads;
    private final double maxAngleRads;

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Angle> pos;
    private final StatusSignal<AngularVelocity> vel;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private static final LoggableTunedNumber ZERO_VOLTS =
        new LoggableTunedNumber("Hood/zeroVolts", -4, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber ZERO_CURRENT_AMPS =
        new LoggableTunedNumber("Hood/zeroAmps", 2.5, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber ZERO_HOLD_SEC =
        new LoggableTunedNumber("Hood/zeroTime", 0.06, TuningConstants.isTuningIntake);

    protected boolean zeroingActive = false;
    protected boolean openLoop = false;
    private double zeroSpikeStartTimeSec = Double.NaN;
    private final boolean isLeft;

    public HoodIOKrakenFOC(
            CANBus bus,
            int motorID,
            boolean isLeft,
            String name,
            int currentLimitAmps,
            double minAngleRads,
            double maxAngleRads
    ) {
        this.name = name;
        this.isLeft = name.equals(AdvancedMechanismConstants.Turret.leftName);
        this.minAngleRads = minAngleRads;
        this.maxAngleRads = maxAngleRads;

        talon = new TalonFXS(motorID, bus);

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;

        cfg.ExternalTemp.TempSensorRequired = TempSensorRequiredValue.Not_Required;

        cfg.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Quadrature;
        cfg.ExternalFeedback.QuadratureEdgesPerRotation = QUAD_EDGES_PER_OUTPUT_REV;
        cfg.ExternalFeedback.SensorPhase = SensorPhaseValue.Aligned;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodRadToSensorRot(maxAngleRads);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talon.getConfigurator().apply(cfg);

        pos = talon.getPosition();
        vel = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, pos, vel, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
        talon.optimizeBusUtilization(1, 1.0);

        zero();
    }

    private static double getDegPerInchLeft() {
        return (END_ANGLE_DEG_LEFT.get() - START_ANGLE_DEG_LEFT)
                / END_EXTENSION_IN_LEFT.get();
    }

    private static double getDegPerInchRight() {
        return (END_ANGLE_DEG_RIGHT.get() - START_ANGLE_DEG_RIGHT)
                / END_EXTENSION_IN_RIGHT.get();
    }

    private static double clampHoodDeg(double deg, double minRad, double maxRad) {
        return MathUtil.clamp(deg, Math.toDegrees(minRad), Math.toDegrees(maxRad));
    }

    private double hoodDegToExtensionIn(double hoodDeg) {
        return isLeft
            ? ((hoodDeg - START_ANGLE_DEG_LEFT) / getDegPerInchLeft())
            : ((hoodDeg - START_ANGLE_DEG_RIGHT) / getDegPerInchRight());
    }

    private static double extensionInToScrewRev(double extensionIn) {
        return extensionIn * REV_PER_INCH;
    }

    private double hoodRadToSensorRot(double hoodRad) {
        double hoodDeg = clampHoodDeg(Math.toDegrees(hoodRad), minAngleRads, maxAngleRads);
        return extensionInToScrewRev(hoodDegToExtensionIn(hoodDeg));
    }

    private double sensorRotToHoodRad(double sensorRot) {
        double extIn = sensorRot / REV_PER_INCH;
        double hoodDeg = isLeft
            ? (START_ANGLE_DEG_LEFT + (extIn * getDegPerInchLeft()))
            : (START_ANGLE_DEG_RIGHT + (extIn * getDegPerInchRight()));
        return Math.toRadians(hoodDeg);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        processZeroing();

        inputs.connected = BaseStatusSignal.refreshAll(
                appliedVoltage, pos, vel, supplyCurrent, torqueCurrent, tempCelsius).isOK();

        double sensorRot = pos.getValueAsDouble();
        double sensorRps = vel.getValueAsDouble();

        double volts = ff.calculate(sensorRps) + controller.calculate(sensorRot);

        if (!isLeft)runVolts(volts);

        inputs.positionRads = sensorRotToHoodRad(sensorRot);
        inputs.velocityRadsPerSec = Units.rotationsToRadians(sensorRps);

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

    }

    @Override
    public void setPosition(double positionRads) {
        if (zeroingActive) return;

        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
          if (!(isLeft)) controller.setSetpoint(hoodRadToSensorRot(clamped));
    }

    @Override
    public void runVolts(double volts) {
        if (zeroingActive) return;
        talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    public void zero() {
        zeroingActive = true;
        zeroSpikeStartTimeSec = Double.NaN;
    }

    @Override
    public void cancelZero() {
        zeroingActive = false;
        zeroSpikeStartTimeSec = Double.NaN;
    }

    private void processZeroing() {
        if (!zeroingActive) return;

        double now = Timer.getFPGATimestamp();
        talon.setControl(voltageOut.withOutput(ZERO_VOLTS.get()));

        BaseStatusSignal.refreshAll(supplyCurrent, torqueCurrent);
        double current = Math.max(
                Math.abs(supplyCurrent.getValueAsDouble()),
                Math.abs(torqueCurrent.getValueAsDouble()));

        if (current >= ZERO_CURRENT_AMPS.get()) {
            if (Double.isNaN(zeroSpikeStartTimeSec)) {
                zeroSpikeStartTimeSec = now;
            }
        } else {
            zeroSpikeStartTimeSec = Double.NaN;
        }

        if (!Double.isNaN(zeroSpikeStartTimeSec)
                && (now - zeroSpikeStartTimeSec) >= ZERO_HOLD_SEC.get()) {

            talon.setPosition(hoodRadToSensorRot(
                isLeft ? BOOT_ANGLE_RAD_LEFT : BOOT_ANGLE_RAD_RIGHT));

            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
        }
    }

    @Override
    public void setPID(double p, double d, double ks, double kv) {
        controller = new PIDController(p, 0, d);
        ff = new SimpleMotorFeedforward(ks, kv);
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<>();
        hardware.add(new SelfCheckingTalonFXS(name + "_Hood", talon));
        return hardware;
    }

    public void configureMotionMagic(double cruiseRadPerSec, double accelRadPerSec2) {
        double degPerInch = isLeft ? getDegPerInchLeft() : getDegPerInchRight();

        double sensorRotPerHoodRad =
            (180.0 / Math.PI) * (1.0 / degPerInch) * REV_PER_INCH;

        cfg.MotionMagic.MotionMagicCruiseVelocity = cruiseRadPerSec * sensorRotPerHoodRad;
        cfg.MotionMagic.MotionMagicAcceleration = accelRadPerSec2 * sensorRotPerHoodRad;

        talon.getConfigurator().apply(cfg.MotionMagic);
    }
}
