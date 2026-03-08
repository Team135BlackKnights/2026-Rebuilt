package frc.robot.subsystems.Turret.hood;


import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFXS;

/**
 * Hood IO for TalonFXS driving a brushed Johnson PLG motor.
 *
 * Sensor is the PLG's 2 hall outputs wired to the TalonFXS Gadgeteer port:
 *   - Quad A (pin 7) and Quad B (pin 5)
 * plus pullups (WPILib recommends ~1k to 5V on each hall output).
 */
public class HoodIOKrakenFOC implements HoodIO {

    // At screw extension 0.000 in, hood 12 deg (hardstop)
    // At screw extension 1.767 in, hood 50 deg ("hard"stop)
    private static final double START_ANGLE_DEG = 13.0;
    private static final double END_ANGLE_DEG = 50.0;
    private static final double END_EXTENSION_IN = 1.767;
    private static final double DEG_PER_INCH = (END_ANGLE_DEG - START_ANGLE_DEG) / END_EXTENSION_IN;

    // Leadscrew: 4-start, 2mm pitch, lead = 8mm / rev
    private static final double LEAD_MM_PER_REV = 8.0;
    private static final double LEAD_IN_PER_REV = LEAD_MM_PER_REV / 25.4;
    private static final double REV_PER_INCH = 1.0 / LEAD_IN_PER_REV;
    public static final double BOOT_ANGLE_RAD = Math.toRadians(START_ANGLE_DEG);

    // CTRE config expects an int. which is annoying.. Use 178 (closest to 177.6 aka 44.4*4).
    private static final int QUAD_EDGES_PER_OUTPUT_REV = 178; //yes, we're going to be off by 0.1 ticks at max. womp womp


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

    private static final LoggableTunedNumber ZERO_VOLTS = new LoggableTunedNumber("Hood/zeroVolts", -8, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber ZERO_CURRENT_AMPS = new LoggableTunedNumber("Hood/zeroAmps", 2.5, TuningConstants.isTuningIntake);
    private static final LoggableTunedNumber ZERO_HOLD_SEC = new LoggableTunedNumber("Hood/zeroTime", 0.06, TuningConstants.isTuningIntake);

    protected boolean zeroingActive = false;
    protected boolean openLoop = false;
    private double zeroSpikeStartTimeSec = Double.NaN;

    public HoodIOKrakenFOC(
            CANBus bus,
            int motorID,
            String name,
            int currentLimitAmps,
            double minAngleRads,
            double maxAngleRads
    ) {
        this.name = name;
        this.minAngleRads = minAngleRads;
        this.maxAngleRads = maxAngleRads;

        talon = new TalonFXS(motorID, bus);

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;

        cfg.ExternalTemp.TempSensorRequired = TempSensorRequiredValue.Not_Required;

        cfg.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Quadrature;//Gadgeeter
        cfg.ExternalFeedback.QuadratureEdgesPerRotation = QUAD_EDGES_PER_OUTPUT_REV;

        cfg.ExternalFeedback.SensorPhase = SensorPhaseValue.Aligned;//MAKE SURE THIS IS RIGHT

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodRadToSensorRot(maxAngleRads);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kP = 0.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kS = 0.0;
        cfg.Slot0.kV = 0.0;
        cfg.Slot0.kA = 0.0;
        cfg.Slot0.kG = 0.0;

        cfg.MotionMagic.MotionMagicCruiseVelocity = 0.0; 
        cfg.MotionMagic.MotionMagicAcceleration = 0.0;   

        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
        cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;

        talon.getConfigurator().apply(cfg);

        pos = talon.getPosition();
        vel = talon.getVelocity();
        appliedVoltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        torqueCurrent = talon.getTorqueCurrent();
        tempCelsius = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, pos, vel, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
        talon.optimizeBusUtilization(0, 1.0);
        // start zeroing on creation. position will be set to BOOT_ANGLE_RAD when the reverse stop/current spike is detected
        zero();
    }


    private static double clampHoodDeg(double deg, double minRad, double maxRad) {
        double minDeg = Math.toDegrees(minRad);
        double maxDeg = Math.toDegrees(maxRad);
        return MathUtil.clamp(deg, minDeg, maxDeg);
    }

    /** extension (in) from hood deg, using your linear fit. */
    private static double hoodDegToExtensionIn(double hoodDeg) {
        return (hoodDeg - START_ANGLE_DEG) / DEG_PER_INCH;
    }

    /** screw revolutions from extension in. */
    private static double extensionInToScrewRev(double extensionIn) {
        return extensionIn * REV_PER_INCH;
    }

    private double hoodRadToSensorRot(double hoodRad) {
        double hoodDeg = Math.toDegrees(hoodRad);
        hoodDeg = clampHoodDeg(hoodDeg, minAngleRads, maxAngleRads);

        double extIn = hoodDegToExtensionIn(hoodDeg);
        double screwRev = extensionInToScrewRev(extIn);
        return screwRev;
    }

    private double sensorRotToHoodRad(double sensorRot) {
        double screwRev = sensorRot;
        double extIn = screwRev / REV_PER_INCH;
        double hoodDeg = START_ANGLE_DEG + (extIn * DEG_PER_INCH);
        return Math.toRadians(hoodDeg);
    }


    @Override
    public void updateInputs(HoodIOInputs inputs) {
        processZeroing();
        inputs.name = name;
        inputs.connected = BaseStatusSignal.refreshAll(appliedVoltage, pos, vel, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK();
        // Position/velocity from quadrature sensor (sensor ROTS)
        double sensorRot = pos.getValueAsDouble();
        double sensorRps = vel.getValueAsDouble();
        double volts = ff.calculate(sensorRps) + controller.calculate(sensorRot);
        runVolts(volts);
        inputs.positionRads = sensorRotToHoodRad(sensorRot);
        inputs.velocityRadsPerSec = Units.rotationsToRadians(sensorRps);

        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        Logger.recordOutput("Hood/" + name + "/SensorRot", sensorRot);
        Logger.recordOutput("Hood/" + name + "/HoodDegEst", Math.toDegrees(inputs.positionRads));
        Logger.recordOutput("Hood/"+name+"ZERO", zeroingActive);
    }

    @Override
    public void setPosition(double positionRads) {
        if (zeroingActive) {
            return;
        }
        double clamped = MathUtil.clamp(positionRads, minAngleRads, maxAngleRads);
        double targetSensorRot = hoodRadToSensorRot(clamped);
        controller.setSetpoint(targetSensorRot);
    }

    @Override
    public void runVolts(double volts) {
        if (zeroingActive) {
            return;
        }
        openLoop = true;
        //talon.setControl(voltageOut.withOutput(volts));
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
        openLoop = false;
    }

    private void processZeroing() {
        if (!zeroingActive) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        // push gently into the reverse/hardstop direction
        //talon.setControl(voltageOut.withOutput(ZERO_VOLTS.get()));
        BaseStatusSignal.refreshAll(supplyCurrent, torqueCurrent);
        double observedCurrentAmps = Math.max(
                Math.abs(supplyCurrent.getValueAsDouble()),
                Math.abs(torqueCurrent.getValueAsDouble()));

        if (observedCurrentAmps >= ZERO_CURRENT_AMPS.get()) {
            if (Double.isNaN(zeroSpikeStartTimeSec)) {
                zeroSpikeStartTimeSec = now;
            }
        } else {
            zeroSpikeStartTimeSec = Double.NaN;
        }

        if (!Double.isNaN(zeroSpikeStartTimeSec) && (now - zeroSpikeStartTimeSec) >= ZERO_HOLD_SEC.get()) {
            // set encoder to BOOT angle (12 deg) when we've hit the reverse hardstop
            talon.setPosition(hoodRadToSensorRot(BOOT_ANGLE_RAD));
            zeroingActive = false;
            zeroSpikeStartTimeSec = Double.NaN;
            openLoop = false;
        }
    }

    @Override
    public void setCurrentLimit(double amps) {
        cfg.CurrentLimits.SupplyCurrentLimit = amps;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        talon.getConfigurator().apply(cfg.CurrentLimits);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        cfg.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talon.getConfigurator().apply(cfg.MotorOutput);
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
        double sensorRotPerHoodRad =
                (180.0 / Math.PI) * (1.0 / DEG_PER_INCH) * REV_PER_INCH;

        cfg.MotionMagic.MotionMagicCruiseVelocity = cruiseRadPerSec * sensorRotPerHoodRad;   // rot/s
        cfg.MotionMagic.MotionMagicAcceleration = accelRadPerSec2 * sensorRotPerHoodRad;    // rot/s/s
        talon.getConfigurator().apply(cfg.MotionMagic); //the rot consumes
    }
}