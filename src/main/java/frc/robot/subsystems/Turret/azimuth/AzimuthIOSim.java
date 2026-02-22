package frc.robot.subsystems.Turret.azimuth;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRT;
import frc.robot.subsystems.Turret.azimuth.EasyCRT.EasyCRTConfig;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;

public class AzimuthIOSim implements AzimuthIO {

    // Turret ratio between turret and the 10T idler (turret -> idler magnitude).
    private static final double TURRET_RATIO = (double) AdvancedMechanismConstants.Turret.turretTeeth
            / (double) AdvancedMechanismConstants.Turret.idlerTeeth;

    private static final double ENC1_TO_ENC2_RATIO = AdvancedMechanismConstants.Turret.enc1GearTeeth
            / AdvancedMechanismConstants.Turret.enc2GearTeeth;
    private static final double COMBINED_RATIO = TURRET_RATIO * ENC1_TO_ENC2_RATIO;

    private final DCMotorSim sim;

    private double turretAngleRad;
    private double turretVelRadPerSec = 0.0;
    private double motorPosRad = 0.0;
    private double motorVelRadPerSec = 0.0;

    private double appliedVolts = 0.0;
    private double currentDrawAmps = 0.0;

    private final String name = "AzimuthIOSim";
    private final double minAngleRad;
    private final double maxAngleRad;

    private static final double LOOP_DT_SEC = 0.02;

    private enum Mode {
        OPEN_VOLTAGE, POSITION
    }

    private Mode mode = Mode.OPEN_VOLTAGE;

    private double desiredTurretRad = 0.0;
    private double desiredMotorRad = 0.0;
    private double openLoopVolts = 0.0;

    private double kP = 4.0; // volts per motor-rad error
    private double kD = 0.2; // volts per motor-(rad/s)

    private final EasyCRT easyCrt;
    private double estimatedTurretRad = 0.0;

    public AzimuthIOSim(double minTurretAngleRad, double maxTurretAngleRad) {
        this.minAngleRad = minTurretAngleRad;
        this.maxAngleRad = maxTurretAngleRad;

        this.sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        AdvancedMechanismConstants.Turret.azimuthMOI,
                        AdvancedMechanismConstants.Turret.motorRadPerTurretRad),
                DCMotor.getKrakenX44Foc(1));

        double encoder1Ratio = -TURRET_RATIO;
        double encoder2Ratio = TURRET_RATIO
                * (AdvancedMechanismConstants.Turret.enc1GearTeeth / AdvancedMechanismConstants.Turret.enc2GearTeeth);

        EasyCRTConfig crtConfig = new EasyCRTConfig(
                () -> {
                    return Radians.of((-turretAngleRad * TURRET_RATIO));
                },
                () -> {
                    return Radians.of(COMBINED_RATIO * turretAngleRad);
                })
                .withEncoderRatios(encoder1Ratio, encoder2Ratio)
                .withMechanismRange(
                        Rotations.of(minAngleRad / (2.0 * Math.PI - .75)),
                        Rotations.of(maxAngleRad / (2.0 * Math.PI - .75)))
                .withMatchTolerance(Rotations.of(Math.toRadians(5.0) / (2.0 * Math.PI)));

        this.easyCrt = new EasyCRT(crtConfig);
    }

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
        if (mode == Mode.OPEN_VOLTAGE) {
            appliedVolts = MathUtil.clamp(openLoopVolts, -12.0, 12.0);
        } else {
            // If we have a turret target, compute a motor target from current "absolute
            // encoders" + motor position.

            double motorError = desiredMotorRad - motorPosRad;
            double motorVelError = 0.0 - motorVelRadPerSec;

            double volts = kP * motorError + kD * motorVelError;
            appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        }

        sim.setInputVoltage(appliedVolts);
        sim.update(LOOP_DT_SEC);
        double simVal = sim.getAngularPositionRad();
        turretAngleRad = simVal;
        turretVelRadPerSec = sim.getAngularVelocityRadPerSec();

        motorPosRad = turretAngleRad * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;
        motorVelRadPerSec = turretVelRadPerSec * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;

        // enc1 (big) consistent with baseSolution = mod(-enc1/turretRatio,
        // 2pi/turretRatio). Add small simulated noise to emulate sensor jitter.
        double noiseBig = (Math.random() - 0.5) * Math.toRadians(30); // ±15 deg
        double noiseSmall = (Math.random() - 0.5) * Math.toRadians(30); // ±15 deg

        double bigRad = -turretAngleRad * TURRET_RATIO + noiseBig;

        // enc2 consistent with predictedEnc2 = wrapToTwoPi(combinedRatio * turretAngle)
        double smallRad = COMBINED_RATIO * turretAngleRad + noiseSmall;
        currentDrawAmps = sim.getCurrentDrawAmps();

        inputs.motorConnected = true;
        inputs.bothEncodersConnected = true;
        inputs.name = name;

        inputs.motorPositionRads = motorPosRad;
        inputs.motorVelocityRadsPerSec = motorVelRadPerSec;

        inputs.bigEncoderRads = bigRad;
        inputs.smallEncoderRads = smallRad;

        inputs.appliedVoltage = appliedVolts;
        inputs.supplyCurrentAmps = currentDrawAmps; // close enough for sim
        inputs.torqueCurrentAmps = currentDrawAmps;
        inputs.tempCelsius = 25.0;
        Optional<edu.wpi.first.units.measure.Angle> solved = easyCrt.getAngleOptional();
        Logger.recordOutput(name + "/Turret/SolveStatus", easyCrt.getLastStatus());
        Logger.recordOutput(name + "/Turret/SolveAngle", solved.isPresent() ? solved.get().in(Radians) : Double.NaN);
        if (solved.isPresent()) {
            estimatedTurretRad = solved.get().in(Radians);
        } else {
            estimatedTurretRad = Double.NaN;
        }

        inputs.turretPositionRads = estimatedTurretRad;
        inputs.turretVelocityRadsPerSec = turretVelRadPerSec;
    }

    @Override
    public void setDesiredPosition(double turretRads) {
        final double TWO_PI = 2.0 * Math.PI;
        double lower = maxAngleRad - TWO_PI;
        double upper = maxAngleRad;

        double wrappedCmd = MathUtil.inputModulus(turretRads, lower, upper);
        desiredTurretRad = MathUtil.clamp(wrappedCmd, minAngleRad, maxAngleRad);
        desiredMotorRad = desiredTurretRad * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;
        mode = Mode.POSITION;
    }

    @Override
    public void stop() {
        mode = Mode.OPEN_VOLTAGE;
        openLoopVolts = 0.0;
    }

    @Override
    public void runVolts(double volts) {
        mode = Mode.OPEN_VOLTAGE;
        openLoopVolts = volts;
    }

    @Override
    public void setPID(double p, double i, double d, double ks, double kv, double ka, double velocityMax,
            double accelerationMax) {
        this.kP = p;
        this.kD = d;
    }

    @Override
    public void setCurrentLimit(double amps) {
        // No-op
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<>();
    }
}
