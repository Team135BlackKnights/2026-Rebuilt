package frc.robot.subsystems.Turret.azimuth;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;

public class AzimuthIOSim implements AzimuthIO {


    // Turret ratio between turret and the 10T idler (turret -> idler magnitude).
    private static final double TURRET_RATIO = (double) AdvancedMechanismConstants.Turret.turretTeeth / (double) AdvancedMechanismConstants.Turret.idlerTeeth;

    private static final double ENC1_TO_ENC2_RATIO = AdvancedMechanismConstants.Turret.enc1GearTeeth / AdvancedMechanismConstants.Turret.enc2GearTeeth;
    private static final double COMBINED_RATIO = TURRET_RATIO * ENC1_TO_ENC2_RATIO;

    private static final double TWO_PI = 2.0 * Math.PI;

    private final DCMotorSim sim;

    private double turretAngleRad = 0.0;    
    private double turretVelRadPerSec = 0.0;
    private double motorPosRad = 0.0;      
    private double motorVelRadPerSec = 0.0;

    private Rotation2d bigAbs = new Rotation2d();  
    private Rotation2d smallAbs = new Rotation2d();

    private double appliedVolts = 0.0;
    private double currentDrawAmps = 0.0;

    private final String name = "AzimuthIOSim";
    private final double minAngleRad;
    private final double maxAngleRad;

    private static final double LOOP_DT_SEC = 0.02;

    private enum Mode { OPEN_VOLTAGE, POSITION }
    private Mode mode = Mode.OPEN_VOLTAGE;

    private double desiredTurretRad = 0.0;   
    private double desiredMotorRad = 0.0;     
    private double openLoopVolts = 0.0;

    private double kP = 4.0; // volts per motor-rad error
    private double kD = 0.2; // volts per motor-(rad/s)

    public AzimuthIOSim(double minTurretAngleRad, double maxTurretAngleRad) {
        this.minAngleRad = minTurretAngleRad;
        this.maxAngleRad = maxTurretAngleRad;

        this.sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        AdvancedMechanismConstants.Turret.azimuthMOI,
                        AdvancedMechanismConstants.Turret.motorRadPerTurretRad
                ),
                DCMotor.getKrakenX44Foc(1)
        );
    }

    @Override
    public void updateInputs(AzimuthIOInputs inputs) {
        if (mode == Mode.OPEN_VOLTAGE) {
            appliedVolts = MathUtil.clamp(openLoopVolts, -12.0, 12.0);
        } else {
            // If we have a turret target, compute a motor target from current "absolute encoders" + motor position.
            desiredTurretRad = MathUtil.clamp(desiredTurretRad, minAngleRad, maxAngleRad);

            desiredMotorRad = TurretMathematics.TurretMath.motorSetpointForTurretAngle(
                    bigAbs,
                    smallAbs,
                    motorPosRad,
                    desiredTurretRad,
                    minAngleRad,
                    maxAngleRad
            );

            double motorError = desiredMotorRad - motorPosRad;
            double motorVelError = 0.0 - motorVelRadPerSec;

            double volts = kP * motorError + kD * motorVelError;
            appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        }

        sim.setInputVoltage(appliedVolts);
        sim.update(LOOP_DT_SEC);

        turretAngleRad = sim.getAngularPositionRad();
        turretVelRadPerSec = sim.getAngularVelocityRadPerSec();

        motorPosRad = turretAngleRad * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;
        motorVelRadPerSec = turretVelRadPerSec * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;

        // enc1 (big) consistent with baseSolution = mod(-enc1/turretRatio, 2pi/turretRatio)
        double bigRad = wrapToTwoPi(-turretAngleRad * TURRET_RATIO);

        // enc2 consistent with predictedEnc2 = wrapToTwoPi(combinedRatio * turretAngle)
        double smallRad = wrapToTwoPi(COMBINED_RATIO * turretAngleRad);

        bigAbs = new Rotation2d(bigRad);
        smallAbs = new Rotation2d(smallRad);

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
        inputs.turretPositionRads = TurretMathematics.TurretMath.turretAngleFromEncoders(bigAbs, smallAbs);
        inputs.turretVelocityRadsPerSec = turretVelRadPerSec;
    }

    @Override
    public void setDesiredPosition(double turretRads) {
        // turretRads is already unwrapped/good since there ain't no wires to break in sim
        desiredTurretRad = MathUtil.clamp(turretRads, minAngleRad, maxAngleRad);
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

    private static double wrapToTwoPi(double angleRad) {
        double wrapped = angleRad % TWO_PI;
        return wrapped < 0 ? wrapped + TWO_PI : wrapped;
    }
}
