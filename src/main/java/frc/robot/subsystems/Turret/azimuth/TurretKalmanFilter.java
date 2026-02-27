package frc.robot.subsystems.Turret.azimuth;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

/**
 *
 * <h2>Plant model</h2>
 * State {@code x = [θ, ω]} (turret angle rad, turret angular velocity rad/s).<br>
 * Input {@code u = [V]} (motor voltage, volts).<br>
 * Output {@code y = [θ]} (observed turret angle, radians).
 *
 * <p>The plant is created with {@link LinearSystemId#createSingleJointedArmSystem} using the
 * actual motor, MOI, and gearing
 *
 * <h2>Measurement updates</h2>
 * Three sources, each with different noise covariances passed to
 * {@link KalmanFilter#correct(Matrix, Matrix, Matrix)}:
 * <ol>
 *   <li>Encoder "big" — delta-integrated position (moderate noise)</li>
 *   <li>Encoder "small" — delta-integrated position (moderate noise)</li>
 *   <li>EasyCRT absolute — direct absolute angle (low noise, only at low speed)</li>
 * </ol>
 * Motor deltas are also integrated and used as a fourth source with higher noise.
 *
 * <p>Created for FRC Team 135.
 */
public class TurretKalmanFilter {

    private static final double TWO_PI = 2.0 * Math.PI;

    /*
     * Physics-derived velocity process noise:
     *   α_turret = (Kt × I_limit × G) / J
     *            = (0.0194 × 30 × 26.95) / 0.0424 ≈ 370 rad/s²
     *   At dt = 0.005 s → Δω ≈ 1.85 rad/s per cycle.
     *
     */
    private static final double Q_ANGLE_STDDEV  = 0.002;  // rad position process noise
    private static final double Q_VEL_STDDEV    = 1.85;   // rad/s MOI-based

    private static final double R_ENCODER_STDDEV = 0.008;
    private static final double R_CRT_STDDEV     = 0.003;
    private static final double R_MOTOR_STDDEV   = 0.015;

    private static final double DT_SECONDS = 1.0 / 50.0;

    private final LinearSystem<N2, N1, N1> plant;
    private final KalmanFilter<N2, N1, N1> kf;

    /** Pre-built R matrices for each measurement source (1×1). */
    private final Matrix<N1, N1> rEncoder;
    private final Matrix<N1, N1> rCrt;
    private final Matrix<N1, N1> rMotor;

    private final double enc1RadsPerTurretRad;
    private final double enc2RadsPerTurretRad;

    private double prevEnc1Rad = Double.NaN;
    private double prevEnc2Rad = Double.NaN;
    private double prevMotorTurretRad = Double.NaN;

    private boolean seeded = false;

    private final String logPrefix;

    /**
     * Creates a new TurretKalmanFilter backed by WPILib's KalmanFilter.
     *
     * @param turretToIdlerRatio turretTeeth / idlerTeeth (e.g. 7.7)
     * @param enc1GearTeeth      tooth count on the big encoder gear
     * @param enc2GearTeeth      tooth count on the small encoder gear
     * @param logPrefix          name prefix for AdvantageKit logging
     */
    public TurretKalmanFilter(
            double turretToIdlerRatio,
            double enc1GearTeeth,
            double enc2GearTeeth,
            String logPrefix) {

        this.logPrefix = logPrefix;

        // Encoder ratios (encoder-rad per turret-rad, magnitude only)
        this.enc1RadsPerTurretRad = turretToIdlerRatio;
        this.enc2RadsPerTurretRad = turretToIdlerRatio * (enc1GearTeeth / enc2GearTeeth);

        // LinearSystemId gives us N2 outputs (angle + velocity), but our KalmanFilter
        // only observes angle (N1 output). Extract A, B and build a new plant with
        // C = [1, 0] (observe angle only), D = [0].
        DCMotor motor = AdvancedMechanismConstants.Turret.azimuthMotor;
        double moiKgM2 = AdvancedMechanismConstants.Turret.azimuthMOI;
        double gearing = AdvancedMechanismConstants.Turret.motorRadPerTurretRad;

        var fullPlant = LinearSystemId.createSingleJointedArmSystem(motor, moiKgM2, gearing);

        plant = new LinearSystem<>(
                fullPlant.getA(),                                     
                fullPlant.getB(),                                    
                MatBuilder.fill(Nat.N1(), Nat.N2(), 1.0, 0.0),       // C = [1, 0]  (observe angle)
                new Matrix<>(Nat.N1(), Nat.N1()));                    // D = [0]

        kf = new KalmanFilter<>(
                Nat.N2(),                                      
                Nat.N1(),                                       
                plant,
                VecBuilder.fill(Q_ANGLE_STDDEV, Q_VEL_STDDEV), 
                VecBuilder.fill(R_ENCODER_STDDEV),
                DT_SECONDS);

        // Pre-build measurement covariance matrices for the three noise levels.
        // These are CONTINUOUS covariance matrices (σ², not σ); WPILib discretizes internally.
        rEncoder = Matrix.eye(Nat.N1()).times(R_ENCODER_STDDEV * R_ENCODER_STDDEV);
        rCrt     = Matrix.eye(Nat.N1()).times(R_CRT_STDDEV * R_CRT_STDDEV);
        rMotor   = Matrix.eye(Nat.N1()).times(R_MOTOR_STDDEV * R_MOTOR_STDDEV);
    }


    /**
     * Seed the filter with an absolute turret angle from EasyCRT.
     *
     * @param turretAngleRads absolute turret angle in radians
     */
    public void seed(double turretAngleRads) {
        kf.setXhat(0, turretAngleRads);
        kf.setXhat(1, 0.0);

        // Tight initial covariance
        var tightP = Matrix.eye(Nat.N2());
        tightP.set(0, 0, 0.005 * 0.005);  // angle variance
        tightP.set(1, 1, 1.0);            // velocity variance
        tightP.set(0, 1, 0.0);
        tightP.set(1, 0, 0.0);
        kf.setP(tightP);

        seeded = true;
    }

    /** @return true if the filter has been seeded with an absolute position */
    public boolean isSeeded() {
        return seeded;
    }

    /** @return the current turret angle estimate (radians) */
    public double getAngle() {
        return kf.getXhat(0);
    }

    /** @return the current turret velocity estimate (rad/s) */
    public double getVelocity() {
        return kf.getXhat(1);
    }

    /**
     * Predict step — propagate the state forward using the plant model.
     *
     * <p>We don't command voltage through this filter, so we pass {@code u = 0 V} and
     * instead forcibly set the velocity state to the motor-reported velocity (blended with
     * the filter's own estimate). This lets the physics-based covariance propagation work
     * correctly while still trusting the motor encoder for velocity.
     *
     * @param motorVelocityTurretSpaceRadPerSec motor velocity in turret-space rad/s
     * @param dt                                 time step in seconds
     */
    public void predict(double motorVelocityTurretSpaceRadPerSec, double dt) {
        if (!seeded || dt <= 0) return;

        // Blend motor velocity with filter velocity (trust motor highly)
        double alpha = 0.9;
        double blendedVelocity = alpha * motorVelocityTurretSpaceRadPerSec
                + (1.0 - alpha) * kf.getXhat(1);

        // Override velocity state before prediction so the plant propagates from it
        kf.setXhat(1, blendedVelocity);

        // Predict with u = 0V — we already injected velocity; let covariance grow naturally
        kf.predict(VecBuilder.fill(0.0), dt);
    }

    /**
     * Correct the filter with an angle observation at a given noise level.
     *
     * @param observedAngleRad observed turret angle (radians)
     * @param R                1×1 continuous measurement covariance matrix
     */
    private void correctAngle(double observedAngleRad, Matrix<N1, N1> R) {
        if (!seeded) return;
        kf.correct(VecBuilder.fill(0.0), VecBuilder.fill(observedAngleRad), R);
    }

    /**
     * Process encoder + motor readings and produce measurement updates.
     *
     * @param enc1NormalizedRad encoder 1 reading [0, 2π), turret-space normalized
     * @param enc2NormalizedRad encoder 2 reading [0, 2π), turret-space normalized
     * @param motorTurretRad   motor mechanism position in turret-space radians (continuous)
     */
    public void updateFromEncoders(double enc1NormalizedRad, double enc2NormalizedRad, double motorTurretRad) {
        if (!seeded) return;

        if (Double.isFinite(prevEnc1Rad) && Double.isFinite(enc1NormalizedRad)) {
            double encDelta = closestAngle(prevEnc1Rad, enc1NormalizedRad, TWO_PI);
            double turretDelta = encDelta / enc1RadsPerTurretRad;
            double observed = getAngle() + turretDelta;
            correctAngle(observed, rEncoder);
        }
        prevEnc1Rad = enc1NormalizedRad;

        if (Double.isFinite(prevEnc2Rad) && Double.isFinite(enc2NormalizedRad)) {
            double encDelta = closestAngle(prevEnc2Rad, enc2NormalizedRad, TWO_PI);
            double turretDelta = encDelta / enc2RadsPerTurretRad;
            double observed = getAngle() + turretDelta;
            correctAngle(observed, rEncoder);
        }
        prevEnc2Rad = enc2NormalizedRad;

        if (Double.isFinite(prevMotorTurretRad) && Double.isFinite(motorTurretRad)) {
            double motorDelta = motorTurretRad - prevMotorTurretRad;
            double observed = getAngle() + motorDelta;
            correctAngle(observed, rMotor);
        }
        prevMotorTurretRad = motorTurretRad;
    }

    /**
     * Provide an absolute CRT measurement to correct drift.
     * Only call this at low speed when CRT is reliable.
     *
     * @param crtAngleRad absolute turret angle from EasyCRT (radians)
     */
    public void updateFromCrt(double crtAngleRad) {
        if (!seeded) return;
        correctAngle(crtAngleRad, rCrt);
    }


    public void log() {
        Logger.recordOutput(logPrefix + "/KF/AngleRad", getAngle());
        Logger.recordOutput(logPrefix + "/KF/VelocityRadPerSec", getVelocity());
        Logger.recordOutput(logPrefix + "/KF/CovP00", kf.getP(0, 0));
        Logger.recordOutput(logPrefix + "/KF/CovP11", kf.getP(1, 1));
        Logger.recordOutput(logPrefix + "/KF/Seeded", seeded);
        Logger.recordOutput(logPrefix + "/KF/PrevEnc1", prevEnc1Rad);
        Logger.recordOutput(logPrefix + "/KF/PrevEnc2", prevEnc2Rad);
    }
    private static double closestAngle(double from, double to, double period) {
        double diff = (to - from) % period;
        if (diff > period / 2.0) diff -= period;
        if (diff <= -period / 2.0) diff += period;
        return diff;
    }
}
