package frc.robot.subsystems.Turret.azimuth;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

public class TurretMathematics {
    public final class TurretMath {

        private static final double enc1ToEnc2Ratio = AdvancedMechanismConstants.Turret.enc1GearTeeth / AdvancedMechanismConstants.Turret.enc2GearTeeth;
        private static final double gearGreatestCommonDivisor = greatestCommonDivisor(AdvancedMechanismConstants.Turret.enc1GearTeeth, AdvancedMechanismConstants.Turret.enc2GearTeeth);
        private static final int enc1CyclesForPeriod = (int) (AdvancedMechanismConstants.Turret.enc2GearTeeth / gearGreatestCommonDivisor);

        private static final double twoPi = 2.0 * Math.PI;
        private static final double turretRatio = (double) AdvancedMechanismConstants.Turret.turretTeeth / AdvancedMechanismConstants.Turret.idlerTeeth;

        private static final double combinedRatio = turretRatio * enc1ToEnc2Ratio;

        // Repeat period of the encoder-pair solution (in turret radians)
        private static final double turretPeriod =
                twoPi * (AdvancedMechanismConstants.Turret.idlerTeeth / (double) AdvancedMechanismConstants.Turret.turretTeeth) * enc1CyclesForPeriod;

        private static final double enc1ModSpan = twoPi / turretRatio;

        private static final int maxIterations = (int) Math.ceil(turretPeriod / enc1ModSpan) + 2;

        private TurretMath() {}

        public static double turretAngleFromEncoders(Rotation2d enc1Angle, Rotation2d enc2Angle) {
            return turretAngleFromEncoders(enc1Angle, enc2Angle, 0.00436); // 0.25 degrees in radians
        }

        public static double turretAngleFromEncoders(
                Rotation2d enc1Angle,
                Rotation2d enc2Angle,
                double tolerance
        ) {
            double baseSolution = mod(-enc1Angle.getRadians() / turretRatio, enc1ModSpan);

            for (int k = 0; k < maxIterations; k++) {
                double candidate = baseSolution + k * enc1ModSpan;
                if (candidate > turretPeriod + tolerance) {
                    break;
                }

                double predictedEnc2 = wrapToTwoPi(combinedRatio * candidate);
                double error = Math.abs(wrapToPi(predictedEnc2 - enc2Angle.getRadians()));

                if (error <= tolerance) {
                    // Shift from [0, turretPeriod) to AROUND 0 for nicer behavior
                    if (candidate > turretPeriod / 2.0) {
                        candidate -= turretPeriod;
                    }
                    return candidate;
                }
            }

            return 0;
        }

        /**
         * Given two absolute encoders (wrapped), the motor continuous position, and a desired turret angle
         * (assumed already unwrapped/true), return the motor position setpoint (continuous radians).
         *
         * @param enc1Angle absolute encoder 1 (Rotation2d)
         * @param enc2Angle absolute encoder 2 (Rotation2d)
         * @param motorPositionRad continuous motor position in radians (-inf..inf)
         * @param desiredTurretAngleRad desired turret angle in radians (continuous/unwrapped)
         * @param motorRadPerTurretRad motor radians per 1 turret radian (INCLUDE SIGN)
         * @param minTurretAngleRad mechanical min turret angle (radians)
         * @param maxTurretAngleRad mechanical max turret angle (radians)
         * @return motor setpoint position in radians (continuous)
         */
        public static double motorSetpointForTurretAngle(
                Rotation2d enc1Angle,
                Rotation2d enc2Angle,
                double motorPositionRad,
                double desiredTurretAngleRad,
                double minTurretAngleRad,
                double maxTurretAngleRad
        ) {
            return motorSetpointForTurretAngle(
                    turretAngleFromEncoders(enc1Angle, enc2Angle),
                    motorPositionRad, desiredTurretAngleRad,
                    minTurretAngleRad, maxTurretAngleRad
                    
            );
        }

        public static double motorSetpointForTurretAngle(
                double turretRep,
                double motorPositionRad,
                double desiredTurretAngleRad,
                double minTurretAngleRad,
                double maxTurretAngleRad
        ) {

            double desired = clamp(desiredTurretAngleRad, minTurretAngleRad, maxTurretAngleRad);

            // Continuous turret estimate from motor (requires motor was zeroed consistently)
            double turretFromMotor = motorPositionRad / AdvancedMechanismConstants.Turret.motorRadPerTurretRad;

            // Pick the correct equivalent of turretRep that:
            //  - is closest to turretFromMotor
            //  - and is within [min, max] if possible
            double turretAbs = closestEquivalentWithinLimits(
                    turretRep,
                    turretFromMotor,
                    turretPeriod,
                    minTurretAngleRad,
                    maxTurretAngleRad
            );

            // Compute motor setpoint to move turretAbs -> desired
            double turretDelta = desired - turretAbs;
            return motorPositionRad + turretDelta * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;
        }

        /** Choose (angleRep + k*period) closest to reference, preferring values inside [min,max]. */
        private static double closestEquivalentWithinLimits(
                double angleRep,
                double reference,
                double period,
                double min,
                double max
        ) {
            long k0 = Math.round((reference - angleRep) / period);

            double best = Double.NaN;
            double bestErr = Double.POSITIVE_INFINITY;

            // Check a small neighborhood around the best k. Range < period => typically only one valid.
            for (long k = k0 - 2; k <= k0 + 2; k++) {
                double candidate = angleRep + k * period;
                if (candidate < min - 1e-6 || candidate > max + 1e-6) {
                    continue;
                }
                double err = Math.abs(candidate - reference);
                if (err < bestErr) {
                    bestErr = err;
                    best = candidate;
                }
            }

            // If nothing lands inside limits (e.g., limits not configured / mismatch), fall back to closest.
            if (Double.isNaN(best)) {
                return angleRep + k0 * period;
            }
            return best;
        }

        private static double wrapToTwoPi(double angleRad) {
            double wrapped = angleRad % twoPi;
            return wrapped < 0 ? wrapped + twoPi : wrapped;
        }

        private static double wrapToPi(double angleRad) {
            double wrapped = (angleRad + Math.PI) % twoPi;
            if (wrapped < 0) {
                wrapped += twoPi;
            }
            return wrapped - Math.PI;
        }

        private static double mod(double value, double modulus) {
            double result = value % modulus;
            return result < 0 ? result + modulus : result;
        }

        private static double clamp(double x, double min, double max) {
            return Math.max(min, Math.min(max, x));
        }

        private static double greatestCommonDivisor(double a, double b) {
            final double EPS = 1e-10;
            a = Math.abs(a);
            b = Math.abs(b);
            if (a < EPS) return b;
            if (b < EPS) return a;
            while (b > EPS) {
                double temp = b;
                b = a % b;
                a = temp;
            }
            return a;
        }
    }
}
