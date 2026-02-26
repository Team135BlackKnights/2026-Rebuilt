package frc.robot.subsystems.Turret.azimuth;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

public class TurretMathematics {
  public static final class TurretMath {

    private static final double TWO_PI = 2.0 * Math.PI;

    private static final double TURRET_RATIO =
        (double) AdvancedMechanismConstants.Turret.turretTeeth
            / (double) AdvancedMechanismConstants.Turret.idlerTeeth; // 77/10

    private static final double ENC1_MOD_SPAN = TWO_PI / TURRET_RATIO; // turret radians per enc1 wrap

    private TurretMath() {}
public static double motorSetpointForTurretAngle(
    double turretRep,
    double motorPositionRad,
    double desiredTurretAngleRad,
    double minTurretAngleRad,
    double maxTurretAngleRad
) {
    // Clamp target to mechanical limits
    double desired = MathUtil.clamp(desiredTurretAngleRad, minTurretAngleRad, maxTurretAngleRad);

    // If the turret estimate is invalid, HOLD motor position (do not jump).
    // (Your IO code should be holding lastTurretAngleRads anyway.)
    if (Double.isNaN(turretRep)) {
        return motorPositionRad;
    }

    double turretNow = MathUtil.clamp(turretRep, minTurretAngleRad, maxTurretAngleRad);

    double turretDelta = desired - turretNow;

    return motorPositionRad + turretDelta * AdvancedMechanismConstants.Turret.motorRadPerTurretRad;
}
    public static double turretAngleFromEncodersRad(double enc1Rad0to2pi, double enc2Rad0to2pi, double tolRad, double enc1GearTeeth, double enc2GearTeeth) {
      return turretAngleFromEncodersRad(enc1Rad0to2pi, enc2Rad0to2pi, 0.0, tolRad, enc1GearTeeth, enc2GearTeeth);
    }

    /**
     * Phase-aware solve: uses a reference turret angle to infer encoder phases (offsets) and then
     * chooses the candidate closest to that reference.
     *
     * @param enc1Rad0to2pi wrapped 0..2π absolute
     * @param enc2Rad0to2pi wrapped 0..2π absolute
     * @param referenceTurretRad continuous-ish reference (motor-delta based is ideal)
     * @param tolRad tolerance
     * @param enc1GearTeeth tooth count for encoder 1 gear
     * @param enc2GearTeeth tooth count for encoder 2 gear
     */
    public static double turretAngleFromEncodersRad(
        double enc1Rad0to2pi,
        double enc2Rad0to2pi,
        double referenceTurretRad,
        double tolRad,
        double enc1GearTeeth,
        double enc2GearTeeth
    ) {
      final double enc1ToEnc2Ratio = enc1GearTeeth / enc2GearTeeth;
      final double gcd = greatestCommonDivisor(enc1GearTeeth, enc2GearTeeth);
      final int enc1CyclesForPeriod = (int) Math.round(enc2GearTeeth / gcd);
      final double turretPeriod = enc1CyclesForPeriod * ENC1_MOD_SPAN;
      final int maxIterations = (int) Math.ceil(turretPeriod / ENC1_MOD_SPAN) + 4;
      final double enc1 = wrapToTwoPi(enc1Rad0to2pi);
      final double enc2 = wrapToTwoPi(enc2Rad0to2pi);

      // Center reference into (-period/2, +period/2] so distance comparisons are stable
      final double ref = centerToPeriod(referenceTurretRad, turretPeriod);

      // Try both enc2 signs (because two meshes usually makes enc2 same direction as turret,
      // but SensorDirection / mounting can flip it)
      Candidate best = null;

      for (int enc2Sign : new int[] { +1, -1 }) {
        final double combined = enc2Sign * (TURRET_RATIO * enc1ToEnc2Ratio);

        // Estimate phases from reference:
        // enc1 ≈ wrap(-TURRET_RATIO*t + phi1) -> phi1 ≈ wrap(enc1 + TURRET_RATIO*ref)
        final double phi1 = wrapToTwoPi(enc1 + TURRET_RATIO * ref);

        // enc2 ≈ wrap(combined*t + phi2) -> phi2 ≈ wrap(enc2 - combined*ref)
        final double phi2 = wrapToTwoPi(enc2 - combined * ref);

        // Base turret candidate from enc1 using inferred phase
        // -TURRET_RATIO*t + phi1 = enc1 + 2πm -> t = (phi1 - enc1 + 2πm)/TURRET_RATIO
        final double base = mod((phi1 - enc1) / TURRET_RATIO, ENC1_MOD_SPAN);

        Candidate c = solveCandidates(base, enc2, phi2, combined, ref, tolRad, turretPeriod, maxIterations);
        if (c != null && (best == null || c.cost < best.cost)) best = c;
      }

      return (best == null) ? Double.NaN : best.turretRad;
    }

    private static Candidate solveCandidates(
        double base,
        double enc2,
        double phi2,
        double combined,
        double ref,
        double tolRad,
        double turretPeriod,
        int maxIterations
    ) {
      double bestTurret = Double.NaN;
      double bestErr = Double.POSITIVE_INFINITY;
      double bestDist = Double.POSITIVE_INFINITY;

      for (int k = 0; k < maxIterations; k++) {
        double candidate = base + k * ENC1_MOD_SPAN;
        if (candidate > turretPeriod + tolRad) break;

        // shift to around 0
        double t = (candidate > turretPeriod / 2.0) ? candidate - turretPeriod : candidate;

        double pred2 = wrapToTwoPi(combined * t + phi2);
        double err2 = Math.abs(wrapToPi(pred2 - enc2));

        if (err2 <= tolRad) {
          double dist = Math.abs(t - ref);
          if (dist < bestDist - 1e-12 || (Math.abs(dist - bestDist) <= 1e-12 && err2 < bestErr)) {
            bestDist = dist;
            bestErr = err2;
            bestTurret = t;
          }
        }
      }

      if (Double.isNaN(bestTurret)) return null;
      return new Candidate(bestTurret, bestDist * 10.0 + bestErr);
    }

    private static final class Candidate {
      final double turretRad;
      final double cost;
      Candidate(double turretRad, double cost) { this.turretRad = turretRad; this.cost = cost; }
    }

    private static double wrapToTwoPi(double a) {
      double w = a % TWO_PI;
      return w < 0 ? w + TWO_PI : w;
    }

    private static double wrapToPi(double a) {
      double w = (a + Math.PI) % TWO_PI;
      if (w < 0) w += TWO_PI;
      return w - Math.PI;
    }

    private static double mod(double v, double m) {
      double r = v % m;
      return r < 0 ? r + m : r;
    }

    private static double centerToPeriod(double x, double period) {
      double y = x % period;
      if (y <= -period / 2.0) y += period;
      if (y > period / 2.0) y -= period;
      return y;
    }

    private static double greatestCommonDivisor(double a, double b) {
      final double EPS = 1e-10;
      a = Math.abs(a);
      b = Math.abs(b);
      if (a < EPS) return b;
      if (b < EPS) return a;
      while (b > EPS) {
        double t = b;
        b = a % b;
        a = t;
      }
      return a;
    }
  }
}