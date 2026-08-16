package dev.lilkuzco.kinetics.propulsion;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.profile.EngineFrame;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.profile.Stage;

/**
 * Rocket performance accounting (RD1-RD5). Closed-form; no simulation.
 *
 * <p>This is the calculator a launch pipeline consults before it lets a vehicle off the pad,
 * and the reference the test battery checks the simulated burn against. Keeping it separate
 * from the integrator is deliberate: if the same code produced both the prediction and the
 * measurement, agreement between them would prove nothing.
 */
public final class Propulsion {

    private final Constants k;
    private final double g0;
    private final EngineFrame frame;

    public Propulsion(Constants k) {
        this.k = k;
        this.g0 = k.d("gravity.g0");
        this.frame = EngineFrame.of(k);
    }

    /**
     * Tsiolkovsky's rocket equation (RD1): {@code dv = Isp * g0 * ln(m0/mf)}.
     *
     * <p>The logarithm is the tyranny. Doubling your delta-v does not double the propellant, it
     * squares the mass ratio - which is why every real launch vehicle stages, and why the
     * shipped two-stage profile makes orbit while a single-stage one of the same total mass
     * does not.
     */
    public double deltaV(double isp, double initialMass, double finalMass) {
        if (finalMass <= 0.0 || initialMass <= finalMass) return 0.0;
        return frame.exhaustVelocity(isp) * Math.log(initialMass / finalMass);
    }

    /** Total ideal delta-v of a whole vehicle, vacuum Isp, staging included (RD1/RD4). */
    public double totalDeltaV(Profile profile) {
        return profile.idealDeltaVVacuum(frame);
    }

    /** Per-stage delta-v breakdown, in firing order. */
    public double[] stageDeltaV(Profile profile) {
        double[] out = new double[profile.stages().size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = deltaV(profile.stages().get(i).ispVacuum(),
                    profile.massAtIgnition(i), profile.massAtBurnout(i));
        }
        return out;
    }

    /**
     * Gravity loss during a vertical powered ascent (RD5): {@code g * t_burn}.
     *
     * <p>Every second spent fighting gravity instead of gaining horizontal speed is a second of
     * delta-v thrown away, which is why high thrust-to-weight matters and why a low-T/W vehicle
     * can have enough delta-v on paper and still fail to reach orbit. It is also why the
     * gravity turn exists - pitching over early converts the ascent into horizontal
     * acceleration and stops paying this.
     */
    public double gravityLoss(double gravity, double burnSeconds) {
        return gravity * burnSeconds;
    }

    /** Total burn time across all stages at full throttle, s. */
    public double totalBurnTime(Profile profile) {
        double t = 0.0;
        for (Stage s : profile.stages()) t += s.burnTime(frame);
        return t;
    }

    /**
     * A verdict on whether this design reaches orbit, with the reasoning attached.
     *
     * <p>Both gates must pass, and they fail for different reasons. A vehicle can have plenty
     * of delta-v and be unable to lift its own weight; another can leap off the pad and run out
     * of propellant at half orbital velocity.
     */
    public record LaunchAssessment(
            boolean canLiftOff,
            boolean hasDeltaV,
            double twrSeaLevel,
            double idealDeltaV,
            double requiredDeltaV,
            double estimatedGravityLoss,
            String verdict) {

        public boolean reachesOrbit() { return canLiftOff && hasDeltaV; }
    }

    public LaunchAssessment assess(Profile profile, double gravity) {
        double twr = profile.liftoffTwr(gravity, frame);
        double minTwr = k.d("propulsion.min_liftoff_twr");
        double typicalTwr = k.d("propulsion.typical_liftoff_twr");
        double ideal = totalDeltaV(profile);
        double required = k.d("orbit.delta_v_to_orbit");
        double burn = totalBurnTime(profile);
        // Only the vertical portion of the ascent pays the full gravity loss; a gravity turn
        // spends roughly the first third climbing before the velocity vector has rotated over.
        double gravLoss = gravityLoss(gravity, burn) / 3.0;

        boolean lifts = twr > minTwr;
        boolean hasDv = ideal >= required;

        String verdict;
        if (!lifts) {
            verdict = String.format(
                    "cannot lift off: sea-level T/W is %.3f, needs to exceed %.2f. "
                    + "The vehicle weighs more than its first stage can push.", twr, minTwr);
        } else if (!hasDv) {
            verdict = String.format(
                    "lifts off but falls short: %.1f m/s of ideal delta-v against a %.1f m/s "
                    + "budget, %.1f m/s short. Add propellant, stage, or raise Isp.",
                    ideal, required, required - ideal);
        } else if (twr < typicalTwr) {
            verdict = String.format(
                    "reaches orbit, but T/W of %.3f is below the healthy %.2f - expect roughly "
                    + "%.0f m/s of gravity loss eating into a %.0f m/s margin.",
                    twr, typicalTwr, gravLoss, ideal - required);
        } else {
            verdict = String.format(
                    "reaches orbit: T/W %.3f, %.1f m/s of delta-v against a %.1f m/s budget "
                    + "(%.1f m/s margin).", twr, ideal, required, ideal - required);
        }
        return new LaunchAssessment(lifts, hasDv, twr, ideal, required, gravLoss, verdict);
    }

    public double g0() { return g0; }
}
