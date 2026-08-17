package dev.lilkuzco.kinetics.profile;

import dev.lilkuzco.kinetics.constants.Constants;

import java.util.List;

/**
 * A complete flyable body description. Loaded from JSON, validated at load, immutable after.
 *
 * <p>Consumers - cosmos, warfront, naval, aircraft - describe what they are flying here and
 * nowhere else. Everything the sim needs is derivable from this record, which is what lets a
 * rocket's ability to reach orbit be a property of its design (RD1/RD3/RD5) rather than a
 * flag somebody set.
 *
 * @param id             namespaced identifier, e.g. {@code cosmos:rocket_tier1}
 * @param payloadDryMass mass that never stages away: payload plus final-stage structure, kg
 * @param stages         propulsive stages in firing order; empty for an unpowered body
 * @param substeps       physics substeps per tick; 0 means "use the adaptive default" (RA1)
 * @param cep            circular error probable for unguided fire, m
 */
public record Profile(
        String id,
        double payloadDryMass,
        List<Stage> stages,
        Airframe airframe,
        Recovery recovery,
        SeekerSpec seeker,
        int substeps,
        double maxSlewRateDeg,
        double cep) {

    public Profile {
        stages = List.copyOf(stages);
        // A body must still have mass after its last stage is shed, because kinetics DOES shed it
        // - see FlightDirector.onFinalBurnout. A profile with no payload mass therefore ends the
        // flight at zero kg, and the very next force divided by that mass is NaN. I1 catches it,
        // loudly and correctly, several seconds after the profile that caused it went in.
        //
        // Rejecting it here turns a mid-flight P0 into an unmistakable message at construction.
        // There is no such thing as a massless vehicle; the payload figure is the airframe that
        // never stages away, and it is zero only by mistake.
        if (!(payloadDryMass > 0.0)) {
            throw new IllegalArgumentException("profile '" + id + "' has payloadDryMass "
                    + payloadDryMass + "; a body must have mass after its last stage is shed, "
                    + "or the flight ends at zero kg and every force becomes NaN");
        }
    }

    public boolean isPowered() { return !stages.isEmpty(); }

    /** Total mass on the pad: payload plus every stage, full. */
    public double wetMass() {
        double m = payloadDryMass;
        for (Stage s : stages) m += s.wetMass();
        return m;
    }

    /** Mass at the moment stage {@code index} ignites - everything from that stage up. */
    public double massAtIgnition(int index) {
        double m = payloadDryMass;
        for (int i = index; i < stages.size(); i++) m += stages.get(i).wetMass();
        return m;
    }

    /** Mass at burnout of stage {@code index}, before its structure is shed. */
    public double massAtBurnout(int index) {
        return massAtIgnition(index) - stages.get(index).fuelMass();
    }

    /**
     * Ideal delta-v by Tsiolkovsky, summed over stages (RD1, RD4):
     * {@code dv = sum_k Isp_k * g0 * ln(m0_k / mf_k)}.
     *
     * <p>Vacuum Isp throughout, so this is the optimistic upper bound - the figure a design is
     * judged against. What a vehicle actually achieves is lower, because the first stage burns
     * through thick air at reduced Isp and pays gravity losses on the way up. The gap between
     * this number and the achieved number is exactly the loss budget RD3 talks about.
     *
     * <p>Staging shows up here as a mass-ratio improvement: shedding a spent stage's structure
     * means the next stage's {@code m0/mf} is computed against a lighter vehicle.
     */
    public double idealDeltaVVacuum(EngineFrame frame) {
        double total = 0.0;
        for (int i = 0; i < stages.size(); i++) {
            double m0 = massAtIgnition(i);
            double mf = massAtBurnout(i);
            if (mf <= 0.0 || m0 <= mf) continue;
            total += stages.get(i).exhaustVelocityVacuum(frame) * Math.log(m0 / mf);
        }
        return total;
    }

    /**
     * Thrust-to-weight at liftoff, evaluated with <b>sea-level</b> thrust (RD5).
     *
     * <p>Using vacuum thrust here would be the classic paper-rocket error: it flatters the
     * design by roughly 10% for a kerolox first stage, which is enough to turn a vehicle that
     * cannot actually leave the pad into one that looks fine on the spreadsheet.
     */
    public double liftoffTwr(double gravity, EngineFrame frame) {
        if (stages.isEmpty()) return 0.0;
        double thrustSl = stages.get(0).effectiveThrust(1.0, frame);
        double weight = wetMass() * gravity;
        return weight <= 0.0 ? 0.0 : thrustSl / weight;
    }

    /** Whether this vehicle can leave the pad at all (RD5). */
    public boolean canLiftOff(Constants k, double gravity) {
        return liftoffTwr(gravity, EngineFrame.of(k)) > k.d("propulsion.min_liftoff_twr");
    }

    /** Whether the ideal delta-v clears the budget to orbit (RD3). */
    public boolean canReachOrbit(Constants k) {
        return idealDeltaVVacuum(EngineFrame.of(k)) >= k.d("orbit.delta_v_to_orbit");
    }

    /** Substeps to use, resolving 0 to the configured default (RA1). */
    public int effectiveSubsteps(Constants k) {
        return substeps > 0 ? substeps : k.i("limits.default_substeps");
    }

    /** Total usable propellant across all stages, kg. */
    public double totalFuel() {
        double f = 0.0;
        for (Stage s : stages) f += s.fuelMass();
        return f;
    }
}
