package dev.lilkuzco.kinetics.invariant;

import dev.lilkuzco.kinetics.body.BodyState;
import dev.lilkuzco.kinetics.profile.Profile;

/**
 * A breach of the physics constitution (Section 0).
 *
 * <p>This is thrown, not logged. A violated invariant means the model is wrong, and a sim that
 * carries on past that point produces trajectories nobody can trust and golden hashes that
 * encode the bug. The message carries the offending profile and the full state that broke it,
 * because "NaN somewhere in flight" is not a diagnosable report - I1 explicitly requires the
 * dump.
 *
 * <p>The standing rule when one of these fires: fix the model, never special-case the symptom.
 */
public final class InvariantViolation extends RuntimeException {

    private final String invariant;
    private final BodyState state;

    public InvariantViolation(String invariant, String detail, Profile profile, BodyState state) {
        super(render(invariant, detail, profile, state));
        this.invariant = invariant;
        this.state = state;
    }

    public String invariant() { return invariant; }

    public BodyState state() { return state; }

    private static String render(String invariant, String detail, Profile profile, BodyState state) {
        StringBuilder sb = new StringBuilder();
        sb.append("PHYSICS INVARIANT ").append(invariant).append(" BREACHED\n");
        sb.append("  ").append(detail).append('\n');
        if (profile != null) {
            sb.append("  profile: ").append(profile.id())
              .append("  dry=").append(fmt(profile.payloadDryMass())).append(" kg")
              .append("  stages=").append(profile.stages().size())
              .append("  refArea=").append(fmt(profile.airframe().referenceArea())).append(" m2")
              .append("  cd0=").append(fmt(profile.airframe().cd0()))
              .append("  gLimit=").append(fmt(profile.airframe().gLimitG())).append(" g\n");
        }
        if (state != null) {
            sb.append("  state:   ").append(state.toLine()).append('\n');
        }
        sb.append("  This is a P0 physics bug. Fix the model; do not special-case the symptom.");
        return sb.toString();
    }

    private static String fmt(double v) {
        return String.format("%.4g", v);
    }
}
