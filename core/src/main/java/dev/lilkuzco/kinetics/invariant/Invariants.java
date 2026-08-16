package dev.lilkuzco.kinetics.invariant;

import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Runtime enforcement of the physics constitution (Section 0).
 *
 * <p>Only the invariants that can be violated by a coding mistake are checked here. Several of
 * the twelve are enforced by construction instead, and that is the stronger arrangement -
 * there is no code path that could break them, so there is nothing to check:
 *
 * <ul>
 *   <li><b>I2</b> - the g-limit clamp is applied inside the integrator before the acceleration
 *       reaches the state update; no unclamped value exists.</li>
 *   <li><b>I4</b> - mass is derived from stage state rather than stored, and the only fuel
 *       path is {@link KineticBody#burn}.</li>
 *   <li><b>I5</b> - drag is built as a negative multiple of the airspeed unit vector, so it
 *       cannot fail to oppose it; the stall bound lives in the lift curve.</li>
 *   <li><b>I6</b> - turn authority is whatever lift the airframe can make at the current q,
 *       so it is speed-dependent by construction.</li>
 *   <li><b>I9</b> - the constants loader throws on a missing key.</li>
 *   <li><b>I10</b> - the event hierarchy is sealed and carries no damage field.</li>
 * </ul>
 *
 * <p>What remains - finiteness, continuity, energy honesty, quaternion sanity - can only be
 * verified after the fact, and is checked every substep.
 */
public final class Invariants {

    private final double maxSpeed;
    private final double maxSubstepScaleHeightFraction;
    private final double scaleHeight;
    private final double quaternionTolerance;
    private final double energyGainTolerance;

    /** Set false only in the fuzz harness, which converts violations into counted failures. */
    private boolean throwOnViolation = true;

    public Invariants(Constants k) {
        this.maxSpeed = k.d("limits.max_speed");
        this.maxSubstepScaleHeightFraction = k.d("limits.max_substep_scale_height_fraction");
        this.scaleHeight = k.d("atmosphere.scale_height");
        this.quaternionTolerance = k.d("limits.quaternion_norm_tolerance");
        this.energyGainTolerance = k.d("limits.energy_gain_tolerance_per_tick");
    }

    public Invariants collecting() { this.throwOnViolation = false; return this; }

    /**
     * I1 - continuity and finiteness. Any NaN or infinity anywhere in the state is a hard
     * failure; so is a speed above the ceiling, which is not clamped because a body moving
     * that fast means something upstream has already gone wrong.
     */
    public void checkContinuity(KineticBody body, Vec3 displacement, int substeps,
                                double airDensity) {
        if (!body.position().isFinite() || !body.velocity().isFinite()) {
            fail(body, "I1", "position or velocity is not finite: pos=" + body.position()
                    + " vel=" + body.velocity());
        }
        if (!body.orientation().isFinite()) {
            fail(body, "I1", "orientation quaternion is not finite: " + body.orientation());
        }
        if (!Double.isFinite(body.mass()) || body.mass() <= 0.0) {
            fail(body, "I1", "mass is not a positive finite number: " + body.mass());
        }
        double speed = body.speed();
        if (speed > maxSpeed) {
            fail(body, "I1", String.format(
                    "speed %.3f m/s exceeds the ceiling of %.1f m/s", speed, maxSpeed));
        }
        // The substep budget is an accuracy bound, not a collision one - swept voxel traversal
        // already makes tunnelling impossible at any step size. What it protects is the air
        // density: cross too much of a scale height in one substep and the density driving
        // drag is stale enough that the trajectory stops meaning anything. Outside the
        // atmosphere there is nothing to go stale, so the bound does not apply.
        if (airDensity > 0.0) {
            double step = displacement.length();
            double allowed = maxSubstepScaleHeightFraction * scaleHeight;
            if (step > allowed) {
                fail(body, "I1", String.format(
                        "substep displacement %.4f m exceeds %.4f m (%.0f%% of the %.1f m scale "
                        + "height) at %d substeps - the density used for drag is stale",
                        step, allowed, maxSubstepScaleHeightFraction * 100.0, scaleHeight,
                        substeps));
            }
        }
    }

    /** I12 - the orientation quaternion must stay on the unit sphere. */
    public void checkQuaternion(KineticBody body) {
        double norm = body.orientation().norm();
        if (Math.abs(norm - 1.0) > quaternionTolerance) {
            fail(body, "I12", String.format(
                    "quaternion norm %.12f deviates from unity by %.3g, tolerance %.3g",
                    norm, Math.abs(norm - 1.0), quaternionTolerance));
        }
        if (!body.angularVelocity().isFinite()) {
            fail(body, "I12", "angular velocity is not finite: " + body.angularVelocity());
        }
    }

    /**
     * I3 - energy honesty. An unpowered body's mechanical energy may only fall.
     *
     * <p>Semi-implicit Euler loses a fixed {@code g^2*dt^2/2} per step by construction, so the
     * expected sign is always negative and any measurable <em>gain</em> means energy is being
     * created - by a lift vector that is not quite perpendicular, a drag term with the wrong
     * sign, or a guidance command leaking into the state update.
     *
     * <p>The tolerance is relative to an energy scale rather than to the energy itself,
     * because specific energy passes through zero when a body crosses the sea-level datum and
     * a percentage of nearly-nothing is not a usable bound.
     *
     * <p><b>Wind is a genuine exception, and the fuzz harness found it.</b> With RB7 wind
     * enabled, drag opposes the airspeed rather than the ground velocity, so a body being blown
     * along really does gain mechanical energy in the world frame - the wind does work on it,
     * exactly as it does on a leaf. That is correct physics, not free energy, so the check
     * allows it up to the most work the wind could possibly have done:
     * {@code |F_drag| * |v_wind| * dt / m}. With wind off the allowance is exactly zero and the
     * strict form of the invariant applies unchanged, which is the case that matters for every
     * golden trajectory.
     *
     * @param windWorkAllowance upper bound on specific energy the wind could have added, J/kg
     */
    public void checkEnergy(KineticBody body, double before, double after,
                            double gravity, double altitude, double windWorkAllowance) {
        double gain = after - before - windWorkAllowance;
        if (gain <= 0.0) return;
        double scale = 0.5 * body.velocity().lengthSq() + gravity * Math.abs(altitude) + 1.0;
        double relative = gain / scale;
        if (relative > energyGainTolerance) {
            fail(body, "I3", String.format(
                    "unpowered body gained %.6g J/kg of mechanical energy in one step "
                    + "(%.4f%% of the %.6g J/kg energy scale, tolerance %.4f%%). "
                    + "Coasting bodies may only lose energy to drag - this is free energy.",
                    gain, relative * 100.0, scale, energyGainTolerance * 100.0));
        }
    }

    /**
     * I4 - thrust with no propellant is a breach, not a no-op.
     *
     * <p>The fuel state that matters is the one <em>before</em> the burn, not after. A stage
     * that spends its last kilogram this substep legitimately produces thrust from it and ends
     * the substep empty; testing the post-burn state would flag every burnout as a violation
     * while missing the thing the invariant is actually for - an engine that produces thrust it
     * never paid for.
     */
    public void checkThrust(KineticBody body, double thrust, boolean hadFuelBeforeBurn) {
        if (thrust > 0.0 && !hadFuelBeforeBurn) {
            fail(body, "I4", String.format(
                    "produced %.3f N of thrust in stage %d, which had no propellant at the start "
                    + "of the substep", thrust, body.stageIndex()));
        }
        if (body.mass() < body.dryMassFloor() - 1e-9) {
            fail(body, "I4", String.format(
                    "mass %.6f kg has fallen below the dry-mass floor of %.6f kg",
                    body.mass(), body.dryMassFloor()));
        }
    }

    /** I5 - drag must oppose the airspeed vector, never assist it. */
    public void checkDrag(KineticBody body, Vec3 dragForce, Vec3 airspeed) {
        if (dragForce.lengthSq() < 1e-24 || airspeed.lengthSq() < 1e-18) return;
        double alignment = dragForce.dot(airspeed);
        if (alignment > 0.0) {
            fail(body, "I5", String.format(
                    "drag force %s has a positive component along airspeed %s (dot=%.6g) - "
                    + "drag is accelerating the body", dragForce, airspeed, alignment));
        }
    }

    private void fail(KineticBody body, String invariant, String detail) {
        InvariantViolation v = new InvariantViolation(
                invariant, detail, body.profile(), body.snapshot());
        if (throwOnViolation) throw v;
        lastViolation = v;
    }

    private InvariantViolation lastViolation;

    /** Most recent violation when collecting rather than throwing. */
    public InvariantViolation lastViolation() { return lastViolation; }

    public void clearViolation() { lastViolation = null; }
}
