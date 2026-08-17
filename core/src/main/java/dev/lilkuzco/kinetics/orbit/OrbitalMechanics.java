package dev.lilkuzco.kinetics.orbit;

import dev.lilkuzco.kinetics.constants.Constants;

/**
 * Closed-form two-body mechanics (RE1, RE2, RE3).
 *
 * <p>Everything here is exact, so it is what the simulation and the registry get checked
 * against rather than something derived from them.
 *
 * <p>The gravitational parameter is <b>derived, not chosen</b>: {@code mu = g0 * R^2}. Picking
 * mu independently of surface gravity is the usual way a space mod ends up with orbits that do
 * not match the gravity players feel standing on the ground. Deriving it means the two can
 * never drift apart - the price is that the planet's radius becomes the free parameter, and it
 * came out at 220 km, a compact dense world. That fiction is stated in the scale audit rather
 * than buried.
 */
public final class OrbitalMechanics {

    private final double mu;
    private final double planetRadius;
    private final double dayPeriod;
    private final double referenceAltitude;
    private final double minSustainableAltitude;
    private final double lunarDistance;
    private final double lunarTransferDeltaV;

    public OrbitalMechanics(Constants k) {
        this.mu = k.d("orbit.mu");
        this.planetRadius = k.d("orbit.planet_radius");
        this.dayPeriod = k.d("world.day_seconds");
        this.referenceAltitude = k.d("orbit.reference_orbit_altitude");
        this.minSustainableAltitude = k.d("orbit.minimum_sustainable_altitude");
        this.lunarDistance = k.d("orbit.lunar_distance");
        this.lunarTransferDeltaV = k.d("orbit.lunar_transfer_delta_v");
    }

    /** Circular orbital speed at radius r (RE1): {@code v = sqrt(mu/r)}. */
    public double circularVelocity(double radius) {
        if (radius <= 0.0) return 0.0;
        return Math.sqrt(mu / radius);
    }

    /** Orbital period at semi-major axis a (RE1): {@code T = 2*pi*sqrt(a^3/mu)}. */
    public double period(double semiMajorAxis) {
        if (semiMajorAxis <= 0.0) return 0.0;
        return 2.0 * Math.PI * Math.sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / mu);
    }

    /**
     * Time to coast a Hohmann transfer between two circular radii (RE2): half the period of the
     * transfer ellipse, {@code t = pi*sqrt(a^3/mu)} with {@code a = (r1+r2)/2}.
     *
     * <p>Exact, and the reason a trans-lunar coast has a duration nobody had to invent. Between the
     * reference orbit and the Moon it comes out at 99,747 s - 27.7 hours, against reality's 3.0
     * days for the same manoeuvre. Consumers are free to play that back faster; the number itself
     * is not negotiable.
     */
    public double hohmannTransferTime(double r1, double r2) {
        if (r1 <= 0.0 || r2 <= 0.0) return 0.0;
        return period((r1 + r2) / 2.0) / 2.0;
    }

    /**
     * Delta-v for the first burn of a Hohmann transfer (RE2): the difference between the transfer
     * ellipse's perigee speed and the circular speed already held.
     */
    public double hohmannFirstBurn(double r1, double r2) {
        if (r1 <= 0.0 || r2 <= 0.0) return 0.0;
        return Math.abs(visViva(r1, (r1 + r2) / 2.0) - circularVelocity(r1));
    }

    /** Distance to the Moon, m. Scaled from reality by the same factor as the planet's radius. */
    public double lunarDistance() { return lunarDistance; }

    /** Budgeted trans-lunar injection cost from the reference orbit, m/s. */
    public double lunarTransferDeltaV() { return lunarTransferDeltaV; }

    /** Coast time from the reference orbit to the Moon, s of simulated time. */
    public double lunarTransferTime() {
        return hohmannTransferTime(radiusForAltitude(referenceAltitude), lunarDistance);
    }

    /** Mean motion, rad/s. */
    public double meanMotion(double semiMajorAxis) {
        if (semiMajorAxis <= 0.0) return 0.0;
        return Math.sqrt(mu / (semiMajorAxis * semiMajorAxis * semiMajorAxis));
    }

    /**
     * Escape velocity at radius r (RE2): {@code sqrt(2*mu/r)}, which is exactly
     * {@code sqrt(2)} times the circular velocity there. The test battery asserts that identity
     * rather than the formula, because the ratio is the thing that has to hold.
     */
    public double escapeVelocity(double radius) {
        if (radius <= 0.0) return 0.0;
        return Math.sqrt(2.0 * mu / radius);
    }

    /**
     * Vis-viva (RE3): {@code v^2 = mu*(2/r - 1/a)}.
     *
     * <p>Scaffolded and correct, but v0.1 only ever calls it with {@code r == a}, which reduces
     * to the circular case. Elliptical orbits and Hohmann transfers are fenced to v0.2 - the
     * API is here so that adding them is not a signature change for consumers.
     */
    public double visViva(double radius, double semiMajorAxis) {
        if (radius <= 0.0 || semiMajorAxis <= 0.0) return 0.0;
        double vSq = mu * (2.0 / radius - 1.0 / semiMajorAxis);
        return vSq <= 0.0 ? 0.0 : Math.sqrt(vSq);
    }

    /** Semi-major axis from a position and speed. Inverse of vis-viva. */
    public double semiMajorAxisFrom(double radius, double speed) {
        double denom = 2.0 / radius - (speed * speed) / mu;
        if (denom <= 0.0) return Double.POSITIVE_INFINITY; // escape trajectory
        return 1.0 / denom;
    }

    /** Specific orbital energy, J/kg: {@code -mu/(2a)}. Negative for a bound orbit. */
    public double specificEnergy(double semiMajorAxis) {
        if (semiMajorAxis <= 0.0 || Double.isInfinite(semiMajorAxis)) return 0.0;
        return -mu / (2.0 * semiMajorAxis);
    }

    /**
     * Westward ground-track shift per revolution, degrees of longitude (RE5b):
     * {@code (T_orbit / T_day) * 360}.
     *
     * <p>The planet turns underneath the orbit. After one revolution the satellite is back over
     * the same inertial direction, but the ground beneath has moved, so it crosses a different
     * longitude. Predicting successive passes in the non-rotating frame gives answers that are
     * simply wrong, and increasingly wrong with each orbit.
     *
     * <p>At the reference altitude the period is 1199.6 s against a 1200 s day, so the shift is
     * 359.88 degrees - within a tenth of a degree of repeating. That near-resonance is a
     * feature: the reference orbit passes over the same ground at the same time each day.
     */
    public double groundTrackShiftPerOrbit(double semiMajorAxis) {
        return (period(semiMajorAxis) / dayPeriod) * 360.0;
    }

    /** Planet rotation rate, degrees per second. */
    public double planetRotationRate() { return 360.0 / dayPeriod; }

    /** Whether an orbit at this altitude decays (RE4). */
    public boolean decays(double altitude) { return altitude < minSustainableAltitude; }

    public double radiusForAltitude(double altitude) { return planetRadius + altitude; }

    public double altitudeForRadius(double radius) { return radius - planetRadius; }

    public double mu() { return mu; }

    public double planetRadius() { return planetRadius; }

    public double referenceAltitude() { return referenceAltitude; }

    public double minSustainableAltitude() { return minSustainableAltitude; }

    public double dayPeriod() { return dayPeriod; }
}
