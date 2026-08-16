package dev.lilkuzco.kinetics.orbit;

/**
 * A circular orbit, expressed as elements referred to an epoch (RE5).
 *
 * <p><b>Nothing here is ticked.</b> The elements are constants of the orbit and the position at
 * any world time is computed from them directly. That is not an optimisation, it is a
 * correctness requirement: a satellite whose position accumulated tick by tick would stop
 * advancing the moment its chunk unloaded, drift out of step with a satellite in a loaded
 * chunk, and produce a different trajectory on a laggy server than on a smooth one. Computing
 * from epoch means orbits progress while nobody is watching and two servers running the same
 * world agree exactly (I7).
 *
 * <p>Eccentricity is carried but v0.1 only ships circular orbits; elliptical propagation is
 * fenced to v0.2 behind {@link OrbitalMechanics#visViva}. The field exists so that adding them
 * is not a signature change for consumers.
 *
 * @param id                     registry identifier
 * @param epochSeconds           world time the elements refer to
 * @param semiMajorAxisAtEpoch   metres from the planet's centre
 * @param eccentricity           0 in v0.1
 * @param inclinationDeg         0 = equatorial, 90 = polar. Caps the latitude the ground track
 *                               can reach, which is why a polar orbit is what a recon satellite
 *                               wants
 * @param raanDeg                right ascension of the ascending node - which way round the
 *                               planet the orbital plane is turned
 * @param argumentOfLatitudeAtEpochDeg where along the orbit the satellite sat at epoch
 */
public record Orbit(
        String id,
        double epochSeconds,
        double semiMajorAxisAtEpoch,
        double eccentricity,
        double inclinationDeg,
        double raanDeg,
        double argumentOfLatitudeAtEpochDeg) {

    /** A circular orbit at a given altitude. */
    public static Orbit circular(String id, OrbitalMechanics mechanics, double epochSeconds,
                                 double altitude, double inclinationDeg, double raanDeg,
                                 double argumentOfLatitudeDeg) {
        return new Orbit(id, epochSeconds, mechanics.radiusForAltitude(altitude), 0.0,
                inclinationDeg, raanDeg, argumentOfLatitudeDeg);
    }

    public boolean isCircular() { return eccentricity < 1e-9; }

    public Orbit withSemiMajorAxis(double a) {
        return new Orbit(id, epochSeconds, a, eccentricity, inclinationDeg, raanDeg,
                argumentOfLatitudeAtEpochDeg);
    }

    /**
     * Re-refer the elements to a new epoch. Used when an orbit is manoeuvred: the new state
     * becomes the epoch state, so propagation stays a pure function of elapsed time.
     */
    public Orbit rebasedTo(double newEpoch, double semiMajorAxis, double argumentOfLatitudeDeg) {
        return new Orbit(id, newEpoch, semiMajorAxis, eccentricity, inclinationDeg, raanDeg,
                argumentOfLatitudeDeg);
    }
}
