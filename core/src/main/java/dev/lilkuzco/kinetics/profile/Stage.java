package dev.lilkuzco.kinetics.profile;

/**
 * One propulsive stage (RD2, RD2b, RD4).
 *
 * <p>Mass flow is <b>derived, never declared</b>: {@code mdot = F_vac / v_e_vac}. That is RD2's
 * "one door" requirement and it is what makes I4 checkable - burn time, fuel consumption and
 * achieved delta-v all follow from the same number, so a profile cannot declare a burn time
 * that disagrees with its own fuel load.
 *
 * <p>Thrust varies with altitude for the same reason Isp does. Mass flow through the throat is
 * essentially fixed by the chamber and the throat area, so as ambient pressure falls and the
 * nozzle stops fighting the atmosphere, the same propellant flow produces more thrust. A first
 * stage really is weaker at the pad than it is at altitude, and RD5's T/W liftoff gate is
 * evaluated with the sea-level figure precisely because that is the one that has to lift the
 * vehicle off the ground.
 *
 * @param engineName    label for logs and errors
 * @param fuelMass      usable propellant, kg
 * @param stageDryMass  structure shed at staging, kg
 * @param thrustVacuum  vacuum thrust, N
 * @param ispSeaLevel   specific impulse at sea level, s (real-world value)
 * @param ispVacuum     specific impulse in vacuum, s (real-world value)
 */
public record Stage(
        String engineName,
        double fuelMass,
        double stageDryMass,
        double thrustVacuum,
        double ispSeaLevel,
        double ispVacuum) {

    /** Mass flow rate, kg/s. The single derivation everything else hangs off (RD2). */
    public double massFlow(EngineFrame frame) {
        double ve = frame.exhaustVelocity(ispVacuum);
        if (ve <= 0.0) return 0.0;
        return thrustVacuum / ve;
    }

    /** Burn duration at full throttle, s. Derived from the fuel load, never declared. */
    public double burnTime(EngineFrame frame) {
        double mdot = massFlow(frame);
        return mdot <= 0.0 ? 0.0 : fuelMass / mdot;
    }

    /**
     * Effective specific impulse at an ambient pressure ratio (RD2b). Linear interpolation
     * between the sea-level and vacuum figures against {@code p/p0}, which under the RB1
     * exponential atmosphere equals {@code rho/rho0}.
     *
     * @param pressureRatio ambient pressure over sea-level pressure: 1 at the pad, 0 in vacuum
     */
    public double effectiveIsp(double pressureRatio) {
        double r = pressureRatio;
        if (r < 0.0) r = 0.0;
        if (r > 1.0) r = 1.0;
        return ispVacuum + (ispSeaLevel - ispVacuum) * r;
    }

    /** Effective exhaust velocity at an ambient pressure ratio, m/s. */
    public double effectiveExhaustVelocity(double pressureRatio, EngineFrame frame) {
        return frame.exhaustVelocity(effectiveIsp(pressureRatio));
    }

    /** Thrust at an ambient pressure ratio, N. Mass flow is constant; exhaust velocity is not. */
    public double effectiveThrust(double pressureRatio, EngineFrame frame) {
        return massFlow(frame) * effectiveExhaustVelocity(pressureRatio, frame);
    }

    /** Effective exhaust velocity in vacuum, m/s. */
    public double exhaustVelocityVacuum(EngineFrame frame) {
        return frame.exhaustVelocity(ispVacuum);
    }

    /** Total mass of this stage when full: propellant plus its own structure. */
    public double wetMass() { return fuelMass + stageDryMass; }
}
