package dev.lilkuzco.kinetics.env;

import dev.lilkuzco.kinetics.constants.Constants;

/**
 * Exponential atmosphere, RB1: {@code rho(h) = rho0 * e^(-h/H)}.
 *
 * <p>The only compression here is the scale height: 8500 m of real atmosphere folded into
 * 55 game metres so that the 257 m Minecraft can actually build in spans the whole density
 * profile. The consequence is worth stating plainly, because it drives the rest of the
 * library: <em>game altitude x 154.55 is the real altitude with the same air density.</em>
 * A body at y=313 is flying in air as thin as 38.6 km up, which is why drag stops mattering
 * there and why the Karman handoff sits at exactly that altitude.
 *
 * <p>Pressure follows density ({@code p / p0 = rho / rho0}) under this isothermal model. That
 * is the ratio {@link dev.lilkuzco.kinetics.propulsion.Propulsion} interpolates sea-level and
 * vacuum Isp against (RD2b).
 */
public final class Atmosphere {

    private final double rho0;
    private final double scaleHeight;
    private final double rhoFloor;
    private final double aSeaLevel;
    private final double aMin;
    private final double karmanAltitude;
    private final boolean present;

    private Atmosphere(Constants k, boolean present) {
        this.rho0 = k.d("atmosphere.rho_sea_level");
        this.scaleHeight = k.d("atmosphere.scale_height");
        this.rhoFloor = k.d("atmosphere.rho_floor");
        this.aSeaLevel = k.d("atmosphere.speed_of_sound_sea_level");
        this.aMin = k.d("atmosphere.speed_of_sound_min");
        this.karmanAltitude = k.d("atmosphere.karman_altitude_game");
        this.present = present;
    }

    public static Atmosphere standard(Constants k) { return new Atmosphere(k, true); }

    /** A dimension with no air: the Moon (Phase B). Drag, lift and parachutes are all zero. */
    public static Atmosphere vacuum(Constants k) { return new Atmosphere(k, false); }

    public boolean isPresent() { return present; }

    /**
     * Air density in kg/m^3 at altitude {@code h} metres above the sea-level datum.
     * Negative altitudes (caves, deep valleys) return the sea-level value rather than
     * extrapolating to an unphysically dense atmosphere.
     */
    public double density(double h) {
        if (!present) return 0.0;
        if (h <= 0.0) return rho0;
        double rho = rho0 * Math.exp(-h / scaleHeight);
        return rho < rhoFloor ? 0.0 : rho;
    }

    /** Density relative to sea level - also the pressure ratio under this model (RD2b). */
    public double pressureRatio(double h) {
        if (!present) return 0.0;
        return density(h) / rho0;
    }

    /**
     * Speed of sound in m/s (RB5). It tracks temperature rather than density, so it falls far
     * more gently than rho does; modelled as a linear fall from the sea-level value to the
     * cold upper value across the modelled column.
     */
    public double speedOfSound(double h) {
        if (!present) return aSeaLevel; // Mach is meaningless in vacuum; keep the divisor sane.
        double t = h / karmanAltitude;
        if (t <= 0.0) return aSeaLevel;
        if (t >= 1.0) return aMin;
        return aSeaLevel + (aMin - aSeaLevel) * t;
    }

    /** Mach number of a speed at an altitude. */
    public double mach(double speed, double h) {
        double a = speedOfSound(h);
        return a <= 0.0 ? 0.0 : speed / a;
    }

    /** Dynamic pressure q = 1/2 rho v^2 in Pa (RB6). */
    public double dynamicPressure(double airspeed, double h) {
        double rho = density(h);
        return 0.5 * rho * airspeed * airspeed;
    }

    public double seaLevelDensity() { return rho0; }

    public double karmanAltitude() { return karmanAltitude; }

    public double scaleHeight() { return scaleHeight; }
}
