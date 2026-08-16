package dev.lilkuzco.kinetics.env;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Everything a body needs to know about where it is flying: gravity, air, wind, terrain.
 *
 * <p>One instance per dimension. The Moon (Phase B) is this same class with a 0.1654 gravity
 * scalar and a vacuum atmosphere - no special-casing anywhere downstream, which is what makes
 * "parachutes are useless on the Moon" a consequence of the physics rather than a rule
 * somebody wrote.
 */
public final class Environment {

    private final Constants constants;
    private final Atmosphere atmosphere;
    private final WindField wind;
    private final WorldProbe world;
    private final double gravityScalar;
    private final double seaLevelY;
    private final double metersPerBlock;
    private final double g0;

    public Environment(Constants constants, Atmosphere atmosphere, WindField wind,
                       WorldProbe world, double gravityScalar) {
        this.constants = constants;
        this.atmosphere = atmosphere;
        this.wind = wind;
        this.world = world;
        this.gravityScalar = gravityScalar;
        this.seaLevelY = constants.d("world.sea_level_y");
        this.metersPerBlock = constants.d("world.meters_per_block");
        this.g0 = constants.d("gravity.g0");
    }

    /** Standard overworld: 1 g, standard air, no wind, empty world. The test default. */
    public static Environment overworld(Constants k) {
        return new Environment(k, Atmosphere.standard(k), WindField.disabled(k),
                WorldProbe.empty(), k.d("gravity.dimension_scalars.overworld"));
    }

    public static Environment overworld(Constants k, WorldProbe world) {
        return new Environment(k, Atmosphere.standard(k), WindField.disabled(k),
                world, k.d("gravity.dimension_scalars.overworld"));
    }

    /** Airless 0.1654 g body (RA2). Phase B's Moon. */
    public static Environment moon(Constants k, WorldProbe world) {
        return new Environment(k, Atmosphere.vacuum(k), WindField.disabled(k),
                world, k.d("gravity.dimension_scalars.moon"));
    }

    public Environment withWind(WindField w) {
        return new Environment(constants, atmosphere, w, world, gravityScalar);
    }

    public Environment withWorld(WorldProbe w) {
        return new Environment(constants, atmosphere, wind, w, gravityScalar);
    }

    /** Gravitational acceleration in m/s^2, positive magnitude. */
    public double gravity() { return g0 * gravityScalar; }

    /** Gravity as an acceleration vector (downward). */
    public Vec3 gravityVector() { return new Vec3(0.0, -gravity(), 0.0); }

    /** Altitude in metres above the sea-level datum, from a world y coordinate. */
    public double altitudeOf(double worldY) {
        return (worldY - seaLevelY) * metersPerBlock;
    }

    /** Inverse of {@link #altitudeOf}. */
    public double worldYOf(double altitude) {
        return seaLevelY + altitude / metersPerBlock;
    }

    /**
     * The airspeed vector a body actually feels: its velocity minus the local wind (RB7).
     * With wind off this is just the velocity, which is why every drag and lift call site can
     * use this unconditionally and I5 ("drag opposes the airspeed vector") holds either way.
     */
    public Vec3 airspeedOf(Vec3 position, Vec3 velocity, double worldTimeSeconds) {
        if (!wind.isEnabled()) return velocity;
        return velocity.sub(wind.at(new Vec3(position.x(), altitudeOf(position.y()), position.z()),
                worldTimeSeconds));
    }

    public double densityAt(double worldY) { return atmosphere.density(altitudeOf(worldY)); }

    public double pressureRatioAt(double worldY) {
        return atmosphere.pressureRatio(altitudeOf(worldY));
    }

    public double machAt(double speed, double worldY) {
        return atmosphere.mach(speed, altitudeOf(worldY));
    }

    public double dynamicPressureAt(double airspeed, double worldY) {
        return atmosphere.dynamicPressure(airspeed, altitudeOf(worldY));
    }

    /** World y of the virtual Karman line - the orbital-insertion handoff altitude. */
    public double karmanWorldY() { return worldYOf(atmosphere.karmanAltitude()); }

    public Atmosphere atmosphere() { return atmosphere; }

    public WindField wind() { return wind; }

    public WorldProbe world() { return world; }

    public Constants constants() { return constants; }

    public double gravityScalar() { return gravityScalar; }
}
