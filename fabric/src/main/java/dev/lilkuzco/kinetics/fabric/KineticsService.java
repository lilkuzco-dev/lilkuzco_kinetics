package dev.lilkuzco.kinetics.fabric;

import dev.lilkuzco.kinetics.body.BodyState;
import dev.lilkuzco.kinetics.body.KineticBody;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.env.Atmosphere;
import dev.lilkuzco.kinetics.env.Environment;
import dev.lilkuzco.kinetics.env.WindField;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.integrate.Integrator;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.orbit.OrbitalRegistry;
import dev.lilkuzco.kinetics.phase.FlightDirector;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.sensors.Countermeasures;
import net.minecraft.resources.ResourceKey;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * What consumers talk to. One instance per server.
 *
 * <p>Cosmos, warfront, naval and aircraft all use this and nothing below it. The contract is
 * narrow on purpose: hand kinetics a profile and an initial state, get back a handle, receive
 * events. Nothing here accepts or returns damage, and nothing here lets a consumer reach into
 * the integrator (I10).
 *
 * <p>Simulation is <b>server-authoritative</b>. Clients receive {@link BodyState} snapshots and
 * interpolate between them; they never integrate. That is not a performance decision - it is the
 * only arrangement in which determinism (I7) means anything, because two clients running their
 * own physics would disagree the moment either dropped a frame.
 */
public final class KineticsService {

    private final Constants constants;
    private final Integrator integrator;
    private final Map<ResourceKey<Level>, DimensionContext> dimensions = new LinkedHashMap<>();
    private final Map<String, Handle> bodies = new LinkedHashMap<>();
    private final OrbitalRegistry registry;
    private final List<EventSink> listeners = new ArrayList<>();
    private final long worldSeed;

    private double worldTimeSeconds;

    public KineticsService(Constants constants, long worldSeed) {
        this.constants = constants;
        this.worldSeed = worldSeed;
        this.integrator = new Integrator(constants);
        this.registry = new OrbitalRegistry(constants);
    }

    /** Per-dimension environment and world probe. */
    private record DimensionContext(Environment environment, MinecraftWorldProbe probe) {}

    /** A live body under simulation. */
    public record Handle(KineticBody body, FlightDirector director,
                         ResourceKey<Level> dimension) {

        public String id() { return body.id(); }

        public BodyState state() { return body.snapshot(); }

        public FlightPhase phase() { return body.phase(); }
    }

    // ---- registration -----------------------------------------------------

    /**
     * Register a dimension. Vacuum and gravity are chosen from the dimension key, so the Moon
     * (Phase B) is airless and 0.165 g without any code downstream knowing it is the Moon.
     */
    public void registerDimension(ServerLevel level, boolean hasAtmosphere,
                                  double gravityScalar, boolean windEnabled) {
        MinecraftWorldProbe probe = new MinecraftWorldProbe(level);
        Environment env = new Environment(constants,
                hasAtmosphere ? Atmosphere.standard(constants) : Atmosphere.vacuum(constants),
                windEnabled ? WindField.seeded(constants, worldSeed)
                        : WindField.disabled(constants),
                probe, gravityScalar);
        dimensions.put(level.dimension(), new DimensionContext(env, probe));
    }

    /** The standard overworld: 1 g, standard air, wind off. */
    public void registerOverworld(ServerLevel level) {
        registerDimension(level, true,
                constants.d("gravity.dimension_scalars.overworld"), false);
    }

    public Environment environmentOf(ResourceKey<Level> dimension) {
        DimensionContext ctx = dimensions.get(dimension);
        return ctx == null ? null : ctx.environment();
    }

    /** Subscribe to kinetic events. Consumers resolve damage from these; kinetics never does. */
    public void addListener(EventSink listener) { listeners.add(listener); }

    // ---- spawning ---------------------------------------------------------

    /**
     * Put a body into the world.
     *
     * @param id       unique identifier, namespaced by the consumer
     * @param mission  what it is trying to do; selects the guidance chain, not the physics
     * @return the handle, or null if the dimension is not registered
     */
    public Handle spawn(String id, Profile profile, ResourceKey<Level> dimension,
                        Vec3 position, Vec3 velocity, FlightDirector.Mission mission) {
        DimensionContext ctx = dimensions.get(dimension);
        if (ctx == null) return null;

        // A lander is powered but is not on a rail: it arrives already falling, and putting it
        // through the liftoff T/W gate would fail a vehicle that never intended to lift off.
        FlightPhase initial = mission == FlightDirector.Mission.LANDING ? FlightPhase.DESCENT
                : profile.isPowered() ? FlightPhase.RAIL : FlightPhase.DESCENT;

        // Initial attitude. A body with velocity points along it; a body without one points
        // UP if it is on a rail and along +Z otherwise.
        //
        // The "up" case is not cosmetic. Quat.IDENTITY points +Z - horizontal - so a rocket
        // spawned at rest on a pad started lying on its side, spent seven seconds slewing
        // upright at its 12 deg/s rate while thrusting sideways, and flew into the ground. A
        // vehicle on a launch rail points at the sky; that is what a launch rail is for.
        Quat facing;
        if (velocity.lengthSq() > 1e-12) {
            facing = Quat.between(new Vec3(0, 0, 1), velocity.normalized());
        } else if (initial == FlightPhase.RAIL) {
            facing = Quat.between(new Vec3(0, 0, 1), Vec3.UP);
        } else {
            facing = Quat.IDENTITY;
        }

        KineticBody body = new KineticBody(id, profile, constants, position, velocity,
                facing, initial);
        FlightDirector director = new FlightDirector(constants, ctx.environment(), body,
                mission, integrator, worldSeed);

        Handle handle = new Handle(body, director, dimension);
        bodies.put(id, handle);
        return handle;
    }

    public Handle handle(String id) { return bodies.get(id); }

    public boolean despawn(String id) { return bodies.remove(id) != null; }

    public int liveBodies() { return bodies.size(); }

    public OrbitalRegistry orbits() { return registry; }

    public Constants constants() { return constants; }

    public double worldTimeSeconds() { return worldTimeSeconds; }

    // ---- the tick ---------------------------------------------------------

    /**
     * Advance every body and the registry by one server tick.
     *
     * @param targets  supplies the current target for a body, or null if it has none
     * @param decoys   supplies active countermeasures near a body, or null
     */
    public void tick(MinecraftServer server,
                     Function<String, Target> targets,
                     Function<String, List<Countermeasures.Decoy>> decoys) {
        double dt = constants.d("world.tick_seconds");
        EventSink sink = this::publish;

        List<String> finished = null;
        for (Handle handle : List.copyOf(bodies.values())) {
            if (!handle.body().phase().isInWorld()) {
                if (finished == null) finished = new ArrayList<>();
                finished.add(handle.id());
                continue;
            }
            handle.director().tick(worldTimeSeconds, dt,
                    targets == null ? null : targets.apply(handle.id()),
                    decoys == null ? null : decoys.apply(handle.id()),
                    sink);
        }
        if (finished != null) finished.forEach(bodies::remove);

        registry.advanceAttitudes(dt);
        registry.advanceDecay(worldTimeSeconds, sink);

        // Block lookups are cached within a tick and dropped at the boundary, so the probe can
        // never serve an answer from before a block changed.
        for (DimensionContext ctx : dimensions.values()) ctx.probe().endTick();

        worldTimeSeconds += dt;
    }

    private void publish(dev.lilkuzco.kinetics.event.KineticEvent event) {
        for (EventSink listener : listeners) listener.accept(event);
    }

    /** Snapshots for every live body, for the client sync packet. */
    public List<BodyState> snapshots() {
        List<BodyState> out = new ArrayList<>(bodies.size());
        for (Handle handle : bodies.values()) out.add(handle.state());
        return out;
    }
}
