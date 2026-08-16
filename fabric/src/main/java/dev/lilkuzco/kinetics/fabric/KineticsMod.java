package dev.lilkuzco.kinetics.fabric;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.constants.ScaleAudit;
import dev.lilkuzco.kinetics.event.KineticEvent;
import net.fabricmc.api.ModInitializer;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerLifecycleEvents;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerTickEvents;
import net.minecraft.server.MinecraftServer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Mod entrypoint. Sets up one {@link KineticsService} per server and ticks it.
 *
 * <p>Kinetics registers no blocks, no items, no entities and no recipes. It is a library that
 * happens to be packaged as a mod, because that is how a Minecraft server loads shared code -
 * consumers depend on it and drive it. Nothing here is player-facing, which is exactly what
 * "no player content" in the campaign brief means.
 */
public final class KineticsMod implements ModInitializer {

    public static final String MOD_ID = "kinetics";
    private static final Logger LOG = LoggerFactory.getLogger("lilkuzco-kinetics");

    private static KineticsService service;

    /** The live service, or null before a world is loaded. */
    public static KineticsService service() { return service; }

    @Override
    public void onInitialize() {
        Constants constants = Constants.get();

        // Fail loudly at load if the constants file is inconsistent, rather than flying a
        // trajectory built on a factor that does not reconstruct its own real value (I11).
        var inconsistencies = new ScaleAudit(constants).inconsistencies();
        if (!inconsistencies.isEmpty()) {
            throw new IllegalStateException(
                    "physics-constants.json has scale factors that do not reconstruct their own "
                    + "real values: " + inconsistencies.stream()
                            .map(dev.lilkuzco.kinetics.constants.Constants.ScaledConstant::path)
                            .toList()
                    + ". Refusing to start on a broken scale mapping (I11).");
        }

        LOG.info("kinetics {}: g0={} m/s^2, planet R={} m, orbit dv={} m/s, {} scaled constants",
                "0.1.1",
                constants.d("gravity.g0"),
                constants.d("orbit.planet_radius"),
                constants.d("orbit.delta_v_to_orbit"),
                constants.scaledConstants().size());

        ServerLifecycleEvents.SERVER_STARTED.register(this::onServerStarted);
        ServerLifecycleEvents.SERVER_STOPPED.register(server -> service = null);
        ServerTickEvents.END_SERVER_TICK.register(this::onEndTick);
    }

    private void onServerStarted(MinecraftServer server) {
        long seed = server.overworld().getSeed();
        service = new KineticsService(Constants.get(), seed);
        service.registerOverworld(server.overworld());

        // Log invariant breaches at ERROR. They are P0 physics bugs, and a server operator
        // seeing one should be able to hand the line straight to a developer.
        service.addListener(event -> {
            if (event instanceof KineticEvent.InvariantBreach breach) {
                LOG.error("INVARIANT {} breached by body {} at t={}s: {}",
                        breach.invariant(), breach.bodyId(), breach.bodyAge(), breach.detail());
            }
        });

        LOG.info("kinetics service ready (world seed {}), overworld registered", seed);
    }

    private void onEndTick(MinecraftServer server) {
        KineticsService live = service;
        if (live == null || live.liveBodies() == 0 && live.orbits().size() == 0) return;
        // Targets and countermeasures are supplied by consumers; with none registered the
        // service still integrates ballistic bodies and advances orbits.
        live.tick(server, null, null);
    }
}
