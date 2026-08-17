package dev.lilkuzco.kinetics.propulsion;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Powered descent guidance: the retro-burn that lands a vehicle instead of cratering it (RD6).
 *
 * <p>On a world with air, arriving is free - drag and a canopy do the work, and the vehicle needs
 * no propellant at all to survive the ground. In vacuum there is nothing to push against, so
 * <b>every metre per second of arrival speed has to be cancelled by the engine</b>, and the fuel
 * bill is the whole difficulty of landing on the Moon.
 *
 * <p>The law is a constant-thrust suicide burn, which is what a real lander flies and what the
 * closed form can be checked against. Under constant net deceleration {@code a = F/m - g}, a body
 * at speed {@code v} needs
 *
 * <pre>  h_burn = v^2 / (2a)</pre>
 *
 * <p>of altitude to stop. Above that height the engine stays cold; at or below it the engine goes
 * to full throttle pointed retrograde, and the vehicle arrives at the surface with its velocity
 * spent. Burning any earlier wastes propellant hovering - which is precisely the "gravity losses"
 * term, and it is why a lander does not simply thrust the whole way down.
 *
 * <p>Two things keep it honest against an integrator rather than a whiteboard:
 *
 * <ul>
 *   <li><b>A trigger margin.</b> The ideal burn has zero slack: one tick late and the vehicle is
 *       already below the height it needed. The margin buys back exactly that discretisation
 *       error, at the cost of a little hover.</li>
 *   <li><b>Mass is read every tick.</b> A lander gets lighter as it burns, so its available
 *       deceleration climbs during the burn. Freezing {@code a} at ignition would size the burn
 *       for a vehicle that no longer exists.</li>
 * </ul>
 *
 * <p>This class decides <em>when</em> and <em>which way</em>. It never applies a force, never
 * touches mass and never decides whether the landing was survivable - the integrator, the mass
 * ledger and the consumer do those, in that order.
 */
public final class PoweredDescent {

    private final double triggerMargin;
    private final double touchdownSpeed;
    private final double minNetDeceleration;

    public PoweredDescent(Constants k) {
        this.triggerMargin = k.d("landing.burn_trigger_margin");
        this.touchdownSpeed = k.d("landing.touchdown_speed");
        this.minNetDeceleration = k.d("landing.min_net_deceleration");
    }

    /** What the guidance wants this tick. */
    public record Command(boolean burn, Vec3 direction, double throttle, double burnAltitude) {}

    /**
     * Decide the descent command.
     *
     * @param altitude    height above the surface, m
     * @param velocity    current velocity, m/s
     * @param thrust      available thrust at this altitude, N
     * @param mass        current mass, kg
     * @param gravity     local gravitational acceleration, m/s^2
     */
    public Command decide(double altitude, Vec3 velocity, double thrust, double mass,
                          double gravity) {
        double speed = velocity.length();

        // Retrograde. Straight up for a vertical fall, and correctly tilted for one that is not -
        // a lander arriving with horizontal velocity has to cancel that too, and pointing at the
        // sky instead of at the velocity would leave it flying sideways into a crater wall.
        Vec3 retrograde = speed > 1e-9 ? velocity.normalized().scale(-1.0) : Vec3.UP;

        if (mass <= 0.0 || thrust <= 0.0) {
            return new Command(false, retrograde, 0.0, Double.POSITIVE_INFINITY);
        }

        double net = thrust / mass - gravity;
        if (net < minNetDeceleration) {
            // Thrust cannot beat gravity: nothing this vehicle does will stop it. Report the burn
            // as impossible rather than firing pointlessly and arriving out of fuel AND fast.
            return new Command(false, retrograde, 0.0, Double.POSITIVE_INFINITY);
        }

        // Already slow enough to touch down. Stop burning: hovering is how a lander runs out of
        // propellant three metres above the ground.
        if (speed <= touchdownSpeed) {
            return new Command(false, retrograde, 0.0, 0.0);
        }

        double burnAltitude = (speed * speed) / (2.0 * net) * triggerMargin;
        boolean burn = altitude <= burnAltitude;
        return new Command(burn, retrograde, burn ? 1.0 : 0.0, burnAltitude);
    }

    /**
     * The ideal delta-v to arrive at rest from a given speed under a given gravity, m/s.
     *
     * <p>Not simply the arrival speed: while the engine burns, gravity keeps adding velocity, and
     * the vehicle must cancel that too. Over a burn of duration {@code t = v/a_net} the extra is
     * {@code g*t}, so the true cost is {@code v * (1 + g/a_net)} - the gravity loss, made explicit
     * rather than absorbed into a fudged budget.
     */
    public double idealDeltaV(double arrivalSpeed, double thrust, double mass, double gravity) {
        if (mass <= 0.0 || thrust <= 0.0) return Double.POSITIVE_INFINITY;
        double net = thrust / mass - gravity;
        if (net < minNetDeceleration) return Double.POSITIVE_INFINITY;
        return arrivalSpeed * (1.0 + gravity / net);
    }

    public double touchdownSpeed() { return touchdownSpeed; }
}
