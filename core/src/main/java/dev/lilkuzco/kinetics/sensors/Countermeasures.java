package dev.lilkuzco.kinetics.sensors;

import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.guidance.Seeker;
import dev.lilkuzco.kinetics.guidance.Target;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.profile.SeekerSpec;
import dev.lilkuzco.kinetics.util.Rng;

import java.util.List;

/**
 * Decoys, chaff and flares (RF5).
 *
 * <p>The design rule that makes this honest: <b>a decoy presents a legitimate signature.</b> It
 * does not set a "confuse the missile" flag - it enters the seeker's candidate list as a
 * contact with a real cross-section at a real position, and the seeker scores it against the
 * true target using the same rule it uses for everything else. If the decoy scores higher, the
 * seeker tracks the decoy, and it does so believing it is right.
 *
 * <p>The corollary, which RF5 states explicitly, is the thing worth guarding: once seduced, the
 * seeker must not silently drift back onto the true target. A seduced seeker is locked onto the
 * decoy; when the decoy burns out it has lost lock and goes to memory-track like any other lost
 * lock. There is no code path that returns it to the truth for free, because that path is what
 * would make countermeasures decorative.
 */
public final class Countermeasures {

    private Countermeasures() {}

    /** A deployed decoy. */
    public record Decoy(
            String id,
            Vec3 position,
            Vec3 velocity,
            double signature,
            Kind kind,
            double expiresAtAge) {

        public enum Kind {
            /** Infrared decoy: seduces heat-seeking heads. */
            FLARE,
            /** Radar-reflective cloud: degrades RF lock rather than presenting a point target. */
            CHAFF,
            /** A towed or launched decoy presenting a full airframe-like return (Sarab). */
            TOWED
        }

        public Target asTarget() {
            return new Target(id, position, velocity, Vec3.ZERO, signature, true);
        }

        public boolean isActive(double age) { return age < expiresAtAge; }
    }

    /**
     * Offer a seeker a set of decoys and let it score them.
     *
     * <p>Scoring is signature over squared range - a bright contact nearby beats a dim one far
     * away, which is exactly why a flare works: it is far brighter than the aircraft, for a few
     * seconds, from close by.
     *
     * @param resistanceRoll deterministic roll against the seeker's resistance; a good head
     *                       rejects most seductions, a cheap one does not
     * @return true if the seeker was seduced or its lock was broken
     */
    public static boolean attemptSeduction(Seeker seeker, SeekerSpec spec, Vec3 seekerPosition,
                                           Target trueTarget, List<Decoy> decoys,
                                           double bodyAge, long seed, EventSink events) {
        if (decoys.isEmpty() || !spec.isPresent()) return false;

        Decoy best = null;
        double bestScore = trueTarget == null ? 0.0
                : score(seekerPosition, trueTarget.position(), trueTarget.signature());

        for (Decoy decoy : decoys) {
            if (!decoy.isActive(bodyAge)) continue;
            double s = score(seekerPosition, decoy.position(), decoy.signature());
            if (s > bestScore) { bestScore = s; best = decoy; }
        }
        if (best == null) return false;

        double resistance = best.kind() == Decoy.Kind.FLARE
                ? spec.flareResistance() : spec.chaffResistance();
        // One roll per (seed, decoy, seeker age) - deterministic, so a replay of this
        // engagement produces the same outcome (I7).
        Rng rng = Rng.forPurpose(seed, "countermeasure:" + best.id() + ":"
                + Double.doubleToRawLongBits(bodyAge));
        if (rng.chance(resistance)) return false;   // the head saw through it

        if (best.kind() == Decoy.Kind.CHAFF) {
            // Chaff is a cloud, not a point return. It does not give the seeker something else
            // to chase; it takes away what it was chasing.
            seeker.breakLock("chaff", bodyAge, events);
        } else {
            seeker.seduceOnto(best.asTarget(), best.kind().name().toLowerCase(java.util.Locale.ROOT),
                    bodyAge, events);
        }
        return true;
    }

    /** Signature strength as seen from a position: brightness over squared range. */
    public static double score(Vec3 from, Vec3 contact, double signature) {
        double rangeSq = contact.sub(from).lengthSq();
        if (rangeSq < 1e-9) return Double.MAX_VALUE;
        return signature / rangeSq;
    }

    /** A flare: very bright, very brief. */
    public static Decoy flare(String id, Vec3 position, Vec3 velocity, double brightness,
                              double currentAge, double burnSeconds) {
        return new Decoy(id, position, velocity, brightness, Decoy.Kind.FLARE,
                currentAge + burnSeconds);
    }

    /** A chaff burst: degrades RF lock, presents no coherent target. */
    public static Decoy chaff(String id, Vec3 position, double reflectivity,
                              double currentAge, double bloomSeconds) {
        return new Decoy(id, position, Vec3.ZERO, reflectivity, Decoy.Kind.CHAFF,
                currentAge + bloomSeconds);
    }

    /** A towed decoy: an airframe-like return that keeps station with the aircraft. */
    public static Decoy towed(String id, Vec3 position, Vec3 velocity, double signature,
                              double currentAge, double lifetime) {
        return new Decoy(id, position, velocity, signature, Decoy.Kind.TOWED,
                currentAge + lifetime);
    }
}
