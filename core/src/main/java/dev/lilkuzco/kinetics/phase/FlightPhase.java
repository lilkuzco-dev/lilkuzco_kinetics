package dev.lilkuzco.kinetics.phase;

import java.util.EnumSet;
import java.util.Map;

/**
 * The flight-phase state machine (Section 2).
 *
 * <p>Transitions are enumerated, not inferred. A consumer cannot invent a transition, and the
 * sim cannot slide from BOOST into TERMINAL because some condition happened to be true - the
 * move has to be in {@link #LEGAL} or {@link PhaseMachine} rejects it. That matters because
 * phases gate physics: thrust only exists in BOOST, guidance only steers in MIDCOURSE and
 * TERMINAL, and heating is only tracked in REENTRY. A silent illegal transition would produce
 * a body that is coasting and burning fuel at the same time, which I3 and I4 would then catch
 * far downstream of the actual mistake.
 */
public enum FlightPhase {

    /** On the pad or rail. No thrust yet; the T/W gate is evaluated on the way out (RD5). */
    RAIL,

    /** Thrusting. Gravity turn (RD6), limited steering, max-Q watched (RB6). */
    BOOST,

    /** Between stages: one burned out, the next has not lit (RD4). Momentary. */
    STAGING,

    /** Powered flight over; guidance steers a coasting body (RC1-RC6). Energy only falls (I3). */
    MIDCOURSE,

    /** Endgame: seeker envelope active, proximity fuse armed. */
    TERMINAL,

    /** Drag-dominated fall (RB2-RB3). Chute deploy gates evaluated here (RB6). */
    DESCENT,

    /** High-energy entry: heating field tracked (RE7). Eases into DESCENT as speed bleeds off. */
    REENTRY,

    /** In the orbital registry (RE1-RE7). Advanced from epoch, not ticked. */
    ORBIT,

    /** On the ground and at rest. Terminal state. */
    LANDED,

    /** Destroyed, impacted or otherwise finished. Terminal state. */
    TERMINATED,

    /**
     * Powered descent: the retro-burn that puts a vehicle on the ground intact (RD6).
     *
     * <p>A separate phase rather than a thrusting DESCENT, because the two are opposite
     * situations. DESCENT is a body giving its energy to the air; LANDING is a body spending
     * propellant to shed energy the air will not take. On an airless world there IS no drag, so
     * arriving intact is a rocketry problem with a fuel bill - and giving it its own phase means
     * the bill shows up in the phase log rather than hiding inside a fall.
     *
     * <p><b>Declared last on purpose, out of the flight's natural order.</b> Trajectory states are
     * hashed with the phase's ORDINAL in them, so inserting a phase mid-enum renumbers every phase
     * after it and invalidates every committed golden hash - not because any trajectory changed,
     * but because the numbering did. Appending keeps the goldens meaningful: they still fail if
     * and only if the physics moved, which is the only thing they are for.
     */
    LANDING;

    /**
     * Whether the phase permits thrust.
     *
     * <p>BOOST obviously, and TERMINAL too - a short-burn interceptor routinely reaches its
     * target while the motor is still lit. Making TERMINAL unpowered would force the endgame to
     * wait for burnout, and a missile whose boost lasts 1.2 s and whose intercept happens at
     * 1.25 s would sail straight past its target with the fuse still safed. That is not a
     * hypothetical: it is what the shipped interceptor profile does.
     */
    public boolean isPowered() {
        return this == BOOST || this == TERMINAL || this == LANDING;
    }

    /** Whether guidance may command acceleration in this phase. */
    public boolean isGuided() {
        return this == BOOST || this == MIDCOURSE || this == TERMINAL || this == LANDING;
    }

    /** Whether the body is still being integrated in the world. */
    public boolean isInWorld() {
        return this != ORBIT && this != LANDED && this != TERMINATED;
    }

    public boolean isTerminal() { return this == LANDED || this == TERMINATED; }

    /**
     * Legal successor phases. Written out one line per source phase so the whole machine is
     * readable at a glance rather than assembled from scattered conditions.
     */
    public static final Map<FlightPhase, EnumSet<FlightPhase>> LEGAL = Map.ofEntries(
            // Ignition, or a failure to leave the pad at all.
            Map.entry(RAIL, EnumSet.of(BOOST, TERMINATED)),
            // Burnout with stages left, burnout of the last stage, an abort, a direct
            // insertion, destruction by a structural limit - or arriving at the target while
            // still under power, which a short-burn interceptor does routinely.
            Map.entry(BOOST, EnumSet.of(STAGING, MIDCOURSE, TERMINAL, DESCENT, ORBIT, TERMINATED)),
            // The next stage lights, or nothing does.
            Map.entry(STAGING, EnumSet.of(BOOST, MIDCOURSE, TERMINATED)),
            // Seeker takes over, the body starts falling, or it makes orbit.
            Map.entry(MIDCOURSE, EnumSet.of(TERMINAL, DESCENT, ORBIT, TERMINATED)),
            // Terminal ends in an impact one way or another.
            Map.entry(TERMINAL, EnumSet.of(DESCENT, TERMINATED)),
            // A fall ends on the ground, turns out to be hot enough to be a reentry, or - on a
            // vehicle with propellant and somewhere to put it - becomes a retro-burn.
            Map.entry(DESCENT, EnumSet.of(REENTRY, LANDING, LANDED, TERMINATED)),
            // The retro-burn lands it, runs dry and hands back to an unpowered fall, or fails.
            Map.entry(LANDING, EnumSet.of(DESCENT, LANDED, TERMINATED)),
            // Heating peaks and eases back into an ordinary descent.
            Map.entry(REENTRY, EnumSet.of(DESCENT, LANDED, TERMINATED)),
            // Orbits decay or are commanded down; either way the body re-enters the world.
            Map.entry(ORBIT, EnumSet.of(DESCENT, REENTRY, TERMINATED)),
            Map.entry(LANDED, EnumSet.of(TERMINATED)),
            Map.entry(TERMINATED, EnumSet.noneOf(FlightPhase.class)));

    public boolean canTransitionTo(FlightPhase next) {
        return LEGAL.getOrDefault(this, EnumSet.noneOf(FlightPhase.class)).contains(next);
    }
}
