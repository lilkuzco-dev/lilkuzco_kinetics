package dev.lilkuzco.kinetics.phase;

import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;

import java.util.ArrayList;
import java.util.List;

/**
 * Holds the current phase and refuses illegal moves.
 *
 * <p>Every transition is logged with a reason. When a golden trajectory diverges, the phase
 * log is usually where the divergence is visible first - a stage that lit one tick earlier
 * shows up here long before it shows up in a position hash.
 */
public final class PhaseMachine {

    private final String bodyId;
    private final List<Entry> log = new ArrayList<>();
    private FlightPhase phase;

    public PhaseMachine(String bodyId, FlightPhase initial) {
        this.bodyId = bodyId;
        this.phase = initial;
        log.add(new Entry(0.0, null, initial, "created"));
    }

    public FlightPhase phase() { return phase; }

    /**
     * Attempt a transition.
     *
     * @return true if it happened; false if it was already in that phase
     * @throws IllegalPhaseTransition if the move is not in {@link FlightPhase#LEGAL}
     */
    public boolean transition(FlightPhase next, double bodyAge, String reason, EventSink events) {
        if (next == phase) return false;
        if (!phase.canTransitionTo(next)) {
            throw new IllegalPhaseTransition(
                    "body '" + bodyId + "' attempted " + phase + " -> " + next + " at t="
                    + String.format("%.3f", bodyAge) + "s (reason: " + reason + "), which is not a "
                    + "legal transition. Legal from " + phase + ": " + FlightPhase.LEGAL.get(phase)
                    + ". Consumers cannot invent transitions - if this move should exist, it "
                    + "belongs in FlightPhase.LEGAL.");
        }
        FlightPhase from = phase;
        phase = next;
        log.add(new Entry(bodyAge, from, next, reason));
        events.accept(new KineticEvent.PhaseChange(bodyId, bodyAge, from.name(), next.name(), reason));
        return true;
    }

    /** Full transition history, in order. */
    public List<Entry> log() { return List.copyOf(log); }

    /** A one-line-per-transition rendering, for test failure output. */
    public String renderLog() {
        StringBuilder sb = new StringBuilder();
        for (Entry e : log) {
            sb.append(String.format("  t=%8.3f  %-10s -> %-10s  %s%n",
                    e.atSeconds(), e.from() == null ? "-" : e.from().name(),
                    e.to().name(), e.reason()));
        }
        return sb.toString();
    }

    public record Entry(double atSeconds, FlightPhase from, FlightPhase to, String reason) {}

    public static final class IllegalPhaseTransition extends RuntimeException {
        public IllegalPhaseTransition(String message) { super(message); }
    }
}
