package dev.lilkuzco.kinetics.event;

import java.util.ArrayList;
import java.util.List;

/**
 * Where events go. Consumers implement this; kinetics only calls it.
 *
 * <p>Ordering is guaranteed: events are delivered in the order the simulation produced them,
 * within a tick and across ticks. Determinism (I7) depends on that, and so does anything a
 * consumer does with them - a staging event that arrived after the impact it preceded would
 * make a replay diverge from the original.
 */
@FunctionalInterface
public interface EventSink {

    void accept(KineticEvent event);

    /** Discard everything. The default for closed-form tests that only check numbers. */
    static EventSink discarding() {
        return event -> { };
    }

    /** Collects events in order. The test battery reads assertions off this. */
    final class Recording implements EventSink {
        private final List<KineticEvent> events = new ArrayList<>();

        @Override
        public void accept(KineticEvent event) { events.add(event); }

        public List<KineticEvent> events() { return List.copyOf(events); }

        public void clear() { events.clear(); }

        public int count() { return events.size(); }

        @SuppressWarnings("unchecked")
        public <T extends KineticEvent> List<T> ofType(Class<T> type) {
            List<T> out = new ArrayList<>();
            for (KineticEvent e : events) if (type.isInstance(e)) out.add((T) e);
            return out;
        }

        public <T extends KineticEvent> boolean has(Class<T> type) {
            for (KineticEvent e : events) if (type.isInstance(e)) return true;
            return false;
        }

        public <T extends KineticEvent> T first(Class<T> type) {
            List<T> matches = ofType(type);
            return matches.isEmpty() ? null : matches.get(0);
        }
    }

    /** Deliver to two sinks. Lets the invariant checker observe without displacing a consumer. */
    static EventSink tee(EventSink a, EventSink b) {
        return event -> { a.accept(event); b.accept(event); };
    }
}
