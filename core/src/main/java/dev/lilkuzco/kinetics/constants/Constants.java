package dev.lilkuzco.kinetics.constants;

import dev.lilkuzco.kinetics.util.Json;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * The one door to every physical constant (invariant I9).
 *
 * <p>Code in this library contains no physics literals. Anything with a unit attached is
 * looked up here by dotted path, and a missing or malformed key is a hard failure at load
 * rather than a silent default - a physics library that quietly substitutes 9.8 for a
 * missing gravity constant is worse than one that refuses to start.
 *
 * <p>Numerical epsilons and array sizes are not physics and stay in code. The test is
 * whether the number would appear in a physics textbook with a unit beside it.
 */
public final class Constants {

    private static final String RESOURCE = "/physics-constants.json";

    private final Map<String, Object> root;
    private final List<ScaledConstant> scaled = new ArrayList<>();

    private static volatile Constants instance;

    private Constants(Map<String, Object> root) {
        this.root = root;
        collectScaled("", root);
    }

    /** Process-wide instance, loaded from the packaged resource on first use. */
    public static Constants get() {
        Constants local = instance;
        if (local == null) {
            synchronized (Constants.class) {
                local = instance;
                if (local == null) {
                    local = loadFromResource();
                    instance = local;
                }
            }
        }
        return local;
    }

    private static Constants loadFromResource() {
        try (InputStream in = Constants.class.getResourceAsStream(RESOURCE)) {
            if (in == null) {
                throw new ConstantsException(
                        "physics-constants.json is not on the classpath at " + RESOURCE
                        + ". kinetics cannot run without it (I9) - this is a packaging fault.");
            }
            String text = new String(in.readAllBytes(), StandardCharsets.UTF_8);
            return new Constants(Json.parseObject(text));
        } catch (IOException e) {
            throw new ConstantsException("could not read " + RESOURCE + ": " + e.getMessage());
        }
    }

    /** Load from explicit text. Used by tests and by hot-reload. */
    public static Constants fromText(String json) {
        return new Constants(Json.parseObject(json));
    }

    /**
     * Value at a dotted path, e.g. {@code "atmosphere.rho_sea_level"}. The path addresses the
     * constant's wrapper object; this returns its {@code value} field.
     */
    public double d(String path) {
        Object node = resolve(path);
        if (!(node instanceof Map<?, ?> m)) {
            throw new ConstantsException("constant '" + path + "' is not a constant object");
        }
        Object v = m.get("value");
        if (!(v instanceof Double dv)) {
            throw new ConstantsException("constant '" + path + "' has no numeric 'value'");
        }
        return dv;
    }

    public int i(String path) {
        double v = d(path);
        if (v != Math.floor(v)) {
            throw new ConstantsException("constant '" + path + "' is not an integer: " + v);
        }
        return (int) v;
    }

    public boolean bool(String path) { return d(path) != 0.0; }

    /** The {@code units} string, for the scale audit and error messages. */
    public String units(String path) {
        Object node = resolve(path);
        if (node instanceof Map<?, ?> m && m.get("units") instanceof String s) return s;
        return "";
    }

    public String sourceNote(String path) {
        Object node = resolve(path);
        if (node instanceof Map<?, ?> m && m.get("source_note") instanceof String s) return s;
        return "";
    }

    public boolean has(String path) {
        try { resolve(path); return true; } catch (ConstantsException e) { return false; }
    }

    private Object resolve(String path) {
        Object node = root;
        StringBuilder walked = new StringBuilder();
        for (String part : path.split("\\.")) {
            if (!walked.isEmpty()) walked.append('.');
            walked.append(part);
            if (!(node instanceof Map<?, ?> m)) {
                throw new ConstantsException("constant path '" + path
                        + "' runs through non-object '" + walked + "'");
            }
            node = m.get(part);
            if (node == null) {
                throw new ConstantsException("constant '" + path + "' is missing from "
                        + "physics-constants.json (failed at '" + walked + "'). "
                        + "Every physics value must be declared there (I9).");
            }
        }
        return node;
    }

    // ---- I11: scale audit -------------------------------------------------

    /**
     * A constant whose game value differs from the real-world measurement, with the factor
     * between them. Invariant I11 requires the full list to be inspectable.
     */
    public record ScaledConstant(
            String path, double gameValue, double realValue, double scaleFactor,
            String units, String sourceNote) {

        /**
         * Whether {@code gameValue * scaleFactor} actually reproduces {@code realValue}. A
         * declared factor that does not reconstruct its own real value is a bookkeeping bug,
         * and the audit reports it rather than trusting the declaration.
         */
        public boolean factorIsConsistent() {
            if (realValue == 0.0) return gameValue == 0.0;
            double reconstructed = gameValue * scaleFactor;
            double relative = Math.abs(reconstructed - realValue) / Math.abs(realValue);
            return relative < 1e-3;
        }
    }

    private void collectScaled(String prefix, Map<?, ?> node) {
        for (Map.Entry<?, ?> e : node.entrySet()) {
            String key = String.valueOf(e.getKey());
            if (key.startsWith("_")) continue;
            String path = prefix.isEmpty() ? key : prefix + "." + key;
            if (!(e.getValue() instanceof Map<?, ?> m)) continue;
            Object real = m.get("real_value");
            Object factor = m.get("scale_factor");
            Object value = m.get("value");
            if (real instanceof Double rv && factor instanceof Double sf && value instanceof Double gv) {
                scaled.add(new ScaledConstant(path, gv, rv, sf,
                        m.get("units") instanceof String u ? u : "",
                        m.get("source_note") instanceof String s ? s : ""));
            }
            // Recurse whether or not this node was itself a constant, so nested groups
            // (gravity.dimension_scalars.moon) are reached.
            collectScaled(path, m);
        }
    }

    /** Every scaled constant, in document order (I11). */
    public List<ScaledConstant> scaledConstants() { return List.copyOf(scaled); }

    public static final class ConstantsException extends RuntimeException {
        public ConstantsException(String message) { super(message); }
    }
}
