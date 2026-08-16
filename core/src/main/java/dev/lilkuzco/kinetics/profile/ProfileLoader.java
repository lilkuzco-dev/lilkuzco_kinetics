package dev.lilkuzco.kinetics.profile;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.util.Json;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Parses and validates profile JSON. Hot-reloadable; errors are meant to be read by a human
 * who is editing a file, not by a developer reading a stack trace.
 *
 * <p>Validation is not politeness - it is where invariant I8 is actually enforced. The fuzz
 * harness throws deliberately absurd profiles at this loader (negative mass, zero area,
 * vacuum Isp below sea-level Isp) and every one must be rejected <em>here</em>, at load, with
 * an explanation. Anything that slips through becomes a NaN somewhere in the integrator three
 * seconds later, where the cause is unrecoverable.
 *
 * <p>The rejections worth naming:
 * <ul>
 *   <li><b>negative or zero mass</b> - divides through the whole force model</li>
 *   <li><b>zero reference area with a nonzero C_d</b> - a body that cannot feel the air</li>
 *   <li><b>Isp_vac &lt; Isp_sl</b> - thermodynamically backwards; a nozzle cannot do worse in
 *       vacuum than against ambient pressure, and a profile claiming so would make the RD2b
 *       interpolation run the wrong way</li>
 *   <li><b>g-limit above the hard cap</b> - clamped to 60 g with a warning rather than
 *       rejected, since it is a design overreach and not a corrupt file (I2)</li>
 * </ul>
 */
public final class ProfileLoader {

    private final Constants k;
    private final List<String> warnings = new ArrayList<>();

    public ProfileLoader(Constants k) { this.k = k; }

    public List<String> warnings() { return List.copyOf(warnings); }

    public Profile load(String json) {
        Map<String, Object> root = Json.parseObject(json);
        return fromMap(root);
    }

    /** Load a document containing {@code {"profiles": [...]}} or a single profile object. */
    @SuppressWarnings("unchecked")
    public List<Profile> loadAll(String json) {
        Map<String, Object> root = Json.parseObject(json);
        Object list = root.get("profiles");
        if (list == null) return List.of(fromMap(root));
        if (!(list instanceof List<?> l)) {
            throw fail("<document>", "'profiles' must be an array");
        }
        List<Profile> out = new ArrayList<>();
        for (Object o : l) {
            if (!(o instanceof Map)) throw fail("<document>", "each entry of 'profiles' must be an object");
            out.add(fromMap((Map<String, Object>) o));
        }
        return out;
    }

    @SuppressWarnings("unchecked")
    private Profile fromMap(Map<String, Object> m) {
        String id = str(m, "id", "<unnamed>");
        if (id.isBlank()) throw fail(id, "'id' must not be blank");

        double payloadDry = req(m, "payload_dry_mass", id);
        if (payloadDry <= 0.0) {
            throw fail(id, "payload_dry_mass must be greater than zero (got " + payloadDry
                    + "). Mass divides through every force in the model, so a zero or negative "
                    + "value has no physical meaning.");
        }

        List<Stage> stages = new ArrayList<>();
        Object rawStages = m.get("stages");
        if (rawStages instanceof List<?> sl) {
            for (int i = 0; i < sl.size(); i++) {
                if (!(sl.get(i) instanceof Map)) {
                    throw fail(id, "stages[" + i + "] must be an object");
                }
                stages.add(stage(id, i, (Map<String, Object>) sl.get(i)));
            }
        }
        if (stages.size() > 3) {
            throw fail(id, "profiles support 1-3 stages; this declares " + stages.size()
                    + " (RD4). More stages than that is a design the launch pipeline was not "
                    + "built to fly.");
        }

        Airframe airframe = airframe(id, mapOf(m, "airframe", id));
        Recovery recovery = recovery(id, m.get("recovery"));
        SeekerSpec seeker = seeker(id, m.get("seeker"));

        int substeps = (int) opt(m, "substeps", 0.0);
        int maxSubsteps = k.i("limits.max_substeps");
        if (substeps < 0 || substeps > maxSubsteps) {
            throw fail(id, "substeps must be between 0 (auto) and " + maxSubsteps
                    + ", got " + substeps);
        }
        double slew = opt(m, "max_slew_rate_deg", k.d("limits.max_slew_rate_default"));
        if (slew <= 0.0) throw fail(id, "max_slew_rate_deg must be positive, got " + slew);

        double cep = opt(m, "cep", k.d("dispersion.default_cep"));
        if (cep < 0.0) throw fail(id, "cep must not be negative, got " + cep);

        return new Profile(id, payloadDry, stages, airframe, recovery, seeker,
                substeps, slew, cep);
    }

    private Stage stage(String id, int index, Map<String, Object> m) {
        String where = "stages[" + index + "]";
        String engine = str(m, "engine", "stage" + index);
        double fuel = req(m, "fuel_mass", id, where);
        double dry = req(m, "stage_dry_mass", id, where);
        double thrust = req(m, "thrust_vacuum", id, where);
        double ispSl = req(m, "isp_sea_level", id, where);
        double ispVac = req(m, "isp_vacuum", id, where);

        if (fuel < 0.0) throw fail(id, where + ".fuel_mass must not be negative, got " + fuel);
        if (dry < 0.0) throw fail(id, where + ".stage_dry_mass must not be negative, got " + dry);
        if (thrust <= 0.0) {
            throw fail(id, where + ".thrust_vacuum must be positive, got " + thrust
                    + ". A stage with no thrust is not a stage - remove it or give it an engine.");
        }
        if (ispSl <= 0.0 || ispVac <= 0.0) {
            throw fail(id, where + " specific impulse must be positive (sea level " + ispSl
                    + ", vacuum " + ispVac + "). Isp appears in the denominator of the mass-flow "
                    + "derivation, so zero would divide by zero.");
        }
        if (ispVac < ispSl) {
            throw fail(id, where + " declares vacuum Isp (" + ispVac
                    + " s) below sea-level Isp (" + ispSl + " s), which is thermodynamically "
                    + "backwards - a nozzle performs better in vacuum than against ambient "
                    + "pressure, never worse. Check the two values are not swapped (RD2b).");
        }

        double ionIsp = k.d("propulsion.ion_isp_threshold");
        double ionThrust = k.d("propulsion.ion_launch_thrust_flag_newtons");
        if (ispVac >= ionIsp && thrust < ionThrust && index == 0) {
            warnings.add(id + " " + where + ": engine '" + engine + "' has electric-propulsion "
                    + "Isp (" + ispVac + " s) but only " + thrust + " N of thrust, and it is the "
                    + "first stage. Ion drives cannot lift off - this vehicle will fail the T/W "
                    + "gate (RD2/RD5).");
        }
        return new Stage(engine, fuel, dry, thrust, ispSl, ispVac);
    }

    private Airframe airframe(String id, Map<String, Object> m) {
        double refArea = req(m, "reference_area", id, "airframe");
        if (refArea <= 0.0) {
            throw fail(id, "airframe.reference_area must be positive, got " + refArea
                    + ". A body with no reference area cannot exchange momentum with the air, "
                    + "which is not a physical object.");
        }
        double cd0 = req(m, "cd0", id, "airframe");
        if (cd0 < 0.0) {
            throw fail(id, "airframe.cd0 must not be negative, got " + cd0
                    + ". Negative drag is thrust, and thrust comes from stages.");
        }
        double wingArea = opt(m, "wing_area", 0.0);
        if (wingArea < 0.0) throw fail(id, "airframe.wing_area must not be negative");

        double ar = opt(m, "aspect_ratio", Double.POSITIVE_INFINITY);
        if (ar <= 0.0) {
            throw fail(id, "airframe.aspect_ratio must be positive (omit it for a body with no "
                    + "lifting surface), got " + ar);
        }
        double oswald = opt(m, "oswald_efficiency", k.d("aerodynamics.oswald_efficiency_default"));
        if (oswald <= 0.0 || oswald > 1.0) {
            throw fail(id, "airframe.oswald_efficiency must be in (0,1], got " + oswald);
        }

        double gLimit = opt(m, "g_limit", k.d("limits.g_limit_default"));
        double hardCap = k.d("limits.g_limit_hard_cap");
        if (gLimit <= 0.0) throw fail(id, "airframe.g_limit must be positive, got " + gLimit);
        if (gLimit > hardCap) {
            warnings.add(id + " airframe.g_limit of " + gLimit + " g exceeds the hard cap of "
                    + hardCap + " g and has been clamped (I2).");
            gLimit = hardCap;
        }

        double qMax = opt(m, "q_max", k.d("limits.q_max_default"));
        if (qMax <= 0.0) throw fail(id, "airframe.q_max must be positive, got " + qMax);

        return new Airframe(
                refArea, wingArea, cd0, ar, oswald,
                opt(m, "lift_slope_per_deg", wingArea > 0.0
                        ? k.d("aerodynamics.default_lift_slope_per_deg") : 0.0),
                opt(m, "stall_aoa_deg", k.d("aerodynamics.default_stall_aoa_deg")),
                opt(m, "post_stall_aoa_deg", k.d("aerodynamics.post_stall_aoa_deg")),
                opt(m, "post_stall_cl_fraction", k.d("aerodynamics.post_stall_cl_fraction")),
                gLimit, qMax,
                opt(m, "nose_radius", k.d("reentry.default_nose_radius")),
                opt(m, "overheat_threshold", k.d("reentry.overheat_threshold_default")),
                opt(m, "rcs", k.d("sensors.rcs_bins.cruise_missile")));
    }

    @SuppressWarnings("unchecked")
    private Recovery recovery(String id, Object raw) {
        if (raw == null) return Recovery.none();
        if (!(raw instanceof List<?> l)) {
            throw fail(id, "'recovery' must be an array of parachutes, in deploy order");
        }
        List<Recovery.Parachute> chutes = new ArrayList<>();
        for (int i = 0; i < l.size(); i++) {
            if (!(l.get(i) instanceof Map)) throw fail(id, "recovery[" + i + "] must be an object");
            Map<String, Object> m = (Map<String, Object>) l.get(i);
            String where = "recovery[" + i + "]";
            double cd = req(m, "cd", id, where);
            double area = req(m, "area", id, where);
            if (cd <= 0.0 || area <= 0.0) {
                throw fail(id, where + " needs positive cd and area, got cd=" + cd + " area=" + area);
            }
            chutes.add(new Recovery.Parachute(
                    str(m, "name", "chute" + i), cd, area,
                    opt(m, "q_deploy_max", k.d("limits.chute_q_deploy_default")),
                    req(m, "deploy_altitude", id, where)));
        }
        // Deploy order sanity: chutes fire top-down, so each must deploy no higher than the
        // previous one. A main above its drogue would never get the chance to open.
        for (int i = 1; i < chutes.size(); i++) {
            if (chutes.get(i).deployAltitude() > chutes.get(i - 1).deployAltitude()) {
                throw fail(id, "recovery[" + i + "] ('" + chutes.get(i).name() + "') deploys at "
                        + chutes.get(i).deployAltitude() + " m, above the previous chute's "
                        + chutes.get(i - 1).deployAltitude() + " m. Chutes are listed in deploy "
                        + "order, highest first (drogue, then main).");
            }
        }
        return new Recovery(chutes);
    }

    @SuppressWarnings("unchecked")
    private SeekerSpec seeker(String id, Object raw) {
        if (raw == null) return SeekerSpec.none();
        if (!(raw instanceof Map)) throw fail(id, "'seeker' must be an object");
        Map<String, Object> m = (Map<String, Object>) raw;

        String qualityName = str(m, "quality", "STANDARD").toUpperCase(java.util.Locale.ROOT);
        SeekerSpec.Quality quality;
        try {
            quality = SeekerSpec.Quality.valueOf(qualityName);
        } catch (IllegalArgumentException e) {
            throw fail(id, "seeker.quality must be one of CHEAP, STANDARD, ADVANCED - got '"
                    + qualityName + "'");
        }

        double fov = req(m, "field_of_view_deg", id, "seeker");
        if (fov <= 0.0 || fov > 180.0) {
            throw fail(id, "seeker.field_of_view_deg must be in (0,180], got " + fov);
        }

        double n = opt(m, "pn_gain", k.d("guidance.pn_gain_default"));
        double nMin = k.d("guidance.pn_gain_min");
        double nMax = k.d("guidance.pn_gain_max");
        if (n < nMin || n > nMax) {
            warnings.add(id + " seeker.pn_gain of " + n + " is outside the stable band ["
                    + nMin + "," + nMax + "] and has been clamped (RC1).");
            n = Math.min(nMax, Math.max(nMin, n));
        }

        double minR = opt(m, "min_range", 0.0);
        double maxR = req(m, "max_range", id, "seeker");
        if (maxR <= minR) {
            throw fail(id, "seeker.max_range (" + maxR + ") must exceed min_range (" + minR + ")");
        }
        double floor = opt(m, "altitude_floor", -64.0);
        double ceiling = opt(m, "altitude_ceiling", 1.0e6);
        if (ceiling <= floor) {
            throw fail(id, "seeker.altitude_ceiling (" + ceiling + ") must exceed altitude_floor ("
                    + floor + ")");
        }

        return new SeekerSpec(quality, fov, n,
                opt(m, "memory_track_seconds", k.d("guidance.memory_track_seconds_default")),
                minR, maxR, floor, ceiling,
                opt(m, "max_crossing_rate_deg", 90.0),
                opt(m, "max_flight_time", 60.0),
                clamp01(opt(m, "flare_resistance", 0.5), id, "seeker.flare_resistance"),
                clamp01(opt(m, "chaff_resistance", 0.5), id, "seeker.chaff_resistance"));
    }

    // ---- primitives -------------------------------------------------------

    private double clamp01(double v, String id, String field) {
        if (v < 0.0 || v > 1.0) throw fail(id, field + " must be in [0,1], got " + v);
        return v;
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> mapOf(Map<String, Object> m, String key, String id) {
        Object o = m.get(key);
        if (!(o instanceof Map)) {
            throw fail(id, "'" + key + "' is required and must be an object");
        }
        return (Map<String, Object>) o;
    }

    private double req(Map<String, Object> m, String key, String id) {
        return req(m, key, id, null);
    }

    private double req(Map<String, Object> m, String key, String id, String where) {
        Object v = m.get(key);
        if (!(v instanceof Double d)) {
            String prefix = where == null ? "" : where + ".";
            throw fail(id, "'" + prefix + key + "' is required and must be a number"
                    + (v == null ? " (it is missing)" : " (found " + v + ")"));
        }
        if (!Double.isFinite(d)) {
            throw fail(id, "'" + key + "' must be finite, got " + d);
        }
        return d;
    }

    private static double opt(Map<String, Object> m, String key, double fallback) {
        Object v = m.get(key);
        return v instanceof Double d && Double.isFinite(d) ? d : fallback;
    }

    private static String str(Map<String, Object> m, String key, String fallback) {
        Object v = m.get(key);
        return v instanceof String s ? s : fallback;
    }

    private static ProfileException fail(String id, String message) {
        return new ProfileException("profile '" + id + "': " + message);
    }

    public static final class ProfileException extends RuntimeException {
        public ProfileException(String message) { super(message); }
    }
}
