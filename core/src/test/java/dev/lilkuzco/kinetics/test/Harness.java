package dev.lilkuzco.kinetics.test;

import java.util.ArrayList;
import java.util.List;

/**
 * A ~150-line test harness, so the whole battery runs on {@code javac} and {@code java} with
 * nothing downloaded.
 *
 * <p>That is not minimalism for its own sake. The physics core has zero third-party
 * dependencies precisely so that determinism (I7) cannot be perturbed by anything outside the
 * module, and a test runner that pulled in a dependency tree would undercut the claim it is
 * meant to verify. It also means CI needs no network and the numbers below can be reproduced
 * by anyone with a JDK.
 *
 * <p>Assertions print the actual numbers whether or not they pass. A cross-check that only says
 * "OK" is not evidence - the point of asserting terminal velocity against a closed form is to
 * see both values.
 */
public final class Harness {

    private final List<Result> results = new ArrayList<>();
    private String suite = "";
    private long suiteStart;

    public record Result(String suite, String name, boolean passed, String detail, long micros) {}

    public void suite(String name) {
        this.suite = name;
        this.suiteStart = System.nanoTime();
        System.out.println();
        System.out.println("── " + name + " " + "─".repeat(Math.max(0, 72 - name.length())));
    }

    public void endSuite() {
        long ms = (System.nanoTime() - suiteStart) / 1_000_000;
        System.out.printf("   (%s: %d ms)%n", suite, ms);
    }

    /** Record a check that has already been evaluated, with its numbers. */
    public void check(String name, boolean passed, String detail) {
        results.add(new Result(suite, name, passed, detail, 0));
        System.out.printf("  %s %-52s %s%n", passed ? "PASS" : "FAIL", name, detail);
    }

    /** Assert two values agree within a relative tolerance, printing both and the error. */
    public boolean near(String name, double actual, double expected, double relTolerance,
                        String units) {
        double error = expected == 0.0
                ? Math.abs(actual)
                : Math.abs(actual - expected) / Math.abs(expected);
        boolean ok = error <= relTolerance;
        check(name, ok, String.format("actual %.6g %s  expected %.6g %s  err %.4f%% (tol %.2f%%)",
                actual, units, expected, units, error * 100.0, relTolerance * 100.0));
        return ok;
    }

    public boolean isTrue(String name, boolean condition, String detail) {
        check(name, condition, detail);
        return condition;
    }

    public boolean greater(String name, double actual, double threshold, String units) {
        boolean ok = actual > threshold;
        check(name, ok, String.format("%.6g %s > %.6g %s", actual, units, threshold, units));
        return ok;
    }

    public boolean less(String name, double actual, double threshold, String units) {
        boolean ok = actual < threshold;
        check(name, ok, String.format("%.6g %s < %.6g %s", actual, units, threshold, units));
        return ok;
    }

    public boolean equalStrings(String name, String actual, String expected) {
        boolean ok = actual.equals(expected);
        check(name, ok, ok ? actual : "actual '" + actual + "' expected '" + expected + "'");
        return ok;
    }

    /** Run a body that is expected to throw, and report what it threw. */
    public boolean throwsWith(String name, Class<? extends Throwable> type, Runnable body) {
        try {
            body.run();
            check(name, false, "expected " + type.getSimpleName() + ", nothing was thrown");
            return false;
        } catch (Throwable t) {
            boolean ok = type.isInstance(t);
            String first = t.getMessage() == null ? t.getClass().getSimpleName()
                    : t.getMessage().lines().findFirst().orElse("");
            check(name, ok, (ok ? "threw " : "threw WRONG TYPE ")
                    + t.getClass().getSimpleName() + ": "
                    + (first.length() > 90 ? first.substring(0, 90) + "…" : first));
            return ok;
        }
    }

    /** An informational line that is not a pass/fail check. */
    public void note(String text) {
        System.out.println("       " + text);
    }

    public void metric(String name, String value) {
        System.out.printf("  %-4s %-52s %s%n", "····", name, value);
    }

    public int total() { return results.size(); }

    public int failures() { return (int) results.stream().filter(r -> !r.passed()).count(); }

    public List<Result> failed() { return results.stream().filter(r -> !r.passed()).toList(); }

    /** Final summary. Returns the process exit code. */
    public int summarize() {
        int failures = failures();
        System.out.println();
        System.out.println("═".repeat(78));
        if (failures == 0) {
            System.out.printf("ALL %d CHECKS PASSED%n", total());
        } else {
            System.out.printf("%d of %d CHECKS FAILED%n", failures, total());
            System.out.println();
            for (Result r : failed()) {
                System.out.printf("  [%s] %s%n      %s%n", r.suite(), r.name(), r.detail());
            }
        }
        System.out.println("═".repeat(78));
        return failures == 0 ? 0 : 1;
    }
}
