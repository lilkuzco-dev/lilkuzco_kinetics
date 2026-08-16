package dev.lilkuzco.kinetics.test;

import dev.lilkuzco.kinetics.constants.Constants;

/**
 * The whole test battery. {@code java dev.lilkuzco.kinetics.test.TestMain [suite...]}
 *
 * <p>Exits nonzero on any failure, so it works as a CI gate unchanged. With no arguments it
 * runs everything; naming suites runs a subset while iterating.
 */
public final class TestMain {

    public static void main(String[] args) {
        Constants k = Constants.get();
        Harness h = new Harness();

        java.util.Set<String> only = args.length == 0 ? null
                : new java.util.LinkedHashSet<>(java.util.Arrays.asList(args));

        System.out.println("lilkuzco_kinetics — physics test battery");
        System.out.println("Java " + System.getProperty("java.version")
                + " on " + System.getProperty("os.arch"));

        long start = System.nanoTime();
        if (only == null || only.contains("closed-form")) ClosedFormTests.run(h, k);
        if (only == null || only.contains("invariants")) InvariantTests.run(h, k);
        if (only == null || only.contains("golden")) GoldenTests.run(h, k);
        if (only == null || only.contains("fuzz")) FuzzTests.run(h, k);
        if (only == null || only.contains("perf")) PerformanceTests.run(h, k);
        long ms = (System.nanoTime() - start) / 1_000_000;

        System.out.println();
        System.out.printf("total battery wall time: %d ms%n", ms);
        System.exit(h.summarize());
    }
}
