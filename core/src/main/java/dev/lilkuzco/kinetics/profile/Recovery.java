package dev.lilkuzco.kinetics.profile;

import java.util.List;

/**
 * Parachute recovery, with the deploy gate that makes staging a real decision (RB6).
 *
 * <p>A canopy opened above its {@code qDeployMax} shreds. Kinetics emits the shred event and
 * stops counting that chute; it does not decide what shredding looks like (I10). The
 * consequence is that a capsule falling fast cannot simply open its main chute - it has to
 * deploy a small high-q drogue first, let drag bleed the speed down, and open the main once q
 * has fallen under the main's limit. That is exactly how real recovery sequences are designed,
 * and here it falls out of one inequality rather than a scripted sequence.
 */
public record Recovery(List<Parachute> chutes) {

    public Recovery {
        chutes = List.copyOf(chutes);
    }

    public static Recovery none() { return new Recovery(List.of()); }

    public boolean hasChutes() { return !chutes.isEmpty(); }

    /**
     * @param name           label for events and logs
     * @param cd             canopy drag coefficient, 1.5-1.75 for a hemispherical main (RB2)
     * @param area           inflated reference area, m^2
     * @param qDeployMax     dynamic pressure above which opening shreds the canopy, Pa (RB6)
     * @param deployAltitude altitude above the datum at which deployment is attempted, m
     */
    public record Parachute(
            String name,
            double cd,
            double area,
            double qDeployMax,
            double deployAltitude) {

        /** Whether opening now would survive (RB6). */
        public boolean survivesDeployAt(double dynamicPressure) {
            return dynamicPressure <= qDeployMax;
        }
    }
}
