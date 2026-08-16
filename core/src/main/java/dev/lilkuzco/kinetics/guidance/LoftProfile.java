package dev.lilkuzco.kinetics.guidance;

import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Lofted trajectory: climb first, then hand over to the terminal law (RC4).
 *
 * <p>Lofting buys range. A missile that climbs into thin air spends most of its flight where
 * drag is a fraction of what it is at sea level, then converts that altitude back into speed on
 * the way down. Every long-range weapon does it, and it is why an anti-ship missile's flight
 * path looks nothing like a straight line to the target.
 *
 * <p><b>The blend is the part that matters.</b> RC4 forbids a hard switch, and the reason is
 * visible rather than numerical: at the instant a hard switch fires, the commanded acceleration
 * jumps discontinuously, the airframe slews as fast as it is allowed, and the missile
 * <em>snaps</em>. Players read that snap instantly as a scripted animation rather than a flying
 * object. Blending the two commands over a couple of seconds with a cosine weight removes the
 * discontinuity in both the command and its first derivative, and the transition simply
 * disappears.
 */
public final class LoftProfile {

    private final double loftBias;
    private final double handoverAltitude;
    private final double handoverSeconds;
    private final double blendSeconds;

    private double blendElapsed;
    private boolean handoverStarted;

    /**
     * @param loftBias          upward acceleration bias during the climb, m/s^2
     * @param handoverAltitude  altitude at which to start blending into the terminal law, m
     * @param handoverSeconds   time after which to blend regardless of altitude, s
     */
    public LoftProfile(Constants k, double loftBias, double handoverAltitude,
                       double handoverSeconds) {
        this.loftBias = loftBias;
        this.handoverAltitude = handoverAltitude;
        this.handoverSeconds = handoverSeconds;
        this.blendSeconds = k.d("guidance.loft_blend_seconds");
    }

    /**
     * Blend the loft command into {@code terminalCommand}.
     *
     * @param altitude current altitude above the datum, m
     * @param age      seconds since launch
     * @param dt       substep or tick length, to advance the blend clock
     */
    public Vec3 blend(Vec3 terminalCommand, double altitude, double age, double dt) {
        if (!handoverStarted && (altitude >= handoverAltitude || age >= handoverSeconds)) {
            handoverStarted = true;
        }
        if (!handoverStarted) {
            return new Vec3(0.0, loftBias, 0.0);
        }

        blendElapsed += dt;
        double t = blendSeconds <= 0.0 ? 1.0 : Math.min(1.0, blendElapsed / blendSeconds);
        // Cosine weight: value and slope are both continuous at t=0 and t=1, so neither the
        // command nor its rate of change jumps at either end of the handover.
        double w = 0.5 - 0.5 * Math.cos(Math.PI * t);

        Vec3 loft = new Vec3(0.0, loftBias, 0.0);
        return loft.scale(1.0 - w).add(terminalCommand.scale(w));
    }

    /** Whether the handover has begun. */
    public boolean isHandingOver() { return handoverStarted; }

    /** Whether the blend has finished and the terminal law has full authority. */
    public boolean isComplete() {
        return handoverStarted && blendElapsed >= blendSeconds;
    }

    public double blendProgress() {
        if (!handoverStarted) return 0.0;
        return blendSeconds <= 0.0 ? 1.0 : Math.min(1.0, blendElapsed / blendSeconds);
    }
}
