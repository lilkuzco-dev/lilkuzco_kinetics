package dev.lilkuzco.kinetics.body;

import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.phase.FlightPhase;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

/**
 * An immutable instant of a body's state.
 *
 * <p>Two jobs. It is what a server sends a client to interpolate between, and it is the unit
 * of a golden trajectory. For the second job the hashing below writes <em>raw IEEE-754 bit
 * patterns</em>, not formatted decimals: a golden hash has to fail when the last bit of a
 * position changes, and {@code String.format} would quietly round that difference away and
 * report determinism the sim did not actually have (I7).
 */
public record BodyState(
        String id,
        double age,
        Vec3 position,
        Vec3 velocity,
        Quat orientation,
        Vec3 angularVelocity,
        double mass,
        int stageIndex,
        double stageFuel,
        FlightPhase phase,
        double dynamicPressure,
        double mach,
        double angleOfAttackDeg,
        double heatingRate) {

    /** Append this state's exact bits to a digest. */
    public void hashInto(MessageDigest digest) {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream(160);
        try (DataOutputStream out = new DataOutputStream(bytes)) {
            out.writeLong(Double.doubleToRawLongBits(age));
            position.writeBits(out);
            velocity.writeBits(out);
            orientation.writeBits(out);
            angularVelocity.writeBits(out);
            out.writeLong(Double.doubleToRawLongBits(mass));
            out.writeInt(stageIndex);
            out.writeLong(Double.doubleToRawLongBits(stageFuel));
            out.writeInt(phase.ordinal());
        } catch (IOException e) {
            throw new IllegalStateException("in-memory stream failed", e);
        }
        digest.update(bytes.toByteArray());
    }

    public static MessageDigest newDigest() {
        try {
            return MessageDigest.getInstance("SHA-256");
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("SHA-256 is required by the Java platform", e);
        }
    }

    public static String hex(byte[] bytes) {
        StringBuilder sb = new StringBuilder(bytes.length * 2);
        for (byte b : bytes) sb.append(Character.forDigit((b >> 4) & 0xF, 16))
                              .append(Character.forDigit(b & 0xF, 16));
        return sb.toString();
    }

    /** Human-readable one-liner for diff plots when a golden hash fails. */
    public String toLine() {
        return String.format(
                "t=%8.4f pos=(%12.5f,%12.5f,%12.5f) v=(%10.5f,%10.5f,%10.5f) "
                + "|v|=%9.4f m=%10.4f st=%d fuel=%9.3f %s q=%9.1f M=%5.3f aoa=%6.2f qdot=%.4g",
                age, position.x(), position.y(), position.z(),
                velocity.x(), velocity.y(), velocity.z(), velocity.length(),
                mass, stageIndex, stageFuel, phase, dynamicPressure, mach,
                angleOfAttackDeg, heatingRate);
    }

    public boolean isFinite() {
        return Double.isFinite(age) && position.isFinite() && velocity.isFinite()
                && orientation.isFinite() && angularVelocity.isFinite()
                && Double.isFinite(mass) && Double.isFinite(stageFuel);
    }
}
