package dev.lilkuzco.kinetics.orbit;

import dev.lilkuzco.kinetics.math.Vec3;

/**
 * Where a satellite is over the ground, in the <b>rotating</b> frame (RE5b).
 *
 * <p>This is the class RE5b exists to force into being. Propagating an orbit gives an inertial
 * position - fixed relative to the stars. The planet turns underneath it, so the point on the
 * ground below the satellite is not where the inertial longitude says it is. Predicting a
 * second pass in the inertial frame is not slightly wrong, it is wrong by the entire angle the
 * planet turned through in the meantime, and the error grows with every orbit.
 *
 * <p>The correction is one subtraction: {@code lon_fixed = lon_inertial - rotationRate * t}.
 * Its consequence is the familiar westward drift of successive ground tracks, and the size of
 * that drift is exactly {@code (T_orbit / T_day) * 360} degrees per revolution - the closed
 * form the test battery checks against.
 *
 * @param latitudeDeg   -90 to +90; bounded by the orbit's inclination
 * @param longitudeDeg  -180 to +180, in the rotating frame
 * @param worldX        Minecraft x of the sub-satellite point
 * @param worldZ        Minecraft z of the sub-satellite point
 * @param altitude      metres above the datum
 */
public record GroundTrack(
        double latitudeDeg,
        double longitudeDeg,
        double worldX,
        double worldZ,
        double altitude) {

    /**
     * Project a sub-satellite latitude and longitude onto Minecraft coordinates.
     *
     * <p>The mapping is the planet's own surface geometry: longitude runs along x at
     * {@code circumference/360} metres per degree, latitude runs along z at {@code R} metres
     * per radian. Longitude zero is world x zero, so a base built near spawn sits near the
     * prime meridian.
     */
    public static GroundTrack of(double latitudeDeg, double longitudeDeg, double altitude,
                                 double planetRadius) {
        double circumference = 2.0 * Math.PI * planetRadius;
        double lon = normalizeLongitude(longitudeDeg);
        double worldX = (lon / 360.0) * circumference;
        double worldZ = Math.toRadians(latitudeDeg) * planetRadius;
        return new GroundTrack(latitudeDeg, lon, worldX, worldZ, altitude);
    }

    /** Wrap a longitude into [-180, 180). */
    public static double normalizeLongitude(double degrees) {
        double d = degrees % 360.0;
        if (d >= 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }

    /**
     * Great-circle ground distance from this sub-satellite point to a world position, metres.
     *
     * <p>Computed on the sphere with the haversine formula rather than as a flat difference in
     * world coordinates. Near the equator the two agree; near the poles, and across the
     * longitude wrap at +/-180, the flat version is badly wrong - and a polar orbit spends most
     * of its time exactly where the flat version fails.
     */
    public double groundDistanceTo(double targetWorldX, double targetWorldZ, double planetRadius) {
        double circumference = 2.0 * Math.PI * planetRadius;
        double targetLat = Math.toRadians(targetWorldZ / planetRadius);
        double targetLon = Math.toRadians(normalizeLongitude(targetWorldX / circumference * 360.0));
        double lat = Math.toRadians(latitudeDeg);
        double lon = Math.toRadians(longitudeDeg);

        double dLat = targetLat - lat;
        double dLon = targetLon - lon;
        double a = Math.sin(dLat * 0.5) * Math.sin(dLat * 0.5)
                + Math.cos(lat) * Math.cos(targetLat)
                * Math.sin(dLon * 0.5) * Math.sin(dLon * 0.5);
        if (a < 0.0) a = 0.0;
        if (a > 1.0) a = 1.0;
        return 2.0 * planetRadius * Math.asin(Math.sqrt(a));
    }

    /**
     * Radius on the ground visible from this altitude within a sensor half-angle, metres.
     *
     * <p>Flat-plane approximation {@code h * tan(theta)}, which is accurate while the footprint
     * is small against the planet's radius. At the reference orbit that is 2887 m against a
     * 342.5 km radius, so the curvature error is negligible; a much wider sensor or a much
     * higher orbit would need the spherical form.
     */
    public double footprintRadius(double sensorHalfAngleDeg) {
        double theta = Math.toRadians(Math.min(sensorHalfAngleDeg, 89.0));
        return altitude * Math.tan(theta);
    }

    public Vec3 worldPosition(double y) { return new Vec3(worldX, y, worldZ); }

    @Override
    public String toString() {
        return String.format("lat %+7.3f lon %+8.3f -> world (%.1f, %.1f) at %.1f m",
                latitudeDeg, longitudeDeg, worldX, worldZ, altitude);
    }
}
