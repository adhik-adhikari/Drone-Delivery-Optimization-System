package com.dronedelivery.util;

import com.dronedelivery.model.Location;

import java.util.List;

/**
 * Geographic utility functions for distance and spatial calculations.
 */
public final class GeoUtils {

    private static final double EARTH_RADIUS_KM = 6371.0;

    private GeoUtils() {
        // Utility class — no instantiation
    }

    /**
     * Computes the great-circle distance between two points using the Haversine formula.
     *
     * @param lat1 latitude of point 1 (degrees)
     * @param lon1 longitude of point 1 (degrees)
     * @param lat2 latitude of point 2 (degrees)
     * @param lon2 longitude of point 2 (degrees)
     * @return distance in kilometers
     */
    public static double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);

        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(dLon / 2) * Math.sin(dLon / 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return EARTH_RADIUS_KM * c;
    }

    /**
     * Computes the initial bearing from point 1 to point 2.
     *
     * @return bearing in degrees (0–360)
     */
    public static double bearing(double lat1, double lon1, double lat2, double lon2) {
        double dLon = Math.toRadians(lon2 - lon1);
        double y = Math.sin(dLon) * Math.cos(Math.toRadians(lat2));
        double x = Math.cos(Math.toRadians(lat1)) * Math.sin(Math.toRadians(lat2))
                - Math.sin(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) * Math.cos(dLon);
        double bearing = Math.toDegrees(Math.atan2(y, x));
        return (bearing + 360) % 360;
    }

    /**
     * Computes the geographic centroid of a list of locations.
     *
     * @param locations list of locations
     * @return a new Location representing the centroid
     */
    public static Location centroid(List<Location> locations) {
        if (locations.isEmpty()) {
            throw new IllegalArgumentException("Cannot compute centroid of empty location list");
        }

        // Convert to Cartesian, average, then convert back for accuracy
        double x = 0, y = 0, z = 0;
        for (Location loc : locations) {
            double latRad = Math.toRadians(loc.getLatitude());
            double lonRad = Math.toRadians(loc.getLongitude());
            x += Math.cos(latRad) * Math.cos(lonRad);
            y += Math.cos(latRad) * Math.sin(lonRad);
            z += Math.sin(latRad);
        }

        int n = locations.size();
        x /= n;
        y /= n;
        z /= n;

        double centralLon = Math.atan2(y, x);
        double centralLat = Math.atan2(z, Math.sqrt(x * x + y * y));

        return new Location(-1, "Centroid", Math.toDegrees(centralLat), Math.toDegrees(centralLon));
    }

    /**
     * Computes the total path distance for an ordered list of locations.
     */
    public static double totalPathDistance(List<Location> path) {
        double total = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            total += path.get(i).distanceTo(path.get(i + 1));
        }
        return total;
    }
}
