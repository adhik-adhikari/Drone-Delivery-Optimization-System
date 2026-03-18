package com.dronedelivery.model;

import com.dronedelivery.util.GeoUtils;

/**
 * Represents a geographic location for delivery or depot.
 */
public class Location {

    private final int id;
    private final String name;
    private final double latitude;
    private final double longitude;

    public Location(int id, String name, double latitude, double longitude) {
        this.id = id;
        this.name = name;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    /**
     * Computes the great-circle distance to another location using the Haversine formula.
     *
     * @param other the target location
     * @return distance in kilometers
     */
    public double distanceTo(Location other) {
        return GeoUtils.haversineDistance(
                this.latitude, this.longitude,
                other.latitude, other.longitude
        );
    }

    @Override
    public String toString() {
        return String.format("Location{id=%d, name='%s', lat=%.4f, lon=%.4f}", id, name, latitude, longitude);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Location location = (Location) o;
        return id == location.id;
    }

    @Override
    public int hashCode() {
        return Integer.hashCode(id);
    }
}
