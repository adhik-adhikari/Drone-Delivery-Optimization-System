package com.dronedelivery.util;

import org.junit.jupiter.api.Test;
import com.dronedelivery.model.Location;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for geographic utility functions.
 */
class GeoUtilsTest {

    @Test
    void haversineDistance_knownCities_returnsAccurateDistance() {
        // New York (40.7128, -74.0060) to Los Angeles (34.0522, -118.2437)
        // Known distance: ~3,944 km
        double distance = GeoUtils.haversineDistance(40.7128, -74.0060, 34.0522, -118.2437);
        assertEquals(3944, distance, 50, "NYC to LA should be approximately 3,944 km");
    }

    @Test
    void haversineDistance_samePoint_returnsZero() {
        double distance = GeoUtils.haversineDistance(41.8781, -87.6298, 41.8781, -87.6298);
        assertEquals(0.0, distance, 0.001, "Distance from a point to itself should be 0");
    }

    @Test
    void haversineDistance_shortDistance_returnsAccurateValue() {
        // Chicago Loop (41.8781, -87.6298) to Lincoln Park (41.9214, -87.6513)
        // Known distance: ~5.2 km
        double distance = GeoUtils.haversineDistance(41.8781, -87.6298, 41.9214, -87.6513);
        assertEquals(5.2, distance, 1.0, "Chicago Loop to Lincoln Park should be approximately 5.2 km");
    }

    @Test
    void haversineDistance_isSymmetric() {
        double d1 = GeoUtils.haversineDistance(40.0, -80.0, 42.0, -85.0);
        double d2 = GeoUtils.haversineDistance(42.0, -85.0, 40.0, -80.0);
        assertEquals(d1, d2, 0.001, "Haversine distance should be symmetric");
    }

    @Test
    void bearing_northward_returnsNearZero() {
        // Moving directly north
        double bearing = GeoUtils.bearing(40.0, -80.0, 42.0, -80.0);
        assertEquals(0.0, bearing, 1.0, "Moving due north should have bearing near 0°");
    }

    @Test
    void bearing_eastward_returnsNear90() {
        // Moving directly east
        double bearing = GeoUtils.bearing(40.0, -80.0, 40.0, -78.0);
        assertEquals(90.0, bearing, 5.0, "Moving due east should have bearing near 90°");
    }

    @Test
    void centroid_singleLocation_returnsSameLocation() {
        Location loc = new Location(1, "Test", 41.0, -87.0);
        Location centroid = GeoUtils.centroid(List.of(loc));
        assertEquals(41.0, centroid.getLatitude(), 0.01);
        assertEquals(-87.0, centroid.getLongitude(), 0.01);
    }

    @Test
    void centroid_symmetricLocations_returnsMidpoint() {
        Location north = new Location(1, "North", 42.0, -87.0);
        Location south = new Location(2, "South", 40.0, -87.0);
        Location centroid = GeoUtils.centroid(List.of(north, south));
        assertEquals(41.0, centroid.getLatitude(), 0.1);
        assertEquals(-87.0, centroid.getLongitude(), 0.1);
    }

    @Test
    void centroid_emptyList_throwsException() {
        assertThrows(IllegalArgumentException.class, () -> GeoUtils.centroid(List.of()));
    }

    @Test
    void totalPathDistance_computesCorrectSum() {
        Location a = new Location(1, "A", 41.0, -87.0);
        Location b = new Location(2, "B", 41.1, -87.0);
        Location c = new Location(3, "C", 41.2, -87.0);

        double expected = a.distanceTo(b) + b.distanceTo(c);
        double actual = GeoUtils.totalPathDistance(List.of(a, b, c));
        assertEquals(expected, actual, 0.001);
    }
}
