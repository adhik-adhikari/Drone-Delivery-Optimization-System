package com.dronedelivery.model;

/**
 * Represents a drone with physical and operational constraints.
 */
public class Drone {

    private final int id;
    private final String name;
    private final double maxRangeKm;
    private final double maxPayloadKg;
    private final double speedKmh;

    public Drone(int id, String name, double maxRangeKm, double maxPayloadKg, double speedKmh) {
        if (maxRangeKm <= 0) throw new IllegalArgumentException("Max range must be positive");
        if (maxPayloadKg <= 0) throw new IllegalArgumentException("Max payload must be positive");
        if (speedKmh <= 0) throw new IllegalArgumentException("Speed must be positive");

        this.id = id;
        this.name = name;
        this.maxRangeKm = maxRangeKm;
        this.maxPayloadKg = maxPayloadKg;
        this.speedKmh = speedKmh;
    }

    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public double getMaxRangeKm() {
        return maxRangeKm;
    }

    public double getMaxPayloadKg() {
        return maxPayloadKg;
    }

    public double getSpeedKmh() {
        return speedKmh;
    }

    /**
     * Estimates flight time for a given distance.
     *
     * @param distanceKm distance in kilometers
     * @return estimated time in minutes
     */
    public double estimateFlightTimeMinutes(double distanceKm) {
        return (distanceKm / speedKmh) * 60.0;
    }

    /**
     * Checks if a route distance is within this drone's battery range.
     */
    public boolean canCoverDistance(double distanceKm) {
        return distanceKm <= maxRangeKm;
    }

    /**
     * Checks if a payload weight is within this drone's capacity.
     */
    public boolean canCarryPayload(double payloadKg) {
        return payloadKg <= maxPayloadKg;
    }

    @Override
    public String toString() {
        return String.format("Drone{id=%d, name='%s', range=%.1fkm, payload=%.1fkg, speed=%.1fkm/h}",
                id, name, maxRangeKm, maxPayloadKg, speedKmh);
    }
}
