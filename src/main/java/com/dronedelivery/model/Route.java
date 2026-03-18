package com.dronedelivery.model;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Represents an ordered route assigned to a single drone.
 * The route starts and ends at the depot (first location in the list).
 */
public class Route {

    private final List<Location> waypoints;
    private final double totalDistanceKm;
    private final double totalPayloadKg;
    private final boolean feasible;
    private final String infeasibilityReason;

    private Route(Builder builder) {
        this.waypoints = Collections.unmodifiableList(new ArrayList<>(builder.waypoints));
        this.totalDistanceKm = builder.totalDistanceKm;
        this.totalPayloadKg = builder.totalPayloadKg;
        this.feasible = builder.feasible;
        this.infeasibilityReason = builder.infeasibilityReason;
    }

    public List<Location> getWaypoints() {
        return waypoints;
    }

    public double getTotalDistanceKm() {
        return totalDistanceKm;
    }

    public double getTotalPayloadKg() {
        return totalPayloadKg;
    }

    public boolean isFeasible() {
        return feasible;
    }

    public String getInfeasibilityReason() {
        return infeasibilityReason;
    }

    public int getDeliveryCount() {
        // Subtract 2 for depot at start and end
        return Math.max(0, waypoints.size() - 2);
    }

    /**
     * Computes the total distance of the route by summing consecutive waypoint distances.
     */
    public static double computeDistance(List<Location> waypoints) {
        double total = 0.0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            total += waypoints.get(i).distanceTo(waypoints.get(i + 1));
        }
        return total;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Route{");
        sb.append(String.format("dist=%.2fkm, payload=%.2fkg, stops=%d, feasible=%s",
                totalDistanceKm, totalPayloadKg, getDeliveryCount(), feasible));
        if (!feasible) {
            sb.append(", reason='").append(infeasibilityReason).append("'");
        }
        sb.append(", path=[");
        for (int i = 0; i < waypoints.size(); i++) {
            if (i > 0) sb.append(" → ");
            sb.append(waypoints.get(i).getName());
        }
        sb.append("]}");
        return sb.toString();
    }

    // ── Builder ──────────────────────────────────────────────────────────

    public static class Builder {
        private final List<Location> waypoints = new ArrayList<>();
        private double totalDistanceKm = 0.0;
        private double totalPayloadKg = 0.0;
        private boolean feasible = true;
        private String infeasibilityReason = "";

        public Builder addWaypoint(Location location) {
            waypoints.add(location);
            return this;
        }

        public Builder addWaypoints(List<Location> locations) {
            waypoints.addAll(locations);
            return this;
        }

        public Builder totalDistanceKm(double distance) {
            this.totalDistanceKm = distance;
            return this;
        }

        public Builder totalPayloadKg(double payload) {
            this.totalPayloadKg = payload;
            return this;
        }

        public Builder feasible(boolean feasible) {
            this.feasible = feasible;
            return this;
        }

        public Builder infeasibilityReason(String reason) {
            this.infeasibilityReason = reason;
            return this;
        }

        /**
         * Auto-computes distance from waypoints if not explicitly set.
         */
        public Builder autoComputeDistance() {
            this.totalDistanceKm = Route.computeDistance(waypoints);
            return this;
        }

        public Route build() {
            return new Route(this);
        }
    }
}
