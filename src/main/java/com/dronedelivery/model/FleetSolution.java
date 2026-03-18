package com.dronedelivery.model;

import java.util.*;

/**
 * Represents the complete fleet solution — all drone routes and aggregate statistics.
 */
public class FleetSolution {

    private final List<Route> routes;
    private final Map<Integer, Drone> droneAssignments; // route index → drone
    private final double totalFleetDistanceKm;
    private final boolean allRoutesFeasible;
    private final long computationTimeMs;
    private final int totalDeliveries;
    private final OptimizationStats stats;

    private FleetSolution(Builder builder) {
        this.routes = Collections.unmodifiableList(new ArrayList<>(builder.routes));
        this.droneAssignments = Collections.unmodifiableMap(new HashMap<>(builder.droneAssignments));
        this.totalFleetDistanceKm = builder.totalFleetDistanceKm;
        this.allRoutesFeasible = builder.allRoutesFeasible;
        this.computationTimeMs = builder.computationTimeMs;
        this.totalDeliveries = builder.totalDeliveries;
        this.stats = builder.stats;
    }

    public List<Route> getRoutes() {
        return routes;
    }

    public Map<Integer, Drone> getDroneAssignments() {
        return droneAssignments;
    }

    public double getTotalFleetDistanceKm() {
        return totalFleetDistanceKm;
    }

    public boolean isAllRoutesFeasible() {
        return allRoutesFeasible;
    }

    public long getComputationTimeMs() {
        return computationTimeMs;
    }

    public int getTotalDeliveries() {
        return totalDeliveries;
    }

    public OptimizationStats getStats() {
        return stats;
    }

    // ── Optimization Statistics ──────────────────────────────────────────

    public static class OptimizationStats {
        private final int totalNodes;
        private final int branchesPruned;
        private final int clustersFormed;
        private final int routesSplit;
        private final double improvementPercent;

        public OptimizationStats(int totalNodes, int branchesPruned, int clustersFormed,
                                 int routesSplit, double improvementPercent) {
            this.totalNodes = totalNodes;
            this.branchesPruned = branchesPruned;
            this.clustersFormed = clustersFormed;
            this.routesSplit = routesSplit;
            this.improvementPercent = improvementPercent;
        }

        public int getTotalNodes() { return totalNodes; }
        public int getBranchesPruned() { return branchesPruned; }
        public int getClustersFormed() { return clustersFormed; }
        public int getRoutesSplit() { return routesSplit; }
        public double getImprovementPercent() { return improvementPercent; }

        @Override
        public String toString() {
            return String.format(
                    "OptimizationStats{nodes=%d, pruned=%d, clusters=%d, splits=%d, improvement=%.1f%%}",
                    totalNodes, branchesPruned, clustersFormed, routesSplit, improvementPercent);
        }
    }

    // ── Builder ──────────────────────────────────────────────────────────

    public static class Builder {
        private final List<Route> routes = new ArrayList<>();
        private final Map<Integer, Drone> droneAssignments = new HashMap<>();
        private double totalFleetDistanceKm = 0.0;
        private boolean allRoutesFeasible = true;
        private long computationTimeMs = 0;
        private int totalDeliveries = 0;
        private OptimizationStats stats;

        public Builder addRoute(Route route, Drone drone) {
            int index = routes.size();
            routes.add(route);
            droneAssignments.put(index, drone);
            return this;
        }

        public Builder addRoute(Route route) {
            routes.add(route);
            return this;
        }

        public Builder totalFleetDistanceKm(double distance) {
            this.totalFleetDistanceKm = distance;
            return this;
        }

        public Builder allRoutesFeasible(boolean feasible) {
            this.allRoutesFeasible = feasible;
            return this;
        }

        public Builder computationTimeMs(long timeMs) {
            this.computationTimeMs = timeMs;
            return this;
        }

        public Builder totalDeliveries(int deliveries) {
            this.totalDeliveries = deliveries;
            return this;
        }

        public Builder stats(OptimizationStats stats) {
            this.stats = stats;
            return this;
        }

        /**
         * Auto-computes aggregate values from the added routes.
         */
        public Builder autoCompute() {
            this.totalFleetDistanceKm = routes.stream()
                    .mapToDouble(Route::getTotalDistanceKm).sum();
            this.allRoutesFeasible = routes.stream().allMatch(Route::isFeasible);
            this.totalDeliveries = routes.stream()
                    .mapToInt(Route::getDeliveryCount).sum();
            return this;
        }

        public FleetSolution build() {
            return new FleetSolution(this);
        }
    }
}
