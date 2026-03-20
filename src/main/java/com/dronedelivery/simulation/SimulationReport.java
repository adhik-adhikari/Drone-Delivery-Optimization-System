package com.dronedelivery.simulation;

import com.dronedelivery.model.Drone;
import com.dronedelivery.model.FleetSolution;
import com.dronedelivery.model.Location;
import com.dronedelivery.model.Route;

import java.util.List;
import java.util.Map;

/**
 * Formats and prints detailed simulation reports to the console.
 */
public class SimulationReport {

    private static final String SEPARATOR = "═══════════════════════════════════════════════════════════════";
    private static final String THIN_SEP  = "───────────────────────────────────────────────────────────────";

    /**
     * Prints the complete simulation report.
     */
    public static void print(FleetSolution solution) {
        System.out.println();
        System.out.println(SEPARATOR);
        System.out.println("              DRONE DELIVERY OPTIMIZATION REPORT            ");
        System.out.println(SEPARATOR);

        printFleetSummary(solution);
        printRouteDetails(solution);
        printOptimizationStats(solution);
        printFeasibilityReport(solution);

        System.out.println(SEPARATOR);
        System.out.println();
    }

    private static void printFleetSummary(FleetSolution solution) {
        System.out.println();
        System.out.println("  FLEET SUMMARY");
        System.out.println(THIN_SEP);
        System.out.printf("  %-4s  %-15s  %-8s  %-12s  %-12s  %-10s%n",
                "#", "Drone", "Routes", "Distance", "Payload", "Status");
        System.out.println(THIN_SEP);

        Map<Integer, Drone> assignments = solution.getDroneAssignments();

        // Aggregate per-drone statistics
        java.util.Map<Integer, Double> droneDistances = new java.util.LinkedHashMap<>();
        java.util.Map<Integer, Double> dronePayloads = new java.util.LinkedHashMap<>();
        java.util.Map<Integer, Integer> droneRouteCounts = new java.util.LinkedHashMap<>();
        java.util.Map<Integer, Boolean> droneFeasibility = new java.util.LinkedHashMap<>();

        for (int i = 0; i < solution.getRoutes().size(); i++) {
            Route route = solution.getRoutes().get(i);
            Drone drone = assignments.get(i);
            if (drone != null) {
                int droneId = drone.getId();
                droneDistances.merge(droneId, route.getTotalDistanceKm(), Double::sum);
                dronePayloads.merge(droneId, route.getTotalPayloadKg(), Double::sum);
                droneRouteCounts.merge(droneId, 1, Integer::sum);
                droneFeasibility.merge(droneId, route.isFeasible(), (a, b) -> a && b);
            }
        }

        // Build unique drone list preserving order
        java.util.Set<Integer> printedDrones = new java.util.LinkedHashSet<>();
        for (int i = 0; i < solution.getRoutes().size(); i++) {
            Drone drone = assignments.get(i);
            if (drone != null && printedDrones.add(drone.getId())) {
                int droneId = drone.getId();
                String status = droneFeasibility.getOrDefault(droneId, true) ? "✓ OK" : "✗ ISSUE";
                System.out.printf("  %-4d  %-15s  %-8d  %8.2f km  %8.2f kg  %-10s%n",
                        droneId,
                        drone.getName(),
                        droneRouteCounts.getOrDefault(droneId, 0),
                        droneDistances.getOrDefault(droneId, 0.0),
                        dronePayloads.getOrDefault(droneId, 0.0),
                        status);
            }
        }

        System.out.println(THIN_SEP);
        System.out.printf("  TOTAL: %d routes, %.2f km total fleet distance, %d deliveries%n",
                solution.getRoutes().size(),
                solution.getTotalFleetDistanceKm(),
                solution.getTotalDeliveries());
    }

    private static void printRouteDetails(FleetSolution solution) {
        System.out.println();
        System.out.println("  ROUTE DETAILS");
        System.out.println(THIN_SEP);

        Map<Integer, Drone> assignments = solution.getDroneAssignments();

        for (int i = 0; i < solution.getRoutes().size(); i++) {
            Route route = solution.getRoutes().get(i);
            Drone drone = assignments.get(i);
            String droneName = drone != null ? drone.getName() : "Unassigned";

            System.out.printf("  Route %d [%s] — %.2f km, %.2f kg, %d deliveries%n",
                    i + 1, droneName,
                    route.getTotalDistanceKm(), route.getTotalPayloadKg(),
                    route.getDeliveryCount());

            // Print waypoint sequence
            List<Location> waypoints = route.getWaypoints();
            StringBuilder path = new StringBuilder("    ");
            for (int w = 0; w < waypoints.size(); w++) {
                if (w > 0) path.append(" → ");
                path.append(waypoints.get(w).getName());
            }
            System.out.println(path);

            // Print leg distances
            StringBuilder legs = new StringBuilder("    Legs: ");
            for (int w = 0; w < waypoints.size() - 1; w++) {
                if (w > 0) legs.append(", ");
                double legDist = waypoints.get(w).distanceTo(waypoints.get(w + 1));
                legs.append(String.format("%.2f km", legDist));
            }
            System.out.println(legs);

            if (drone != null) {
                double flightTime = drone.estimateFlightTimeMinutes(route.getTotalDistanceKm());
                System.out.printf("    Est. flight time: %.1f min (at %.0f km/h)%n",
                        flightTime, drone.getSpeedKmh());
            }

            if (!route.isFeasible()) {
                System.out.printf("    ⚠ INFEASIBLE: %s%n", route.getInfeasibilityReason());
            }

            System.out.println();
        }
    }

    private static void printOptimizationStats(FleetSolution solution) {
        FleetSolution.OptimizationStats stats = solution.getStats();
        if (stats == null) return;

        System.out.println("  OPTIMIZATION STATISTICS");
        System.out.println(THIN_SEP);
        System.out.printf("  Computation time:     %,d ms%n", solution.getComputationTimeMs());
        System.out.printf("  B&B nodes explored:   %,d%n", stats.getTotalNodes());
        System.out.printf("  Branches pruned:      %,d%n", stats.getBranchesPruned());
        System.out.printf("  Spatial clusters:     %d%n", stats.getClustersFormed());
        System.out.printf("  Routes after split:   %d%n", stats.getRoutesSplit());
        System.out.printf("  Distance improvement: %.1f%% vs naive ordering%n", stats.getImprovementPercent());
        System.out.println();
    }

    private static void printFeasibilityReport(FleetSolution solution) {
        System.out.println("  FEASIBILITY VALIDATION");
        System.out.println(THIN_SEP);

        if (solution.isAllRoutesFeasible()) {
            System.out.println("  ✓ All routes are feasible within battery range and payload limits.");
        } else {
            System.out.println("  ✗ Some routes have feasibility issues:");
            for (int i = 0; i < solution.getRoutes().size(); i++) {
                Route route = solution.getRoutes().get(i);
                if (!route.isFeasible()) {
                    System.out.printf("    Route %d: %s%n", i + 1, route.getInfeasibilityReason());
                }
            }
        }
        System.out.println();
    }
}
