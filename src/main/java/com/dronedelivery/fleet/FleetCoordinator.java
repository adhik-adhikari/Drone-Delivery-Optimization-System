package com.dronedelivery.fleet;

import com.dronedelivery.model.Drone;
import com.dronedelivery.model.FleetSolution;
import com.dronedelivery.model.Route;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Coordinates the assignment of optimized routes to available drones in the fleet.
 * <p>
 * The coordinator ensures each drone's assigned route respects its specific range
 * and payload constraints, and balances workload across the fleet using a
 * least-loaded-first assignment strategy.
 */
public class FleetCoordinator {

    private static final Logger logger = LoggerFactory.getLogger(FleetCoordinator.class);

    private final List<Drone> fleet;

    /**
     * @param fleet list of available drones
     */
    public FleetCoordinator(List<Drone> fleet) {
        if (fleet.isEmpty()) {
            throw new IllegalArgumentException("Fleet must contain at least one drone");
        }
        this.fleet = new ArrayList<>(fleet);
    }

    /**
     * Assigns routes to drones using a least-loaded-first strategy with capacity validation.
     * <p>
     * Routes are sorted by distance (longest first) and assigned to the drone with
     * the least accumulated distance that can handle the route's requirements.
     *
     * @param routes      list of sub-routes from the optimizer
     * @param stats       optimization statistics to include in the solution
     * @param elapsedMs   total computation time
     * @return complete fleet solution with drone assignments
     */
    public FleetSolution assignRoutes(List<Route> routes,
                                       FleetSolution.OptimizationStats stats,
                                       long elapsedMs) {
        logger.info("Assigning {} routes to {} drones", routes.size(), fleet.size());

        // Sort routes longest-first for better load balancing
        List<Route> sortedRoutes = new ArrayList<>(routes);
        sortedRoutes.sort(Comparator.comparingDouble(Route::getTotalDistanceKm).reversed());

        // Track accumulated distance per drone for load balancing
        double[] droneLoad = new double[fleet.size()];
        int[] droneRouteCount = new int[fleet.size()];

        FleetSolution.Builder solutionBuilder = new FleetSolution.Builder();

        for (Route route : sortedRoutes) {
            int bestDroneIdx = findBestDrone(route, droneLoad);

            if (bestDroneIdx >= 0) {
                Drone assignedDrone = fleet.get(bestDroneIdx);
                droneLoad[bestDroneIdx] += route.getTotalDistanceKm();
                droneRouteCount[bestDroneIdx]++;

                solutionBuilder.addRoute(route, assignedDrone);
                logger.debug("  Route ({} km, {} kg) → {} (load: {} km)",
                        route.getTotalDistanceKm(), route.getTotalPayloadKg(),
                        assignedDrone.getName(), droneLoad[bestDroneIdx]);
            } else {
                // No suitable drone found — mark route but still add it
                logger.warn("  No suitable drone for route ({} km, {} kg) — assigning to least loaded",
                        route.getTotalDistanceKm(), route.getTotalPayloadKg());

                // Assign to least loaded drone anyway with a warning
                int leastLoaded = findLeastLoaded(droneLoad);
                Drone fallbackDrone = fleet.get(leastLoaded);
                droneLoad[leastLoaded] += route.getTotalDistanceKm();
                droneRouteCount[leastLoaded]++;

                Route infeasibleRoute = new Route.Builder()
                        .addWaypoints(route.getWaypoints())
                        .totalDistanceKm(route.getTotalDistanceKm())
                        .totalPayloadKg(route.getTotalPayloadKg())
                        .feasible(false)
                        .infeasibilityReason(String.format(
                                "No drone can fully handle this route (needs %.1f km range, %.1f kg payload)",
                                route.getTotalDistanceKm(), route.getTotalPayloadKg()))
                        .build();

                solutionBuilder.addRoute(infeasibleRoute, fallbackDrone);
            }
        }

        solutionBuilder.autoCompute()
                .computationTimeMs(elapsedMs)
                .stats(stats);

        FleetSolution solution = solutionBuilder.build();

        // Log fleet utilization summary
        logger.info("Fleet assignment complete:");
        for (int d = 0; d < fleet.size(); d++) {
            logger.info("  {} — {} routes, {} km total",
                    fleet.get(d).getName(), droneRouteCount[d], droneLoad[d]);
        }

        return solution;
    }

    /**
     * Finds the best drone for a given route: must have capacity and range,
     * then selects the one with the least current load.
     *
     * @return index of the best drone, or -1 if no drone is suitable
     */
    private int findBestDrone(Route route, double[] droneLoad) {
        int bestIdx = -1;
        double bestLoad = Double.MAX_VALUE;

        for (int i = 0; i < fleet.size(); i++) {
            Drone drone = fleet.get(i);

            // Check if drone can handle this route
            if (!drone.canCoverDistance(route.getTotalDistanceKm())) continue;
            if (!drone.canCarryPayload(route.getTotalPayloadKg())) continue;

            // Prefer the drone with the least accumulated load
            if (droneLoad[i] < bestLoad) {
                bestLoad = droneLoad[i];
                bestIdx = i;
            }
        }

        return bestIdx;
    }

    /**
     * Finds the drone with the least accumulated load.
     */
    private int findLeastLoaded(double[] droneLoad) {
        int minIdx = 0;
        for (int i = 1; i < droneLoad.length; i++) {
            if (droneLoad[i] < droneLoad[minIdx]) {
                minIdx = i;
            }
        }
        return minIdx;
    }
}
