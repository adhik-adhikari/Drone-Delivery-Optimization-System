package com.dronedelivery.algorithm;

import com.dronedelivery.model.DeliveryRequest;
import com.dronedelivery.model.Location;
import com.dronedelivery.model.Route;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Splits a global TSP tour into multiple sub-routes that respect drone battery range
 * and payload capacity constraints.
 * <p>
 * Given a full tour (depot → deliveries → depot), this class segments it into feasible
 * sub-routes where each sub-route:
 * <ul>
 *   <li>Starts at the depot</li>
 *   <li>Visits a consecutive subset of the tour's delivery stops</li>
 *   <li>Returns to the depot</li>
 *   <li>Has total distance ≤ drone max range</li>
 *   <li>Has total payload ≤ drone max payload capacity</li>
 * </ul>
 * <p>
 * The algorithm greedily extends each sub-route until adding the next stop would violate
 * a constraint, then starts a new sub-route. It also accounts for the return-to-depot
 * distance when checking range feasibility.
 */
public class RouteSplitter {

    private static final Logger logger = LoggerFactory.getLogger(RouteSplitter.class);

    private final double maxRangeKm;
    private final double maxPayloadKg;
    private int routesSplit = 0;

    /**
     * @param maxRangeKm   maximum flight range per sub-route (round-trip from depot)
     * @param maxPayloadKg maximum payload per sub-route
     */
    public RouteSplitter(double maxRangeKm, double maxPayloadKg) {
        this.maxRangeKm = maxRangeKm;
        this.maxPayloadKg = maxPayloadKg;
    }

    /**
     * Splits a global tour into feasible sub-routes.
     *
     * @param globalTour       ordered list of locations (starts and ends at depot)
     * @param depot            the depot location
     * @param payloadMap       maps location ID → payload weight (kg) for each delivery stop
     * @return list of feasible sub-routes
     */
    public List<Route> split(List<Location> globalTour, Location depot,
                             Map<Integer, Double> payloadMap) {
        logger.info("Splitting tour of {} stops with range limit {} km and payload limit {} kg",
                globalTour.size(), maxRangeKm, maxPayloadKg);

        List<Route> subRoutes = new ArrayList<>();

        // Extract delivery stops (exclude depot entries)
        List<Location> deliveryStops = new ArrayList<>();
        for (Location loc : globalTour) {
            if (loc.getId() != depot.getId()) {
                deliveryStops.add(loc);
            }
        }

        if (deliveryStops.isEmpty()) {
            logger.info("No delivery stops to route");
            return subRoutes;
        }

        int i = 0;
        while (i < deliveryStops.size()) {
            List<Location> currentStops = new ArrayList<>();
            double currentDistance = 0.0;
            double currentPayload = 0.0;
            Location lastLocation = depot;

            // Greedily add stops to the current sub-route
            while (i < deliveryStops.size()) {
                Location nextStop = deliveryStops.get(i);
                double distToNext = lastLocation.distanceTo(nextStop);
                double distBackToDepot = nextStop.distanceTo(depot);
                double payloadForStop = payloadMap.getOrDefault(nextStop.getId(), 0.0);

                // Check if adding this stop keeps the route feasible
                double projectedDistance = currentDistance + distToNext + distBackToDepot;
                double projectedPayload = currentPayload + payloadForStop;

                if (!currentStops.isEmpty() &&
                        (projectedDistance > maxRangeKm || projectedPayload > maxPayloadKg)) {
                    // Adding this stop would violate constraints — close the current sub-route
                    break;
                }

                // Special case: single stop that exceeds range on its own
                if (currentStops.isEmpty() && projectedDistance > maxRangeKm) {
                    logger.warn("Location '{}' is unreachable within range {} km (requires {} km round-trip)",
                            nextStop.getName(), maxRangeKm, projectedDistance);
                    // Still add it but mark the route as infeasible
                    currentStops.add(nextStop);
                    currentDistance += distToNext;
                    currentPayload += payloadForStop;
                    i++;

                    Route infeasibleRoute = buildRoute(depot, currentStops, currentDistance, currentPayload, false,
                            String.format("Stop '%s' exceeds range (%.1f km needed, %.1f km available)",
                                    nextStop.getName(), projectedDistance, maxRangeKm));
                    subRoutes.add(infeasibleRoute);
                    currentStops = new ArrayList<>();
                    currentDistance = 0.0;
                    currentPayload = 0.0;
                    lastLocation = depot;
                    continue;
                }

                currentStops.add(nextStop);
                currentDistance += distToNext;
                currentPayload += payloadForStop;
                lastLocation = nextStop;
                i++;
            }

            // Build the sub-route (if there are stops)
            if (!currentStops.isEmpty()) {
                double returnDistance = currentDistance + lastLocation.distanceTo(depot);
                Route route = buildRoute(depot, currentStops, returnDistance, currentPayload, true, "");
                subRoutes.add(route);
            }
        }

        routesSplit = subRoutes.size();
        logger.info("Tour split into {} sub-routes", routesSplit);

        for (int r = 0; r < subRoutes.size(); r++) {
            Route route = subRoutes.get(r);
            logger.debug("  Route {}: {} deliveries, {} km, {} kg, feasible={}",
                    r + 1, route.getDeliveryCount(), route.getTotalDistanceKm(),
                    route.getTotalPayloadKg(), route.isFeasible());
        }

        return subRoutes;
    }

    /**
     * Builds a Route object from the given stops, adding depot at start and end.
     */
    private Route buildRoute(Location depot, List<Location> stops, double distance,
                             double payload, boolean feasible, String reason) {
        Route.Builder builder = new Route.Builder();
        builder.addWaypoint(depot);
        builder.addWaypoints(stops);
        builder.addWaypoint(depot);
        builder.totalDistanceKm(distance);
        builder.totalPayloadKg(payload);
        builder.feasible(feasible);
        if (!reason.isEmpty()) {
            builder.infeasibilityReason(reason);
        }
        return builder.build();
    }

    /**
     * Validates that all sub-routes are feasible and returns a summary.
     */
    public ValidationResult validate(List<Route> routes) {
        int feasibleCount = 0;
        int infeasibleCount = 0;
        double totalDistance = 0.0;
        List<String> issues = new ArrayList<>();

        for (int i = 0; i < routes.size(); i++) {
            Route route = routes.get(i);
            totalDistance += route.getTotalDistanceKm();

            if (route.isFeasible()) {
                feasibleCount++;
            } else {
                infeasibleCount++;
                issues.add(String.format("Route %d: %s", i + 1, route.getInfeasibilityReason()));
            }
        }

        return new ValidationResult(feasibleCount, infeasibleCount, totalDistance, issues);
    }

    public int getRoutesSplit() {
        return routesSplit;
    }

    // ── Validation Result ────────────────────────────────────────────────

    public record ValidationResult(
            int feasibleRoutes,
            int infeasibleRoutes,
            double totalDistance,
            List<String> issues
    ) {
        public boolean isAllFeasible() {
            return infeasibleRoutes == 0;
        }
    }
}
