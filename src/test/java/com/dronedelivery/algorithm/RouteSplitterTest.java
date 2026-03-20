package com.dronedelivery.algorithm;

import com.dronedelivery.model.Location;
import com.dronedelivery.model.Route;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for the route splitter.
 */
class RouteSplitterTest {

    private final Location depot = new Location(0, "Depot", 41.88, -87.63);

    @Test
    void split_allWithinRange_returnsSingleRoute() {
        // All stops close together — should fit in one route
        Location a = new Location(1, "A", 41.89, -87.64);
        Location b = new Location(2, "B", 41.87, -87.62);

        List<Location> tour = List.of(depot, a, b, depot);
        Map<Integer, Double> payloads = Map.of(1, 1.0, 2, 1.0);

        RouteSplitter splitter = new RouteSplitter(100.0, 10.0); // generous limits
        List<Route> routes = splitter.split(tour, depot, payloads);

        assertEquals(1, routes.size(), "All stops should fit in a single route");
        assertTrue(routes.get(0).isFeasible(), "Route should be feasible");
        assertEquals(2, routes.get(0).getDeliveryCount(), "Should have 2 deliveries");
    }

    @Test
    void split_exceedsRange_splitsIntoMultipleRoutes() {
        // Stops spread out — tight range should force splitting
        Location a = new Location(1, "A", 41.95, -87.70);
        Location b = new Location(2, "B", 42.05, -87.80);
        Location c = new Location(3, "C", 41.75, -87.50);

        List<Location> tour = List.of(depot, a, b, c, depot);
        Map<Integer, Double> payloads = Map.of(1, 1.0, 2, 1.0, 3, 1.0);

        // Very tight range — should split
        RouteSplitter splitter = new RouteSplitter(20.0, 10.0);
        List<Route> routes = splitter.split(tour, depot, payloads);

        assertTrue(routes.size() > 1, "Tight range should force route splitting");

        // All sub-routes should start and end at depot
        for (Route route : routes) {
            List<Location> waypoints = route.getWaypoints();
            assertEquals(depot.getId(), waypoints.get(0).getId(),
                    "Route should start at depot");
            assertEquals(depot.getId(), waypoints.get(waypoints.size() - 1).getId(),
                    "Route should end at depot");
        }
    }

    @Test
    void split_exceedsPayload_splitsOnPayload() {
        // Two stops with heavy payloads — tight payload limit should split them
        Location a = new Location(1, "A", 41.89, -87.64);
        Location b = new Location(2, "B", 41.87, -87.62);

        List<Location> tour = List.of(depot, a, b, depot);
        Map<Integer, Double> payloads = Map.of(1, 4.0, 2, 4.0);

        RouteSplitter splitter = new RouteSplitter(100.0, 5.0); // generous range, tight payload
        List<Route> routes = splitter.split(tour, depot, payloads);

        assertEquals(2, routes.size(), "Heavy payloads should force splitting");
        for (Route route : routes) {
            assertTrue(route.getTotalPayloadKg() <= 5.0,
                    "Each route payload should not exceed limit");
        }
    }

    @Test
    void split_emptyTour_returnsEmptyList() {
        List<Location> tour = List.of(depot, depot);
        RouteSplitter splitter = new RouteSplitter(50.0, 5.0);
        List<Route> routes = splitter.split(tour, depot, Map.of());

        assertTrue(routes.isEmpty(), "Empty tour should produce no routes");
    }

    @Test
    void split_validatesAllRoutesFeasible() {
        Location a = new Location(1, "A", 41.89, -87.64);
        List<Location> tour = List.of(depot, a, depot);
        Map<Integer, Double> payloads = Map.of(1, 1.0);

        RouteSplitter splitter = new RouteSplitter(50.0, 5.0);
        List<Route> routes = splitter.split(tour, depot, payloads);

        RouteSplitter.ValidationResult validation = splitter.validate(routes);
        assertTrue(validation.isAllFeasible(), "All routes should be feasible");
        assertEquals(0, validation.infeasibleRoutes());
    }

    @Test
    void split_totalDeliveriesMatchInput() {
        Location a = new Location(1, "A", 41.90, -87.65);
        Location b = new Location(2, "B", 42.00, -87.75);
        Location c = new Location(3, "C", 41.80, -87.55);
        Location d = new Location(4, "D", 41.85, -87.70);

        List<Location> tour = List.of(depot, a, b, c, d, depot);
        Map<Integer, Double> payloads = Map.of(1, 1.0, 2, 1.0, 3, 1.0, 4, 1.0);

        RouteSplitter splitter = new RouteSplitter(25.0, 10.0);
        List<Route> routes = splitter.split(tour, depot, payloads);

        int totalDeliveries = routes.stream().mapToInt(Route::getDeliveryCount).sum();
        assertEquals(4, totalDeliveries, "Total deliveries should match input count");
    }
}
