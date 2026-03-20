package com.dronedelivery.algorithm;

import com.dronedelivery.model.DeliveryRequest;
import com.dronedelivery.model.Location;
import com.dronedelivery.model.Route;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * End-to-end tests for the hybrid optimization pipeline.
 */
class HybridOptimizerTest {

    @Test
    void optimize_smallProblem_producesFeasibleSolution() {
        Location depot = new Location(0, "Depot", 41.88, -87.63);

        List<DeliveryRequest> requests = List.of(
                new DeliveryRequest(1, new Location(1, "A", 41.92, -87.65), 2.0),
                new DeliveryRequest(2, new Location(2, "B", 41.85, -87.60), 1.5),
                new DeliveryRequest(3, new Location(3, "C", 41.90, -87.68), 3.0),
                new DeliveryRequest(4, new Location(4, "D", 41.87, -87.62), 1.0),
                new DeliveryRequest(5, new Location(5, "E", 41.89, -87.66), 2.5)
        );

        HybridOptimizer optimizer = new HybridOptimizer(50.0, 10.0, 5);
        List<Route> routes = optimizer.optimize(depot, requests);

        // All routes should be feasible
        assertTrue(routes.stream().allMatch(Route::isFeasible),
                "All routes should be feasible");

        // Total deliveries should match input
        int totalDeliveries = routes.stream().mapToInt(Route::getDeliveryCount).sum();
        assertEquals(5, totalDeliveries, "All 5 deliveries should be covered");

        // All routes should start and end at depot
        for (Route route : routes) {
            List<Location> waypoints = route.getWaypoints();
            assertEquals(depot.getId(), waypoints.get(0).getId());
            assertEquals(depot.getId(), waypoints.get(waypoints.size() - 1).getId());
        }
    }

    @Test
    void optimize_tightRange_splitsRoutes() {
        Location depot = new Location(0, "Depot", 41.88, -87.63);

        List<DeliveryRequest> requests = List.of(
                new DeliveryRequest(1, new Location(1, "A", 41.95, -87.70), 1.0),
                new DeliveryRequest(2, new Location(2, "B", 42.05, -87.80), 1.0),
                new DeliveryRequest(3, new Location(3, "C", 41.75, -87.50), 1.0),
                new DeliveryRequest(4, new Location(4, "D", 41.80, -87.75), 1.0)
        );

        // Very tight range — should force multiple routes
        HybridOptimizer optimizer = new HybridOptimizer(20.0, 5.0, 4);
        List<Route> routes = optimizer.optimize(depot, requests);

        assertTrue(routes.size() > 1, "Tight range should produce multiple routes");
    }

    @Test
    void optimize_largerProblem_completesAndProducesStats() {
        Location depot = new Location(0, "Depot", 41.88, -87.63);

        // Generate 12 delivery locations
        List<DeliveryRequest> requests = new ArrayList<>();
        double[] lats = {41.92, 41.79, 41.91, 42.05, 41.85, 41.75,
                41.95, 41.83, 42.00, 41.87, 41.90, 41.84};
        double[] lons = {-87.65, -87.59, -87.68, -87.69, -87.78, -88.15,
                -87.70, -87.60, -87.74, -87.62, -87.66, -87.75};

        for (int i = 0; i < 12; i++) {
            Location loc = new Location(i + 1, "Stop-" + (i + 1), lats[i], lons[i]);
            requests.add(new DeliveryRequest(i + 1, loc, 1.0 + (i % 3)));
        }

        HybridOptimizer optimizer = new HybridOptimizer(55.0, 8.0, 6);
        List<Route> routes = optimizer.optimize(depot, requests);

        // Verify statistics
        assertTrue(optimizer.getTotalNodesExplored() > 0, "Should have explored nodes");
        assertTrue(optimizer.getTotalClusters() > 0, "Should have formed clusters");

        // Verify all deliveries covered
        int totalDeliveries = routes.stream().mapToInt(Route::getDeliveryCount).sum();
        assertEquals(12, totalDeliveries, "All 12 deliveries should be covered");

        // Verify optimized distance is positive
        assertTrue(optimizer.getOptimizedDistance() > 0, "Optimized distance should be positive");
    }

    @Test
    void optimize_emptyRequests_returnsEmptyRoutes() {
        Location depot = new Location(0, "Depot", 41.88, -87.63);
        HybridOptimizer optimizer = new HybridOptimizer(50.0, 5.0);
        List<Route> routes = optimizer.optimize(depot, List.of());

        assertTrue(routes.isEmpty(), "Empty requests should produce no routes");
    }

    @Test
    void optimize_singleDelivery_returnsSingleRoute() {
        Location depot = new Location(0, "Depot", 41.88, -87.63);
        List<DeliveryRequest> requests = List.of(
                new DeliveryRequest(1, new Location(1, "A", 41.90, -87.65), 2.0)
        );

        HybridOptimizer optimizer = new HybridOptimizer(50.0, 5.0);
        List<Route> routes = optimizer.optimize(depot, requests);

        assertEquals(1, routes.size(), "Single delivery should produce one route");
        assertEquals(1, routes.get(0).getDeliveryCount());
    }
}
