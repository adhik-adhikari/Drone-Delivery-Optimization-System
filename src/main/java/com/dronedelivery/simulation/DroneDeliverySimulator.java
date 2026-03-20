package com.dronedelivery.simulation;

import com.dronedelivery.model.*;
import com.dronedelivery.algorithm.HybridOptimizer;
import com.dronedelivery.fleet.FleetCoordinator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

/**
 * Main simulation engine for drone delivery optimization.
 * <p>
 * Orchestrates the complete workflow: scenario generation → optimization →
 * fleet assignment → result reporting.
 */
public class DroneDeliverySimulator {

    private static final Logger logger = LoggerFactory.getLogger(DroneDeliverySimulator.class);

    private final SimulationConfig config;

    public DroneDeliverySimulator(SimulationConfig config) {
        this.config = config;
    }

    /**
     * Runs the simulation and returns the complete fleet solution.
     */
    public FleetSolution run() {
        logger.info("╔═══════════════════════════════════════════════════════════╗");
        logger.info("║           DRONE DELIVERY OPTIMIZATION SYSTEM             ║");
        logger.info("╚═══════════════════════════════════════════════════════════╝");

        long startTime = System.currentTimeMillis();

        // Generate scenario
        Location depot = config.getDepot();
        List<DeliveryRequest> requests = config.getDeliveryRequests();
        List<Drone> fleet = config.getFleet();

        logger.info("Scenario: {} deliveries, {} drones, depot at '{}'",
                requests.size(), fleet.size(), depot.getName());

        // Run optimization
        HybridOptimizer optimizer = new HybridOptimizer(
                config.getMaxRangeKm(), config.getMaxPayloadKg(), config.getMaxClusterSize());
        List<Route> routes = optimizer.optimize(depot, requests);

        // Assign routes to fleet
        long elapsed = System.currentTimeMillis() - startTime;
        FleetCoordinator coordinator = new FleetCoordinator(fleet);
        FleetSolution solution = coordinator.assignRoutes(routes, optimizer.buildStats(), elapsed);

        return solution;
    }

    // ── Simulation Configuration ─────────────────────────────────────────

    /**
     * Holds the complete simulation configuration.
     */
    public static class SimulationConfig {

        private final Location depot;
        private final List<DeliveryRequest> deliveryRequests;
        private final List<Drone> fleet;
        private final double maxRangeKm;
        private final double maxPayloadKg;
        private final int maxClusterSize;

        private SimulationConfig(Builder builder) {
            this.depot = builder.depot;
            this.deliveryRequests = Collections.unmodifiableList(builder.deliveryRequests);
            this.fleet = Collections.unmodifiableList(builder.fleet);
            this.maxRangeKm = builder.maxRangeKm;
            this.maxPayloadKg = builder.maxPayloadKg;
            this.maxClusterSize = builder.maxClusterSize;
        }

        public Location getDepot() { return depot; }
        public List<DeliveryRequest> getDeliveryRequests() { return deliveryRequests; }
        public List<Drone> getFleet() { return fleet; }
        public double getMaxRangeKm() { return maxRangeKm; }
        public double getMaxPayloadKg() { return maxPayloadKg; }
        public int getMaxClusterSize() { return maxClusterSize; }

        public static class Builder {
            private Location depot;
            private List<DeliveryRequest> deliveryRequests = new ArrayList<>();
            private List<Drone> fleet = new ArrayList<>();
            private double maxRangeKm = 50.0;
            private double maxPayloadKg = 5.0;
            private int maxClusterSize = 10;

            public Builder depot(Location depot) { this.depot = depot; return this; }
            public Builder deliveryRequests(List<DeliveryRequest> requests) { this.deliveryRequests = requests; return this; }
            public Builder fleet(List<Drone> fleet) { this.fleet = fleet; return this; }
            public Builder maxRangeKm(double km) { this.maxRangeKm = km; return this; }
            public Builder maxPayloadKg(double kg) { this.maxPayloadKg = kg; return this; }
            public Builder maxClusterSize(int size) { this.maxClusterSize = size; return this; }
            public SimulationConfig build() { return new SimulationConfig(this); }
        }
    }

    // ── Built-in Demo Scenarios ──────────────────────────────────────────

    /**
     * Creates a demo scenario with real-world city coordinates.
     * Simulates a delivery hub in downtown Chicago serving surrounding neighborhoods.
     */
    public static SimulationConfig createDemoScenario() {
        // Depot: Downtown Chicago distribution center
        Location depot = new Location(0, "Chicago Hub", 41.8781, -87.6298);

        // Delivery locations: neighborhoods and suburbs around Chicago
        List<DeliveryRequest> requests = List.of(
                new DeliveryRequest(1,
                        new Location(1, "Lincoln Park", 41.9214, -87.6513), 2.5, DeliveryRequest.Priority.HIGH),
                new DeliveryRequest(2,
                        new Location(2, "Hyde Park", 41.7943, -87.5907), 1.8, DeliveryRequest.Priority.MEDIUM),
                new DeliveryRequest(3,
                        new Location(3, "Wicker Park", 41.9088, -87.6796), 3.2, DeliveryRequest.Priority.URGENT),
                new DeliveryRequest(4,
                        new Location(4, "Evanston", 42.0451, -87.6877), 1.5, DeliveryRequest.Priority.LOW),
                new DeliveryRequest(5,
                        new Location(5, "Oak Park", 41.8850, -87.7845), 2.0, DeliveryRequest.Priority.MEDIUM),
                new DeliveryRequest(6,
                        new Location(6, "Naperville", 41.7508, -88.1535), 4.1, DeliveryRequest.Priority.HIGH),
                new DeliveryRequest(7,
                        new Location(7, "Schaumburg", 42.0334, -88.0834), 1.2, DeliveryRequest.Priority.MEDIUM),
                new DeliveryRequest(8,
                        new Location(8, "Cicero", 41.8456, -87.7539), 3.6, DeliveryRequest.Priority.LOW),
                new DeliveryRequest(9,
                        new Location(9, "Skokie", 42.0324, -87.7416), 2.8, DeliveryRequest.Priority.HIGH),
                new DeliveryRequest(10,
                        new Location(10, "Berwyn", 41.8506, -87.7936), 1.9, DeliveryRequest.Priority.MEDIUM),
                new DeliveryRequest(11,
                        new Location(11, "Rogers Park", 42.0087, -87.6676), 2.3, DeliveryRequest.Priority.URGENT),
                new DeliveryRequest(12,
                        new Location(12, "Bridgeport", 41.8381, -87.6510), 4.5, DeliveryRequest.Priority.LOW),
                new DeliveryRequest(13,
                        new Location(13, "Pilsen", 41.8523, -87.6616), 1.0, DeliveryRequest.Priority.MEDIUM),
                new DeliveryRequest(14,
                        new Location(14, "Lakeview", 41.9429, -87.6531), 3.3, DeliveryRequest.Priority.HIGH),
                new DeliveryRequest(15,
                        new Location(15, "Des Plaines", 42.0334, -87.8834), 2.7, DeliveryRequest.Priority.MEDIUM)
        );

        // Fleet of drones
        List<Drone> fleet = List.of(
                new Drone(1, "Falcon-1", 55.0, 6.0, 65.0),
                new Drone(2, "Falcon-2", 55.0, 6.0, 65.0),
                new Drone(3, "Hawk-1", 40.0, 8.0, 50.0)
        );

        return new SimulationConfig.Builder()
                .depot(depot)
                .deliveryRequests(requests)
                .fleet(fleet)
                .maxRangeKm(55.0)
                .maxPayloadKg(6.0)
                .maxClusterSize(8)
                .build();
    }

    /**
     * Creates a random scenario with the given parameters.
     */
    public static SimulationConfig createRandomScenario(int numLocations, int numDrones,
                                                         double maxRange, double maxPayload,
                                                         long seed) {
        Random rng = new Random(seed);

        // Depot at a central location
        double depotLat = 40.0 + rng.nextDouble() * 5;
        double depotLon = -90.0 + rng.nextDouble() * 5;
        Location depot = new Location(0, "Depot", depotLat, depotLon);

        // Generate random delivery locations within ~50km radius
        List<DeliveryRequest> requests = new ArrayList<>();
        DeliveryRequest.Priority[] priorities = DeliveryRequest.Priority.values();

        for (int i = 1; i <= numLocations; i++) {
            double lat = depotLat + (rng.nextDouble() - 0.5) * 0.6;
            double lon = depotLon + (rng.nextDouble() - 0.5) * 0.8;
            double payload = 0.5 + rng.nextDouble() * (maxPayload - 0.5);
            DeliveryRequest.Priority priority = priorities[rng.nextInt(priorities.length)];

            Location loc = new Location(i, "Stop-" + i, lat, lon);
            requests.add(new DeliveryRequest(i, loc, Math.round(payload * 10.0) / 10.0, priority));
        }

        // Generate fleet
        List<Drone> fleet = new ArrayList<>();
        for (int d = 1; d <= numDrones; d++) {
            double droneRange = maxRange * (0.8 + rng.nextDouble() * 0.4);
            double dronePayload = maxPayload * (0.8 + rng.nextDouble() * 0.4);
            double speed = 40.0 + rng.nextDouble() * 40.0;
            fleet.add(new Drone(d, "Drone-" + d,
                    Math.round(droneRange * 10.0) / 10.0,
                    Math.round(dronePayload * 10.0) / 10.0,
                    Math.round(speed * 10.0) / 10.0));
        }

        return new SimulationConfig.Builder()
                .depot(depot)
                .deliveryRequests(requests)
                .fleet(fleet)
                .maxRangeKm(maxRange)
                .maxPayloadKg(maxPayload)
                .maxClusterSize(Math.min(10, Math.max(4, numLocations / 3)))
                .build();
    }
}
