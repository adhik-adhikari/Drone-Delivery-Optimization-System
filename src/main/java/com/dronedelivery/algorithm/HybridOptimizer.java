package com.dronedelivery.algorithm;

import com.dronedelivery.model.DeliveryRequest;
import com.dronedelivery.model.FleetSolution;
import com.dronedelivery.model.Location;
import com.dronedelivery.model.Route;
import com.dronedelivery.util.DistanceMatrix;
import com.dronedelivery.util.GeoUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Hybrid optimization engine that orchestrates the full TSP-solving pipeline:
 * <ol>
 *   <li>Spatial decomposition — divide delivery locations into manageable clusters</li>
 *   <li>Branch-and-bound exact solve — find optimal tour within each cluster</li>
 *   <li>Sub-tour merging — combine cluster tours into a global tour</li>
 *   <li>Route splitting — segment the global tour into feasible sub-routes</li>
 * </ol>
 * <p>
 * This hybrid approach combines the exactness of branch-and-bound with the scalability
 * of divide-and-conquer, producing near-optimal solutions for large problem instances
 * that would be intractable for pure exact methods.
 */
public class HybridOptimizer {

    private static final Logger logger = LoggerFactory.getLogger(HybridOptimizer.class);

    /** Maximum cluster size for exact B&B solving */
    private static final int DEFAULT_MAX_CLUSTER_SIZE = 10;

    private final int maxClusterSize;
    private final double maxRangeKm;
    private final double maxPayloadKg;

    // Aggregate statistics
    private int totalNodesExplored = 0;
    private int totalBranchesPruned = 0;
    private int totalClusters = 0;
    private int totalRoutesSplit = 0;
    private double naiveTourDistance = 0.0;
    private double optimizedDistance = 0.0;

    public HybridOptimizer(double maxRangeKm, double maxPayloadKg) {
        this(maxRangeKm, maxPayloadKg, DEFAULT_MAX_CLUSTER_SIZE);
    }

    public HybridOptimizer(double maxRangeKm, double maxPayloadKg, int maxClusterSize) {
        this.maxRangeKm = maxRangeKm;
        this.maxPayloadKg = maxPayloadKg;
        this.maxClusterSize = maxClusterSize;
    }

    /**
     * Runs the full optimization pipeline.
     *
     * @param depot    the depot location (start/end point)
     * @param requests list of delivery requests
     * @return list of feasible sub-routes ready for fleet assignment
     */
    public List<Route> optimize(Location depot, List<DeliveryRequest> requests) {
        long startTime = System.currentTimeMillis();
        logger.info("═══════════════════════════════════════════════════════════");
        logger.info("Starting hybrid optimization: {} deliveries, range={}km, payload={}kg",
                requests.size(), maxRangeKm, maxPayloadKg);
        logger.info("═══════════════════════════════════════════════════════════");

        // Build payload map: location ID → weight
        Map<Integer, Double> payloadMap = new HashMap<>();
        List<Location> deliveryLocations = new ArrayList<>();
        for (DeliveryRequest req : requests) {
            deliveryLocations.add(req.getDestination());
            payloadMap.put(req.getDestination().getId(), req.getPayloadKg());
        }

        // ── Phase 1: Spatial Decomposition ───────────────────────────────
        logger.info("──── Phase 1: Spatial Decomposition ────");
        SpatialDecomposer decomposer = new SpatialDecomposer(maxClusterSize);
        List<List<Location>> clusters = decomposer.decompose(deliveryLocations);
        totalClusters = decomposer.getTotalClustersFormed();
        logger.info("Formed {} clusters", totalClusters);

        // ── Phase 2: Branch-and-Bound Solve per Cluster ──────────────────
        logger.info("──── Phase 2: Branch & Bound per Cluster ────");
        List<List<Location>> clusterTours = new ArrayList<>();

        for (int c = 0; c < clusters.size(); c++) {
            List<Location> cluster = clusters.get(c);
            logger.info("Solving cluster {}/{} ({} locations)", c + 1, clusters.size(), cluster.size());

            // Build location list: depot + cluster locations
            List<Location> clusterWithDepot = new ArrayList<>();
            clusterWithDepot.add(depot);
            clusterWithDepot.addAll(cluster);

            DistanceMatrix dm = new DistanceMatrix(clusterWithDepot);
            BranchAndBoundTSP solver = new BranchAndBoundTSP(dm);
            List<Integer> tourIndices = solver.solve();

            totalNodesExplored += solver.getNodesExplored();
            totalBranchesPruned += solver.getBranchesPruned();

            // Convert indices back to locations
            List<Location> tour = tourIndices.stream()
                    .map(clusterWithDepot::get)
                    .collect(Collectors.toList());

            clusterTours.add(tour);

            logger.info("  Cluster {} optimal cost: {} km (explored {} nodes, pruned {} branches)",
                    c + 1, solver.getBestCost(), solver.getNodesExplored(), solver.getBranchesPruned());
        }

        // ── Phase 3: Merge Sub-Tours ─────────────────────────────────────
        logger.info("──── Phase 3: Merging Sub-Tours ────");
        List<Location> globalTour = decomposer.mergeSubTours(depot, clusterTours);
        double mergedDistance = GeoUtils.totalPathDistance(globalTour);
        logger.info("Merged global tour distance: {} km", mergedDistance);

        // Compute naive tour distance for comparison
        naiveTourDistance = computeNaiveTourDistance(depot, deliveryLocations);
        logger.info("Naive (sequential) tour distance: {} km", naiveTourDistance);

        // ── Phase 4: Route Splitting ─────────────────────────────────────
        logger.info("──── Phase 4: Route Splitting (battery constraints) ────");
        RouteSplitter splitter = new RouteSplitter(maxRangeKm, maxPayloadKg);
        List<Route> routes = splitter.split(globalTour, depot, payloadMap);
        totalRoutesSplit = splitter.getRoutesSplit();

        // Validate
        RouteSplitter.ValidationResult validation = splitter.validate(routes);
        optimizedDistance = validation.totalDistance();

        long elapsed = System.currentTimeMillis() - startTime;
        logger.info("═══════════════════════════════════════════════════════════");
        logger.info("Optimization complete in {} ms", elapsed);
        logger.info("  Total routes: {}", routes.size());
        logger.info("  Total distance: {} km", optimizedDistance);
        logger.info("  Feasible: {}", validation.isAllFeasible());
        logger.info("═══════════════════════════════════════════════════════════");

        return routes;
    }

    /**
     * Computes a naive sequential tour distance for comparison benchmarking.
     * Visits all delivery locations in their given order then returns to depot.
     */
    private double computeNaiveTourDistance(Location depot, List<Location> deliveries) {
        if (deliveries.isEmpty()) return 0.0;
        double dist = depot.distanceTo(deliveries.get(0));
        for (int i = 0; i < deliveries.size() - 1; i++) {
            dist += deliveries.get(i).distanceTo(deliveries.get(i + 1));
        }
        dist += deliveries.get(deliveries.size() - 1).distanceTo(depot);
        return dist;
    }

    /**
     * Builds a FleetSolution.OptimizationStats from the collected metrics.
     */
    public FleetSolution.OptimizationStats buildStats() {
        double improvement = naiveTourDistance > 0
                ? ((naiveTourDistance - optimizedDistance) / naiveTourDistance) * 100.0
                : 0.0;

        return new FleetSolution.OptimizationStats(
                totalNodesExplored,
                totalBranchesPruned,
                totalClusters,
                totalRoutesSplit,
                improvement
        );
    }

    public int getTotalNodesExplored() { return totalNodesExplored; }
    public int getTotalBranchesPruned() { return totalBranchesPruned; }
    public int getTotalClusters() { return totalClusters; }
    public int getTotalRoutesSplit() { return totalRoutesSplit; }
    public double getNaiveTourDistance() { return naiveTourDistance; }
    public double getOptimizedDistance() { return optimizedDistance; }
}
