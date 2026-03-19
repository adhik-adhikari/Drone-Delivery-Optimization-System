package com.dronedelivery.algorithm;

import com.dronedelivery.model.Location;
import com.dronedelivery.util.GeoUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Divide-and-conquer spatial decomposition engine.
 * <p>
 * Recursively partitions delivery locations into spatially coherent clusters using
 * a k-d tree style median-split strategy. Each resulting cluster is small enough
 * for efficient exact solving via branch-and-bound.
 * <p>
 * The decomposition alternates between splitting on latitude and longitude axes,
 * choosing the axis with the greatest spatial spread at each level.
 */
public class SpatialDecomposer {

    private static final Logger logger = LoggerFactory.getLogger(SpatialDecomposer.class);

    /** Maximum cluster size for exact B&B solving. Larger clusters are split further. */
    private final int maxClusterSize;

    /** Minimum cluster size. Clusters this size or smaller are never split. */
    private static final int MIN_CLUSTER_SIZE = 2;

    private int totalClustersFormed = 0;

    /**
     * @param maxClusterSize maximum number of delivery locations in a single cluster
     */
    public SpatialDecomposer(int maxClusterSize) {
        if (maxClusterSize < MIN_CLUSTER_SIZE) {
            throw new IllegalArgumentException("Max cluster size must be at least " + MIN_CLUSTER_SIZE);
        }
        this.maxClusterSize = maxClusterSize;
    }

    /**
     * Decomposes the delivery locations into spatially coherent clusters.
     * The depot (index 0) is NOT included in the clusters — it is handled separately.
     *
     * @param deliveryLocations list of delivery locations (excluding depot)
     * @return list of clusters, each a list of locations
     */
    public List<List<Location>> decompose(List<Location> deliveryLocations) {
        logger.info("Starting spatial decomposition of {} locations (max cluster size: {})",
                deliveryLocations.size(), maxClusterSize);

        totalClustersFormed = 0;
        List<List<Location>> clusters = new ArrayList<>();

        if (deliveryLocations.isEmpty()) {
            return clusters;
        }

        if (deliveryLocations.size() <= maxClusterSize) {
            clusters.add(new ArrayList<>(deliveryLocations));
            totalClustersFormed = 1;
            logger.info("All locations fit in a single cluster");
            return clusters;
        }

        // Start recursive decomposition
        recursiveDecompose(deliveryLocations, clusters, 0);

        logger.info("Decomposition complete: {} clusters formed", totalClustersFormed);
        return clusters;
    }

    /**
     * Recursively splits locations using alternating axis median partitioning.
     *
     * @param locations current set of locations to partition
     * @param clusters  accumulator for resulting clusters
     * @param depth     current recursion depth (determines split axis)
     */
    private void recursiveDecompose(List<Location> locations, List<List<Location>> clusters, int depth) {
        if (locations.size() <= maxClusterSize) {
            clusters.add(new ArrayList<>(locations));
            totalClustersFormed++;
            logger.debug("Formed cluster #{} with {} locations at depth {}",
                    totalClustersFormed, locations.size(), depth);
            return;
        }

        // Choose split axis based on spatial spread
        boolean splitOnLatitude = shouldSplitOnLatitude(locations);

        // Sort by the chosen axis
        List<Location> sorted = new ArrayList<>(locations);
        if (splitOnLatitude) {
            sorted.sort(Comparator.comparingDouble(Location::getLatitude));
        } else {
            sorted.sort(Comparator.comparingDouble(Location::getLongitude));
        }

        // Split at the median
        int median = sorted.size() / 2;
        List<Location> left = sorted.subList(0, median);
        List<Location> right = sorted.subList(median, sorted.size());

        logger.debug("Depth {}: splitting {} locations on {} axis → left={}, right={}",
                depth, locations.size(), splitOnLatitude ? "latitude" : "longitude",
                left.size(), right.size());

        // Recurse on both halves
        recursiveDecompose(new ArrayList<>(left), clusters, depth + 1);
        recursiveDecompose(new ArrayList<>(right), clusters, depth + 1);
    }

    /**
     * Determines whether to split on latitude or longitude based on which axis
     * has the greater spatial spread.
     */
    private boolean shouldSplitOnLatitude(List<Location> locations) {
        double minLat = Double.MAX_VALUE, maxLat = -Double.MAX_VALUE;
        double minLon = Double.MAX_VALUE, maxLon = -Double.MAX_VALUE;

        for (Location loc : locations) {
            minLat = Math.min(minLat, loc.getLatitude());
            maxLat = Math.max(maxLat, loc.getLatitude());
            minLon = Math.min(minLon, loc.getLongitude());
            maxLon = Math.max(maxLon, loc.getLongitude());
        }

        double latSpread = maxLat - minLat;
        double lonSpread = maxLon - minLon;

        return latSpread >= lonSpread;
    }

    /**
     * Merges sub-tours from individual clusters into a single global tour.
     * Uses a nearest-cluster-centroid strategy to determine cluster ordering,
     * then concatenates sub-tours with optimized cross-boundary transitions.
     *
     * @param depot       the depot location
     * @param clusterTours ordered list of sub-tours (each starting/ending at depot)
     * @return merged global tour as an ordered list of locations
     */
    public List<Location> mergeSubTours(Location depot, List<List<Location>> clusterTours) {
        if (clusterTours.isEmpty()) {
            return List.of(depot, depot);
        }
        if (clusterTours.size() == 1) {
            return clusterTours.get(0);
        }

        logger.info("Merging {} cluster sub-tours using nearest-centroid ordering", clusterTours.size());

        // Compute centroid for each cluster tour (excluding depot entries)
        List<Location> centroids = new ArrayList<>();
        for (List<Location> tour : clusterTours) {
            List<Location> deliveryStops = tour.stream()
                    .filter(loc -> loc.getId() != depot.getId())
                    .toList();
            if (!deliveryStops.isEmpty()) {
                centroids.add(GeoUtils.centroid(deliveryStops));
            } else {
                centroids.add(depot);
            }
        }

        // Order clusters by nearest-centroid from depot
        List<Integer> clusterOrder = new ArrayList<>();
        boolean[] assigned = new boolean[clusterTours.size()];

        Location currentPos = depot;
        for (int step = 0; step < clusterTours.size(); step++) {
            int nearest = -1;
            double minDist = Double.MAX_VALUE;
            for (int i = 0; i < centroids.size(); i++) {
                if (!assigned[i]) {
                    double dist = currentPos.distanceTo(centroids.get(i));
                    if (dist < minDist) {
                        minDist = dist;
                        nearest = i;
                    }
                }
            }
            clusterOrder.add(nearest);
            assigned[nearest] = true;
            currentPos = centroids.get(nearest);
        }

        // Build merged tour
        List<Location> mergedTour = new ArrayList<>();
        mergedTour.add(depot);

        for (int clusterIdx : clusterOrder) {
            List<Location> tour = clusterTours.get(clusterIdx);
            // Add delivery stops (skip depot entries from sub-tours)
            for (Location loc : tour) {
                if (loc.getId() != depot.getId()) {
                    mergedTour.add(loc);
                }
            }
        }

        mergedTour.add(depot); // return to depot
        return mergedTour;
    }

    public int getTotalClustersFormed() {
        return totalClustersFormed;
    }
}
