package com.dronedelivery.algorithm;

import com.dronedelivery.util.DistanceMatrix;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Exact TSP solver using Branch and Bound with MST-based lower bound estimation.
 * <p>
 * The algorithm explores the solution space as a search tree where each node represents
 * a partial tour. Branches are pruned when the current cost plus a lower-bound estimate
 * (computed via a minimum spanning tree on unvisited nodes) exceeds the best known solution.
 * <p>
 * A nearest-neighbor heuristic provides the initial upper bound for aggressive early pruning.
 */
public class BranchAndBoundTSP {

    private static final Logger logger = LoggerFactory.getLogger(BranchAndBoundTSP.class);

    private final DistanceMatrix distanceMatrix;
    private final int n; // number of locations

    // Search state
    private double bestCost;
    private int[] bestTour;
    private int nodesExplored;
    private int branchesPruned;

    public BranchAndBoundTSP(DistanceMatrix distanceMatrix) {
        this.distanceMatrix = distanceMatrix;
        this.n = distanceMatrix.getSize();
    }

    /**
     * Solves the TSP and returns the optimal tour as an ordered list of location indices.
     * The tour starts and ends at index 0 (the depot).
     *
     * @return optimal tour as list of location indices (includes return to start)
     */
    public List<Integer> solve() {
        logger.info("Starting Branch & Bound TSP solver for {} locations", n);

        if (n <= 1) {
            return List.of(0, 0);
        }
        if (n == 2) {
            return List.of(0, 1, 0);
        }

        // Initialize best known solution using nearest-neighbor heuristic
        bestCost = distanceMatrix.nearestNeighborTourDistance(0);
        bestTour = buildNearestNeighborTour(0);
        nodesExplored = 0;
        branchesPruned = 0;

        logger.info("Initial upper bound (nearest-neighbor): {} km", bestCost);

        // Start branch and bound from depot (index 0)
        int[] currentTour = new int[n + 1]; // +1 for return to start
        boolean[] visited = new boolean[n];
        currentTour[0] = 0;
        visited[0] = true;

        branchAndBound(currentTour, visited, 1, 0.0);

        logger.info("B&B complete — optimal cost: {} km, nodes explored: {}, branches pruned: {}",
                bestCost, nodesExplored, branchesPruned);

        List<Integer> result = new ArrayList<>();
        for (int idx : bestTour) {
            result.add(idx);
        }
        return result;
    }

    /**
     * Recursive branch-and-bound search.
     *
     * @param currentTour partial tour being constructed
     * @param visited     which locations have been visited
     * @param depth       current depth in the search tree (number of locations placed)
     * @param currentCost accumulated distance so far
     */
    private void branchAndBound(int[] currentTour, boolean[] visited, int depth, double currentCost) {
        nodesExplored++;

        // Base case: all locations visited — close the tour
        if (depth == n) {
            double totalCost = currentCost + distanceMatrix.getDistance(currentTour[depth - 1], 0);
            if (totalCost < bestCost) {
                bestCost = totalCost;
                bestTour = Arrays.copyOf(currentTour, n + 1);
                bestTour[n] = 0; // return to depot
                logger.debug("New best tour found: cost={} km", bestCost);
            }
            return;
        }

        int lastLocation = currentTour[depth - 1];

        // Try each unvisited location as the next stop
        for (int next = 0; next < n; next++) {
            if (visited[next]) continue;

            double edgeCost = distanceMatrix.getDistance(lastLocation, next);
            double newCost = currentCost + edgeCost;

            // Compute lower bound: MST on remaining unvisited nodes + connections to partial tour
            double lowerBound = computeMSTLowerBound(visited, next, newCost);

            // Prune if lower bound exceeds best known solution
            if (lowerBound >= bestCost) {
                branchesPruned++;
                continue;
            }

            // Branch: extend the partial tour
            currentTour[depth] = next;
            visited[next] = true;

            branchAndBound(currentTour, visited, depth + 1, newCost);

            // Backtrack
            visited[next] = false;
        }
    }

    /**
     * Computes a lower bound on the total tour cost using a minimum spanning tree (MST)
     * over the unvisited nodes, plus minimum connection costs to/from the partial tour.
     * <p>
     * This uses Prim's algorithm for the MST computation.
     *
     * @param visited     which nodes are already in the partial tour
     * @param lastAdded   the most recently added node
     * @param currentCost the accumulated cost of the partial tour so far
     * @return lower bound on the complete tour cost
     */
    private double computeMSTLowerBound(boolean[] visited, int lastAdded, double currentCost) {
        // Collect unvisited nodes (excluding lastAdded which was just added)
        List<Integer> unvisited = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            if (!visited[i] && i != lastAdded) {
                unvisited.add(i);
            }
        }

        if (unvisited.isEmpty()) {
            // Only need to return to depot
            return currentCost + distanceMatrix.getDistance(lastAdded, 0);
        }

        // MST over unvisited nodes using Prim's algorithm
        double mstCost = computeMST(unvisited);

        // Minimum edge from lastAdded to any unvisited node
        double minFromCurrent = Double.MAX_VALUE;
        // Minimum edge from any unvisited node back to depot (node 0)
        double minToDepot = Double.MAX_VALUE;

        for (int u : unvisited) {
            minFromCurrent = Math.min(minFromCurrent, distanceMatrix.getDistance(lastAdded, u));
            minToDepot = Math.min(minToDepot, distanceMatrix.getDistance(u, 0));
        }

        return currentCost + minFromCurrent + mstCost + minToDepot;
    }

    /**
     * Computes the MST cost over a set of node indices using Prim's algorithm.
     */
    private double computeMST(List<Integer> nodes) {
        if (nodes.size() <= 1) return 0.0;

        int m = nodes.size();
        double[] minEdge = new double[m];
        boolean[] inMST = new boolean[m];
        Arrays.fill(minEdge, Double.MAX_VALUE);
        minEdge[0] = 0.0;

        double totalMST = 0.0;

        for (int count = 0; count < m; count++) {
            // Find the node with minimum edge cost not yet in MST
            int u = -1;
            double minCost = Double.MAX_VALUE;
            for (int i = 0; i < m; i++) {
                if (!inMST[i] && minEdge[i] < minCost) {
                    minCost = minEdge[i];
                    u = i;
                }
            }

            if (u == -1) break;

            inMST[u] = true;
            totalMST += minCost;

            // Update edges for remaining nodes
            int nodeU = nodes.get(u);
            for (int i = 0; i < m; i++) {
                if (!inMST[i]) {
                    double dist = distanceMatrix.getDistance(nodeU, nodes.get(i));
                    if (dist < minEdge[i]) {
                        minEdge[i] = dist;
                    }
                }
            }
        }

        return totalMST;
    }

    /**
     * Builds a nearest-neighbor tour starting from a given node.
     */
    private int[] buildNearestNeighborTour(int start) {
        int[] tour = new int[n + 1];
        boolean[] visited = new boolean[n];
        tour[0] = start;
        visited[start] = true;

        for (int step = 1; step < n; step++) {
            int current = tour[step - 1];
            int nearest = distanceMatrix.nearestUnvisited(current, visited);
            tour[step] = nearest;
            visited[nearest] = true;
        }

        tour[n] = start; // return to depot
        return tour;
    }

    // ── Accessors for statistics ────────────────────────────────────────

    public double getBestCost() {
        return bestCost;
    }

    public int getNodesExplored() {
        return nodesExplored;
    }

    public int getBranchesPruned() {
        return branchesPruned;
    }
}
