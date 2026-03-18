package com.dronedelivery.util;

import com.dronedelivery.model.Location;

import java.util.List;

/**
 * Precomputed pairwise distance matrix for efficient O(1) lookups during optimization.
 * Uses the Haversine formula for geographic distance calculation.
 */
public class DistanceMatrix {

    private final double[][] distances;
    private final List<Location> locations;
    private final int size;

    /**
     * Constructs a distance matrix from a list of locations.
     * Location indices in the matrix correspond to their positions in the list.
     *
     * @param locations ordered list of locations (index 0 is typically the depot)
     */
    public DistanceMatrix(List<Location> locations) {
        this.locations = locations;
        this.size = locations.size();
        this.distances = new double[size][size];
        precompute();
    }

    private void precompute() {
        for (int i = 0; i < size; i++) {
            distances[i][i] = 0.0;
            for (int j = i + 1; j < size; j++) {
                double dist = locations.get(i).distanceTo(locations.get(j));
                distances[i][j] = dist;
                distances[j][i] = dist;
            }
        }
    }

    /**
     * Returns the distance between two locations by their indices.
     *
     * @param i index of the first location
     * @param j index of the second location
     * @return distance in kilometers
     */
    public double getDistance(int i, int j) {
        return distances[i][j];
    }

    /**
     * Returns the number of locations in the matrix.
     */
    public int getSize() {
        return size;
    }

    /**
     * Returns the location at a given index.
     */
    public Location getLocation(int index) {
        return locations.get(index);
    }

    /**
     * Finds the nearest unvisited neighbor to a given location index.
     *
     * @param from    source index
     * @param visited boolean array of visited locations
     * @return index of the nearest unvisited neighbor, or -1 if all visited
     */
    public int nearestUnvisited(int from, boolean[] visited) {
        int nearest = -1;
        double minDist = Double.MAX_VALUE;
        for (int i = 0; i < size; i++) {
            if (!visited[i] && distances[from][i] < minDist) {
                minDist = distances[from][i];
                nearest = i;
            }
        }
        return nearest;
    }

    /**
     * Computes a nearest-neighbor tour starting from a given location index.
     * Used as an initial upper bound for branch-and-bound.
     *
     * @param start starting location index
     * @return total tour distance (returning to start)
     */
    public double nearestNeighborTourDistance(int start) {
        boolean[] visited = new boolean[size];
        visited[start] = true;
        double totalDist = 0.0;
        int current = start;

        for (int step = 1; step < size; step++) {
            int next = nearestUnvisited(current, visited);
            if (next == -1) break;
            totalDist += distances[current][next];
            visited[next] = true;
            current = next;
        }

        totalDist += distances[current][start]; // Return to start
        return totalDist;
    }
}
