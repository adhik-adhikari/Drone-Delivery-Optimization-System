package com.dronedelivery.algorithm;

import com.dronedelivery.model.Location;
import com.dronedelivery.util.DistanceMatrix;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for the Branch and Bound TSP solver.
 */
class BranchAndBoundTSPTest {

    @Test
    void solve_threeLocations_returnsOptimalTour() {
        // Triangle: three locations forming a simple triangle
        Location a = new Location(0, "Depot", 0.0, 0.0);
        Location b = new Location(1, "B", 0.1, 0.0);
        Location c = new Location(2, "C", 0.0, 0.1);

        DistanceMatrix dm = new DistanceMatrix(List.of(a, b, c));
        BranchAndBoundTSP solver = new BranchAndBoundTSP(dm);
        List<Integer> tour = solver.solve();

        // Tour should start and end at 0
        assertEquals(0, tour.get(0), "Tour should start at depot");
        assertEquals(0, tour.get(tour.size() - 1), "Tour should end at depot");

        // Tour should visit all locations exactly once (except depot)
        assertEquals(4, tour.size(), "Tour should have 4 entries (3 locations + return)");
        assertTrue(tour.contains(1), "Tour should visit location 1");
        assertTrue(tour.contains(2), "Tour should visit location 2");

        // The optimal cost should be the triangle perimeter
        double perimeter = a.distanceTo(b) + b.distanceTo(c) + c.distanceTo(a);
        assertEquals(perimeter, solver.getBestCost(), 0.1,
                "Optimal tour should equal triangle perimeter");
    }

    @Test
    void solve_fourLocationsSquare_returnsOptimalTour() {
        // Square arrangement: optimal tour visits corners in order
        Location depot = new Location(0, "Depot", 0.0, 0.0);
        Location b = new Location(1, "B", 0.1, 0.0);
        Location c = new Location(2, "C", 0.1, 0.1);
        Location d = new Location(3, "D", 0.0, 0.1);

        DistanceMatrix dm = new DistanceMatrix(List.of(depot, b, c, d));
        BranchAndBoundTSP solver = new BranchAndBoundTSP(dm);
        List<Integer> tour = solver.solve();

        assertEquals(0, tour.get(0), "Tour should start at depot");
        assertEquals(0, tour.get(tour.size() - 1), "Tour should end at depot");
        assertEquals(5, tour.size(), "Tour should have 5 entries (4 locations + return)");

        // The optimal tour should go around the square (not cross)
        double sideLength = depot.distanceTo(b);
        double optimalPerimeter = sideLength * 4;
        assertEquals(optimalPerimeter, solver.getBestCost(), sideLength * 0.1,
                "Optimal tour should be close to square perimeter (no crossing)");
    }

    @Test
    void solve_twoLocations_returnsSimpleTour() {
        Location depot = new Location(0, "Depot", 41.0, -87.0);
        Location dest = new Location(1, "Dest", 41.1, -87.0);

        DistanceMatrix dm = new DistanceMatrix(List.of(depot, dest));
        BranchAndBoundTSP solver = new BranchAndBoundTSP(dm);
        List<Integer> tour = solver.solve();

        assertEquals(List.of(0, 1, 0), tour);
    }

    @Test
    void solve_singleLocation_returnsTrivialTour() {
        Location depot = new Location(0, "Depot", 41.0, -87.0);

        DistanceMatrix dm = new DistanceMatrix(List.of(depot));
        BranchAndBoundTSP solver = new BranchAndBoundTSP(dm);
        List<Integer> tour = solver.solve();

        assertEquals(List.of(0, 0), tour);
    }

    @Test
    void solve_sixLocations_prunesBranches() {
        // Six well-spread locations — should exercise pruning
        Location depot = new Location(0, "Depot", 41.88, -87.63);
        Location l1 = new Location(1, "L1", 41.92, -87.65);
        Location l2 = new Location(2, "L2", 41.79, -87.59);
        Location l3 = new Location(3, "L3", 41.91, -87.68);
        Location l4 = new Location(4, "L4", 42.05, -87.69);
        Location l5 = new Location(5, "L5", 41.85, -87.78);

        DistanceMatrix dm = new DistanceMatrix(List.of(depot, l1, l2, l3, l4, l5));
        BranchAndBoundTSP solver = new BranchAndBoundTSP(dm);
        List<Integer> tour = solver.solve();

        // Should visit all locations
        assertEquals(7, tour.size(), "Tour should have 7 entries (6 locations + return)");
        assertEquals(0, tour.get(0));
        assertEquals(0, tour.get(6));

        // Pruning should have occurred
        assertTrue(solver.getBranchesPruned() > 0,
                "Solver should have pruned branches for 6 locations");
        assertTrue(solver.getNodesExplored() > 0,
                "Solver should have explored some nodes");

        // Result should be better than or equal to nearest-neighbor heuristic
        double nnCost = dm.nearestNeighborTourDistance(0);
        assertTrue(solver.getBestCost() <= nnCost + 0.001,
                "B&B solution should be at least as good as nearest-neighbor heuristic");
    }
}
