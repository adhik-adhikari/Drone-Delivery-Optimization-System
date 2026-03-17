# Drone Delivery Optimization System

A Java-based multi-drone Traveling Salesperson Problem (TSP) solver that computes optimized flight paths under strict battery range and payload constraints using a hybrid **branch-and-bound** + **divide-and-conquer** algorithm.

## Architecture

```
com.dronedelivery
├── model/           # Data models (Location, Drone, Route, FleetSolution)
├── util/            # GeoUtils (Haversine), DistanceMatrix
├── algorithm/       # Core optimization algorithms
│   ├── BranchAndBoundTSP      - Exact TSP solver with MST lower bounds
│   ├── SpatialDecomposer      - K-d tree style spatial clustering
│   ├── RouteSplitter          - Battery-aware route segmentation
│   └── HybridOptimizer        - Full pipeline orchestrator
├── fleet/           # FleetCoordinator (drone assignment)
└── simulation/      # DroneDeliverySimulator, SimulationReport
```

## Algorithm

1. **Spatial Decomposition** — Recursively partitions delivery locations into clusters using k-d tree median splits (divide-and-conquer)
2. **Branch & Bound** — Solves TSP exactly within each cluster using MST-based lower bounds and nearest-neighbor upper bounds
3. **Sub-Tour Merging** — Combines cluster tours into a global tour using nearest-centroid ordering
4. **Route Splitting** — Segments the global tour into feasible sub-routes respecting battery range and payload limits

## Build & Run

```bash
# Build
mvn clean package -q

# Run demo (Chicago metropolitan area, 15 deliveries, 3 drones)
java -jar target/drone-delivery-optimizer-1.0.0-shaded.jar --demo

# Run custom scenario
java -jar target/drone-delivery-optimizer-1.0.0-shaded.jar \
  --locations 20 --drones 4 --range 60 --payload 5 --seed 42

# Run tests
mvn test
```

## CLI Options

| Option         | Description                          | Default |
|----------------|--------------------------------------|---------|
| `--demo`       | Run built-in Chicago demo scenario   | —       |
| `--locations N`| Number of delivery locations         | 15      |
| `--drones N`   | Number of drones in fleet            | 3       |
| `--range D`    | Max drone range in km                | 50.0    |
| `--payload W`  | Max payload capacity in kg           | 5.0     |
| `--seed S`     | Random seed for reproducibility      | random  |

## Tech Stack

- **Java 17** with text blocks and records
- **Maven** build system
- **JUnit 5** unit testing
- **SLF4J** logging

## Key Design Decisions

- **O(1) distance lookups** via precomputed `DistanceMatrix` using Haversine formula
- **MST-based pruning** in branch-and-bound dramatically reduces search space
- **Builder pattern** for `Route` and `FleetSolution` ensures immutability and clean construction
- **Greedy route splitting** with look-ahead for return-to-depot distance
- **Least-loaded-first** drone assignment for balanced fleet utilization
