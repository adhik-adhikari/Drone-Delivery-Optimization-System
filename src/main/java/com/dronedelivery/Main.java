package com.dronedelivery;

import com.dronedelivery.model.FleetSolution;
import com.dronedelivery.simulation.DroneDeliverySimulator;
import com.dronedelivery.simulation.DroneDeliverySimulator.SimulationConfig;
import com.dronedelivery.simulation.SimulationReport;

/**
 * CLI entry point for the Drone Delivery Optimization System.
 * <p>
 * Usage:
 * <pre>
 *   java -jar drone-delivery-optimizer.jar --demo
 *   java -jar drone-delivery-optimizer.jar --locations 20 --drones 4 --range 60 --payload 5
 * </pre>
 */
public class Main {

    private static final String BANNER = """
            
            ╔══════════════════════════════════════════════════════════════╗
            ║                                                            ║
            ║     ██████╗ ██████╗  ██████╗ ███╗   ██╗███████╗           ║
            ║     ██╔══██╗██╔══██╗██╔═══██╗████╗  ██║██╔════╝           ║
            ║     ██║  ██║██████╔╝██║   ██║██╔██╗ ██║█████╗             ║
            ║     ██║  ██║██╔══██╗██║   ██║██║╚██╗██║██╔══╝             ║
            ║     ██████╔╝██║  ██║╚██████╔╝██║ ╚████║███████╗           ║
            ║     ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝           ║
            ║                                                            ║
            ║          DELIVERY OPTIMIZATION SYSTEM v1.0.0               ║
            ║   Multi-Drone TSP · Branch & Bound · Divide & Conquer     ║
            ║                                                            ║
            ╚══════════════════════════════════════════════════════════════╝
            """;

    public static void main(String[] args) {
        System.out.println(BANNER);

        // Parse command line arguments
        boolean demo = false;
        int locations = 15;
        int drones = 3;
        double range = 50.0;
        double payload = 5.0;
        long seed = System.currentTimeMillis();

        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--demo" -> demo = true;
                case "--locations" -> locations = Integer.parseInt(args[++i]);
                case "--drones" -> drones = Integer.parseInt(args[++i]);
                case "--range" -> range = Double.parseDouble(args[++i]);
                case "--payload" -> payload = Double.parseDouble(args[++i]);
                case "--seed" -> seed = Long.parseLong(args[++i]);
                case "--help", "-h" -> {
                    printUsage();
                    return;
                }
                default -> {
                    System.err.println("Unknown argument: " + args[i]);
                    printUsage();
                    return;
                }
            }
        }

        // Build scenario
        SimulationConfig config;
        if (demo) {
            System.out.println("  Running built-in demo scenario (Chicago metropolitan area)");
            System.out.println();
            config = DroneDeliverySimulator.createDemoScenario();
        } else {
            System.out.printf("  Running random scenario: %d locations, %d drones, %.1f km range, seed=%d%n",
                    locations, drones, range, seed);
            System.out.println();
            config = DroneDeliverySimulator.createRandomScenario(locations, drones, range, payload, seed);
        }

        // Run simulation
        DroneDeliverySimulator simulator = new DroneDeliverySimulator(config);
        FleetSolution solution = simulator.run();

        // Print report
        SimulationReport.print(solution);

        // Exit status based on feasibility
        if (!solution.isAllRoutesFeasible()) {
            System.out.println("  ⚠ Warning: Some routes are infeasible. Review the report above.");
            System.exit(1);
        }
    }

    private static void printUsage() {
        System.out.println("Usage: drone-delivery-optimizer [options]");
        System.out.println();
        System.out.println("Options:");
        System.out.println("  --demo              Run built-in demo scenario (Chicago area)");
        System.out.println("  --locations N       Number of delivery locations (default: 15)");
        System.out.println("  --drones N          Number of drones in fleet (default: 3)");
        System.out.println("  --range D           Max drone range in km (default: 50.0)");
        System.out.println("  --payload W         Max payload in kg (default: 5.0)");
        System.out.println("  --seed S            Random seed for reproducibility");
        System.out.println("  --help, -h          Show this help message");
    }
}
