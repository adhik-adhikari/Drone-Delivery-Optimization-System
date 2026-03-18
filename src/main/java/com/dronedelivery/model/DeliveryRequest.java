package com.dronedelivery.model;

/**
 * Represents a single delivery request with destination, payload, and priority.
 */
public class DeliveryRequest {

    /**
     * Priority levels for delivery requests.
     */
    public enum Priority {
        LOW(1), MEDIUM(2), HIGH(3), URGENT(4);

        private final int weight;

        Priority(int weight) {
            this.weight = weight;
        }

        public int getWeight() {
            return weight;
        }
    }

    private final int id;
    private final Location destination;
    private final double payloadKg;
    private final Priority priority;

    public DeliveryRequest(int id, Location destination, double payloadKg, Priority priority) {
        if (payloadKg < 0) throw new IllegalArgumentException("Payload weight cannot be negative");
        this.id = id;
        this.destination = destination;
        this.payloadKg = payloadKg;
        this.priority = priority;
    }

    public DeliveryRequest(int id, Location destination, double payloadKg) {
        this(id, destination, payloadKg, Priority.MEDIUM);
    }

    public int getId() {
        return id;
    }

    public Location getDestination() {
        return destination;
    }

    public double getPayloadKg() {
        return payloadKg;
    }

    public Priority getPriority() {
        return priority;
    }

    @Override
    public String toString() {
        return String.format("DeliveryRequest{id=%d, dest='%s', payload=%.2fkg, priority=%s}",
                id, destination.getName(), payloadKg, priority);
    }
}
