package ca.atlasengine.pathfinding.influence;

/**
 * Dynamic cost contribution from one navigation influence.
 */
public record InfluenceResult(boolean blocked, double costDelta, String reason) {
    public static final InfluenceResult NONE = new InfluenceResult(false, 0, "none");

    public InfluenceResult {
        if (!Double.isFinite(costDelta)) throw new IllegalArgumentException("costDelta");
        if (reason == null) throw new IllegalArgumentException("reason");
    }

    public static InfluenceResult penalty(double cost, String reason) {
        return new InfluenceResult(false, cost, reason);
    }

    public static InfluenceResult preference(double reward, String reason) {
        return new InfluenceResult(false, -Math.abs(reward), reason);
    }

    public static InfluenceResult forbidden(String reason) {
        return new InfluenceResult(true, 0, reason);
    }
}
