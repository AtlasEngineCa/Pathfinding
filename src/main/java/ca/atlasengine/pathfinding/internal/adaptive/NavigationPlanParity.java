package ca.atlasengine.pathfinding.internal.adaptive;

import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import net.minestom.server.coordinate.Point;

/**
 * Production invariants used before a shared plan may replace an individual
 * pathfinder result.
 *
 * <p>Semantic parity is stricter than matching endpoints. It
 * includes the exact physical and graph waypoint coordinates and every
 * movement kind. Search counters are diagnostic work measurements and are not
 * follower semantics.</p>
 */
public final class NavigationPlanParity {
    private NavigationPlanParity() {
    }

    public static boolean requestCompatible(
            NavigationRequest request, NavigationPlan plan) {
        return request != null && plan != null
                && plan.usable()
                && request.boundingBox().equals(plan.boundingBox())
                && request.profile().equals(plan.profile())
                && samePoint(request.start(), plan.start())
                && samePoint(request.target(), plan.target());
    }

    public static boolean semanticallyEquivalent(
            NavigationPlan individual, NavigationPlan shared) {
        return individual != null && shared != null
                && individual.status() == shared.status()
                && individual.boundingBox().equals(shared.boundingBox())
                && individual.profile().equals(shared.profile())
                && samePoint(individual.start(), shared.start())
                && samePoint(individual.target(), shared.target())
                && individual.nodes().equals(shared.nodes());
    }

    public static void requireSemanticParity(
            NavigationPlan individual, NavigationPlan shared) {
        if (!semanticallyEquivalent(individual, shared)) {
            throw new IllegalArgumentException(
                    "shared plan is not semantically identical to its "
                            + "individual pathfinding certificate");
        }
    }

    static boolean samePoint(Point first, Point second) {
        return Double.doubleToLongBits(first.x())
                == Double.doubleToLongBits(second.x())
                && Double.doubleToLongBits(first.y())
                == Double.doubleToLongBits(second.y())
                && Double.doubleToLongBits(first.z())
                == Double.doubleToLongBits(second.z());
    }
}
