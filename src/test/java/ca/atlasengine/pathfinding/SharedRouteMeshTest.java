package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.adaptive.SharedNavigationRoute;
import ca.atlasengine.pathfinding.adaptive.SharedRouteMesh;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SharedRouteMeshTest {
    private static final BoundingBox BOX = new BoundingBox(0.6, 1.8, 0.6);
    private static final NavigationProfile PROFILE =
            BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);

    @Test
    void oneReverseFieldSelectsLeastCostDirectedRoutesForManySources() {
        NavigationPlan ab = segment(0, 1);
        NavigationPlan bd = segment(1, 3);
        NavigationPlan ac = segment(0, 2);
        NavigationPlan cd = segment(2, 3);
        NavigationPlan direct = segment(0, 3);
        SharedRouteMesh<String> mesh = SharedRouteMesh.<String>builder(12)
                .route("a", "b", ab, 2)
                .route("b", "d", bd, 2)
                .route("a", "c", ac, 1)
                .route("c", "d", cd, 1)
                .route("a", "d", direct, 10)
                .build();

        SharedRouteMesh.TargetField<String> field = mesh.routesTo("d", 12);
        SharedNavigationRoute<String> route =
                field.routeFrom("a").orElseThrow();

        assertEquals(2, field.costFrom("a"));
        assertEquals(List.of(ac, cd), route.segments());
        assertSame(route, field.routeFrom("a").orElseThrow(),
                "repeated mobs at a source should share the route object");
        for (int mob = 0; mob < 10_000; mob++) {
            assertSame(route, field.routeFrom("a").orElseThrow());
        }
        NavigationPlan combined = route.plan();
        assertSame(combined, route.plan());
        assertEquals(3, combined.nodes().size());
        assertEquals(3.5, combined.target().x());
        assertTrue(field.canReach("b"));
        assertFalse(field.canReach("unknown"));
    }

    @Test
    void fieldsRejectStaleWorldRevisionAndPreserveDirection() {
        SharedRouteMesh<String> mesh = SharedRouteMesh.<String>builder(5)
                .route("a", "b", segment(0, 1), 1)
                .node("isolated")
                .build();

        assertThrows(IllegalArgumentException.class,
                () -> mesh.routesTo("b", 6));
        var field = mesh.routesTo("b", 5);
        assertTrue(field.routeFrom("a").isPresent());
        assertTrue(field.routeFrom("isolated").isEmpty());
        assertTrue(mesh.routesTo("a", 5).routeFrom("b").isEmpty(),
                "a directed edge must not imply its reverse");
    }

    @Test
    void builderRejectsProfileOrBoundingBoxMixingBeforePublication() {
        NavigationPlan ground = segment(0, 1);
        NavigationProfile flying = BuiltinNavigationProfiles.forEntityType(
                EntityType.BEE);
        NavigationPlan incompatibleProfile = new NavigationPlan(
                ground.start(), ground.target(), BOX, flying,
                PathStatus.FOUND, ground.nodes(), 2, 8);
        NavigationPlan incompatibleBox = new NavigationPlan(
                ground.start(), ground.target(),
                new BoundingBox(1.8, 2.7, 1.8), PROFILE,
                PathStatus.FOUND, ground.nodes(), 2, 8);

        SharedRouteMesh.Builder<String> profiles =
                SharedRouteMesh.<String>builder(1)
                        .route("a", "b", ground, 1);
        assertThrows(IllegalArgumentException.class,
                () -> profiles.route(
                        "b", "c", incompatibleProfile, 1));

        SharedRouteMesh.Builder<String> boxes =
                SharedRouteMesh.<String>builder(1)
                        .route("a", "b", ground, 1);
        assertThrows(IllegalArgumentException.class,
                () -> boxes.route("b", "c", incompatibleBox, 1));
    }

    @Test
    void compositionRejectsSamePositionWithDifferentMovementIdentity() {
        NavigationPlan first = segment(0, 1);
        PathNode overlap = first.nodes().getLast();
        PathNode changed = new PathNode(
                overlap.x(), overlap.y(), overlap.z(),
                PathNode.Movement.CLIMB,
                overlap.graphX(), overlap.graphY(), overlap.graphZ());
        PathNode end = new PathNode(2.5, 2, 0.5,
                PathNode.Movement.CLIMB, 2, 2, 0);
        NavigationPlan second = new NavigationPlan(
                overlap.asVec(), end.asVec(), BOX, PROFILE,
                PathStatus.FOUND, List.of(changed, end), 2, 8);
        SharedNavigationRoute<String> route =
                SharedRouteMesh.<String>builder(1)
                        .route("a", "b", first, 1)
                        .route("b", "c", second, 1)
                        .build().routesTo("c", 1)
                        .routeFrom("a").orElseThrow();

        assertThrows(IllegalStateException.class, route::plan,
                "movement transitions cannot be merged by position alone");
    }

    @Test
    void compositionPreservesPartialOnlyAsTheTerminalSegment() {
        NavigationPlan first = segment(0, 1);
        NavigationPlan terminalPartial = withStatus(segment(1, 2),
                PathStatus.PARTIAL);
        NavigationPlan combined = SharedRouteMesh.<String>builder(1)
                .route("a", "b", first, 1)
                .route("b", "c", terminalPartial, 1)
                .build().routesTo("c", 1)
                .routeFrom("a").orElseThrow().plan();

        assertEquals(PathStatus.PARTIAL, combined.status(),
                "wall-climber fallback depends on retaining partial status");
        assertEquals(first.nodes().getFirst(), combined.nodes().getFirst());
        assertEquals(terminalPartial.nodes().getLast(),
                combined.nodes().getLast());

        NavigationPlan internalPartial = withStatus(first,
                PathStatus.PARTIAL);
        SharedNavigationRoute<String> invalid =
                SharedRouteMesh.<String>builder(1)
                        .route("a", "b", internalPartial, 1)
                        .route("b", "c", segment(1, 2), 1)
                        .build().routesTo("c", 1)
                        .routeFrom("a").orElseThrow();
        assertThrows(IllegalStateException.class, invalid::plan,
                "a partial segment cannot have another route appended");
    }

    private static NavigationPlan withStatus(
            NavigationPlan plan, PathStatus status) {
        return new NavigationPlan(
                plan.start(), plan.target(), plan.boundingBox(),
                plan.profile(), status, plan.nodes(),
                plan.visitedNodes(), plan.examinedNeighbors());
    }

    private static NavigationPlan segment(int from, int to) {
        Pos start = new Pos(from + 0.5, 1, 0.5);
        Pos target = new Pos(to + 0.5, 1, 0.5);
        return new NavigationPlan(start, target, BOX, PROFILE,
                PathStatus.FOUND,
                List.of(new PathNode(start.x(), 1, 0.5,
                                PathNode.Movement.WALK, from, 1, 0),
                        new PathNode(target.x(), 1, 0.5,
                                PathNode.Movement.WALK, to, 1, 0)),
                2, 8);
    }
}
