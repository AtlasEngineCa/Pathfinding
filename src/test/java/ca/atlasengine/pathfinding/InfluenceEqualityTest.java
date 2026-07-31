package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.influence.EntityFearInfluence;
import ca.atlasengine.pathfinding.influence.EntitySnapshot;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.OpenableBlockFamily;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Region sharing is decided by comparing the compatibility of two requests,
 * so every value reachable from one has to compare by value.
 */
class InfluenceEqualityTest {
    private static final BoundingBox SMALL = new BoundingBox(0.6, 1.8, 0.6);
    private static final NavigationProfile GROUND =
            BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
    private static final UUID FIRST = new UUID(1, 2);
    private static final UUID SECOND = new UUID(3, 4);

    @Test
    void lambdasDeclaringEqualKeysReachOneSharedRegion() {
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                promoteThirdActor(() -> fearOf(6), () -> fearOf(6),
                        () -> fearOf(6)),
                "lambdas built by one factory with equal declared keys "
                        + "never promoted to a shared region");
    }

    @Test
    void lambdasDeclaringDifferentKeysNeverShareARegion() {
        assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                promoteThirdActor(() -> fearOf(6), () -> fearOf(6),
                        () -> fearOf(7)));
    }

    /**
     * The key is compared alongside the implementation class, so an
     * implementation that names a value another one already uses stays on its
     * own plan rather than inheriting a route built under other rules.
     */
    @Test
    void equalKeysDeclaredByDifferentImplementationsStayApart() {
        Object shared = new FearKey(6);
        NavigationInfluence penalising = NavigationInfluence.keyed(
                (blocks, point, box) -> InfluenceResult.penalty(2, "near"),
                shared);
        NavigationInfluence forbidding = NavigationInfluence.keyed(
                (blocks, point, box) -> InfluenceResult.forbidden("near"),
                shared);

        assertNotEquals(penalising, forbidding);
        assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                promoteThirdActor(() -> penalising, () -> penalising,
                        () -> forbidding));
    }

    @Test
    void influencesWithoutADeclaredKeyAreLeftExactlyAsTheyWere() {
        NavigationInfluence lambda =
                (blocks, point, box) -> InfluenceResult.NONE;
        List<NavigationInfluence> influences = List.of(lambda, fearOf(6));

        assertSame(List.of(),
                NavigationInfluence.byDeclaredEquality(List.of()));
        assertSame(influences,
                NavigationInfluence.byDeclaredEquality(influences));
        assertNotEquals(
                NavigationInfluence.byDeclaredEquality(List.of(
                        (NavigationInfluence)
                                (blocks, point, box) -> InfluenceResult.NONE)),
                NavigationInfluence.byDeclaredEquality(List.of(
                        (NavigationInfluence)
                                (blocks, point, box) -> InfluenceResult.NONE)));
    }

    @Test
    void anOverriddenKeyDecidesSharingWithoutTouchingEquals() {
        assertNotEquals(new Leash(32), new Leash(32));

        assertEquals(
                NavigationInfluence.byDeclaredEquality(List.of(new Leash(32))),
                NavigationInfluence.byDeclaredEquality(List.of(new Leash(32))));
        assertEquals(
                NavigationInfluence.byDeclaredEquality(
                        List.of(new Leash(32))).hashCode(),
                NavigationInfluence.byDeclaredEquality(
                        List.of(new Leash(32))).hashCode());
        assertNotEquals(
                NavigationInfluence.byDeclaredEquality(List.of(new Leash(32))),
                NavigationInfluence.byDeclaredEquality(List.of(new Leash(16))));
    }

    @Test
    void keyedInfluencesStillEvaluateAsTheirDelegate() {
        NavigationInfluence keyed = fearOf(6);

        assertEquals(InfluenceResult.penalty(6, "fear"),
                keyed.evaluate(null, new Vec(0, 0, 0), SMALL));
        assertEquals(InfluenceResult.penalty(6, "fear"),
                keyed.evaluate(null, new Vec(0, 0, 0), SMALL,
                        SearchControl.NONE));
    }

    /**
     * Two mobs that collected one threat set in different orders evaluate
     * identically, so they have to compare equal, while a threat listed twice
     * is charged twice and must not collapse into a single mention.
     */
    @Test
    void fearFieldsIgnoreThreatOrderButNotThreatCount() {
        EntitySnapshot wolf = new EntitySnapshot(FIRST, EntityType.WOLF, new Vec(4, 1, 4));
        EntitySnapshot creeper =
                new EntitySnapshot(SECOND, EntityType.CREEPER, new Vec(9, 1, 9));
        EntityFearInfluence forward = fear(List.of(wolf, creeper));
        EntityFearInfluence reversed = fear(List.of(creeper, wolf));

        assertEquals(forward, reversed);
        assertEquals(forward.hashCode(), reversed.hashCode());
        for (int step = 0; step <= 12; step++) {
            Point probe = new Vec(step, 1, step);
            assertEquals(forward.evaluate(null, probe, SMALL),
                    reversed.evaluate(null, probe, SMALL),
                    "equal fear fields must evaluate identically at " + probe);
        }
        assertNotEquals(forward, fear(List.of(wolf, wolf)));
        assertNotEquals(forward, fear(List.of(wolf, creeper, wolf)));
        assertNotEquals(fear(List.of(wolf, wolf)), fear(List.of(wolf)));
        assertNotEquals(forward, fear(List.of(wolf,
                new EntitySnapshot(SECOND, EntityType.CREEPER, new Vec(9, 1, 9.5)))));
    }

    @Test
    void duplicatedThreatsAreChargedTwiceAndOrderNeverChangesTheCharge() {
        EntitySnapshot wolf = new EntitySnapshot(FIRST, EntityType.WOLF, new Vec(4, 1, 4));
        Point probe = new Vec(5, 1, 5);

        double once = fear(List.of(wolf)).evaluate(null, probe, SMALL).costDelta();
        double twice =
                fear(List.of(wolf, wolf)).evaluate(null, probe, SMALL).costDelta();

        assertTrue(once > 0);
        assertEquals(once * 2, twice, 1e-12);
    }

    @Test
    void everyValueReachableFromARequestComparesByValue() {
        assertEquals(new BoundingBox(0.6, 1.8, 0.6),
                new BoundingBox(0.6, 1.8, 0.6));
        assertEquals(profile(), profile());
        assertEquals(profile().hashCode(), profile().hashCode());
        assertEquals(profile().mobProfile(), profile().mobProfile());
        assertEquals(profile().groundCapabilities().platformJump(),
                profile().groundCapabilities().platformJump());
        assertEquals(profile().groundCapabilities().climbables(),
                profile().groundCapabilities().climbables());
        assertEquals(profile().mobProfile().blockManipulation(),
                profile().mobProfile().blockManipulation());

        EntityTraversalState positions = new EntityTraversalState(
                true, false, Set.of(Block.WATER, Block.LAVA), -64, 319,
                List.of(new Pos(1.5, 2.5, 3.5), new Pos(4.5, 5.5, 6.5)));
        EntityTraversalState vectors = new EntityTraversalState(
                true, false, Set.of(Block.LAVA, Block.WATER), -64, 319,
                List.of(new Vec(1, 2, 3), new Vec(4, 5, 6)));
        assertEquals(positions, vectors);
        assertEquals(positions.hashCode(), vectors.hashCode());
    }

    private static NavigationProfile profile() {
        MobTraversalProfile mob = MobTraversalProfile.builder("audited")
                .from(MobTraversalProfile.ANIMAL)
                .malus(TerrainType.WATER, 4)
                .blockManipulation(BlockManipulationCapabilities
                        .of(OpenableBlockFamily.DOOR).closingBehind())
                .build();
        return NavigationProfile.builder(NavigationMode.GROUND, mob, GroundCapabilities.STANDARD
                        .withPlatformJump(PlatformJumpCapabilities.acrossGaps(2))
                        .withClimbables(ClimbableCapabilities.STANDARD)).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build();
    }

    private record FearKey(double radius) {}

    private static NavigationInfluence fearOf(double radius) {
        return NavigationInfluence.keyed(
                (blocks, point, box) -> InfluenceResult.penalty(radius, "fear"),
                new FearKey(radius));
    }

    /** Declares a sharing key while keeping the inherited identity equality. */
    private static final class Leash implements NavigationInfluence {
        private final double radius;

        Leash(double radius) {
            this.radius = radius;
        }

        @Override
        public InfluenceResult evaluate(
                Block.Getter blocks, Point point, BoundingBox box) {
            return point.distance(Vec.ZERO) > radius
                    ? InfluenceResult.forbidden("leash") : InfluenceResult.NONE;
        }

        @Override
        public Object equalityKey() {
            return new FearKey(radius);
        }
    }

    private static EntityFearInfluence fear(List<EntitySnapshot> threats) {
        return new EntityFearInfluence(threats, 8, 1, 12);
    }

    private static SharedMeshSource promoteThirdActor(
            Supplier<NavigationInfluence> first,
            Supplier<NavigationInfluence> second,
            Supplier<NavigationInfluence> third) {
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> plan(request),
                        SharedMeshPolicy.builder()
                                .promotionRequests(2).retentionRequests(1)
                                .build());
        TestWorld world = new TestWorld();
        try {
            coordinator.request("seed-1", "target", 1, 0,
                    influenced(world, first.get())).plan().join();
            coordinator.request("seed-2", "target", 1, 1,
                    influenced(world, second.get())).plan().join();
            return coordinator.request("third", "target", 1, 2,
                    influenced(world, third.get())).source();
        } finally {
            coordinator.close();
        }
    }

    private static NavigationRequest influenced(
            TestWorld world, NavigationInfluence influence) {
        return NavigationRequest.builder(world, new Pos(1.5, 1, 1.5),
                        new Pos(10.5, 1, 1.5), SMALL, GROUND)
                .maxPathLength(128).nodeSearchRange(128)
                .influences(List.of(influence)).build();
    }

    private static CompletableFuture<NavigationPlan> plan(
            NavigationRequest request) {
        Pos start = request.start().asPos();
        Pos target = request.target().asPos();
        return CompletableFuture.completedFuture(new NavigationPlan(
                start, target, request.boundingBox(), request.profile(),
                PathStatus.FOUND,
                List.of(new PathNode(start.x(), start.y(), start.z(),
                                PathNode.Movement.WALK, start.blockX(),
                                start.blockY(), start.blockZ()),
                        new PathNode(target.x(), target.y(), target.z(),
                                PathNode.Movement.WALK, target.blockX(),
                                target.blockY(), target.blockZ())),
                2, 8));
    }
}
