package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationMode;import ca.atlasengine.pathfinding.profile.NavigationModifiers;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.result.PathStatus;import ca.atlasengine.pathfinding.search.DiscreteGroundPathfinder;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.GroundSearchLimits;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainClassification;
import ca.atlasengine.pathfinding.terrain.TerrainClassifier;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * An integration names its own blocks in the baseline type vocabulary, so
 * every rule that already switches on a type prices them without being told.
 */
class TerrainClassificationTest {
    private static final BoundingBox NARROW = new BoundingBox(0.6, 1.8, 0.6);
    private static final Block CUSTOM = Block.PINK_WOOL;
    private final DiscreteGroundPathfinder pathfinder =
            new DiscreteGroundPathfinder();

    @Test
    void aNamedBlockRoutesUnderTheTypeItWasNamed() {
        TestWorld world = corridorPavedWith(CUSTOM);

        PathResult baseline = walk(world, MobTraversalProfile.DEFAULT);
        PathResult named = walk(world, naming(CUSTOM, TerrainType.DAMAGING));

        assertTrue(baseline.found(), baseline::toString);
        assertEquals(9, baseline.nodes().size(),
                "an unnamed wool floor is ordinary ground");
        assertFalse(named.found(), named::toString);
    }

    /**
     * The point of naming an existing type: the malus table a profile already
     * carries prices the new block with no further wiring.
     */
    @Test
    void aNamedTypeIsPricedByTheTableThatAlreadyPricesIt() {
        TestWorld world = corridorPavedWith(CUSTOM);
        MobTraversalProfile tolerant = MobTraversalProfile.builder("tolerant")
                .malus(TerrainType.DAMAGING, 0)
                .classification(block -> block.compare(CUSTOM)
                        ? TerrainType.DAMAGING : null)
                .build();

        PathResult result = walk(world, tolerant);

        assertTrue(result.found(),
                "a profile pricing DAMAGING at 0 walks the named floor");
        assertEquals(9, result.nodes().size());
        assertTrue(result.nodeCosts().stream().anyMatch(cost ->
                        cost.terrain() == TerrainType.DAMAGING),
                result::toString);
    }

    @Test
    void aHookThatDeclinesEverythingLeavesEveryRouteExactlyAsItWas() {
        TestWorld world = corridorPavedWith(Block.STONE);
        MobTraversalProfile declining = MobTraversalProfile.builder("default")
                .from(MobTraversalProfile.DEFAULT)
                .classification(block -> null)
                .build();

        PathResult baseline = walk(world, MobTraversalProfile.DEFAULT);
        PathResult hooked = walk(world, declining);

        assertEquals(baseline.status(), hooked.status());
        assertEquals(baseline.nodes(), hooked.nodes());
        assertEquals(baseline.nodeCosts(), hooked.nodeCosts());
        assertEquals(baseline.visitedNodes(), hooked.visitedNodes());
        assertEquals(baseline.examinedNeighbors(), hooked.examinedNeighbors());
    }

    @Test
    void theDefaultProfileCarriesTheDecliningConstant() {
        assertSame(TerrainClassification.NONE,
                MobTraversalProfile.DEFAULT.classification());
        assertNull(TerrainClassification.NONE.classify(Block.STONE));
        assertSame(MobTraversalProfile.DEFAULT,
                MobTraversalProfile.DEFAULT.withMemoizedClassification(),
                "a profile without a hook keeps the identical code path");
    }

    @Test
    void theHookReachesTheAnchoredClassifierEveryFollowerReads() {
        TestWorld world = new TestWorld().set(0, 0, 0, CUSTOM);
        TerrainClassifier classifier = new TerrainClassifier();

        assertEquals(TerrainType.WALKABLE, classifier.classifyAnchored(
                world, 0, 1, 0, NARROW, MobTraversalProfile.DEFAULT));
        assertEquals(TerrainType.DAMAGING, classifier.classifyAnchored(
                world, 0, 1, 0, NARROW, naming(CUSTOM, TerrainType.DAMAGING)));
        assertEquals(TerrainType.FENCE, classifier.classifyAnchored(
                world, 0, 0, 0, NARROW, naming(CUSTOM, TerrainType.FENCE)),
                "select() prefers FENCE, so the diagonal rules see it too");
    }

    /** The static entry point stays the baseline mapping the parity suite pins. */
    @Test
    void theBuiltInMappingIsReachableWithoutAHook() {
        assertEquals(TerrainType.WALKABLE_DOOR, TerrainClassifier.raw(CUSTOM,
                block -> TerrainType.WALKABLE_DOOR));
        assertEquals(TerrainType.BLOCKED, TerrainClassifier.raw(CUSTOM));
        assertEquals(TerrainType.BLOCKED,
                TerrainClassifier.raw(CUSTOM, TerrainClassification.NONE));
        assertEquals(TerrainType.BLOCKED,
                TerrainClassifier.raw(CUSTOM, block -> null));
    }

    @Test
    void theHookReachesPlatformJumpLandings() {
        TestWorld world = new TestWorld()
                .set(0, 0, 0, Block.STONE)
                .set(3, 0, 0, CUSTOM);
        GroundCapabilities jumping = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(0).maxDrop(0).apexClearance(1).build());

        PathResult baseline = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(3.5, 1, 0.5), NARROW, MobTraversalProfile.DEFAULT, GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), jumping, SearchControl.NONE);
        PathResult named = pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(3.5, 1, 0.5), NARROW, naming(CUSTOM, TerrainType.DAMAGING), GroundSearchLimits.builder().maxPathLength(48).reachRange(0).maxVisitedMultiplier(8).build(), jumping, SearchControl.NONE);

        assertTrue(baseline.found(), baseline::toString);
        assertEquals(PathNode.Movement.JUMP,
                baseline.nodes().getLast().movement());
        assertFalse(named.found(), named::toString);
        assertTrue(named.nodes().stream().noneMatch(node ->
                node.movement() == PathNode.Movement.JUMP), named::toString);
    }

    @Test
    void theHookReachesTheFlyingEvaluator() {
        TestWorld tunnel = new TestWorld();
        for (int x = -3; x <= 11; x++) {
            for (int y = -2; y <= 4; y++) {
                for (int z = -2; z <= 2; z++) tunnel.set(x, y, z, Block.STONE);
            }
            if (x > -3 && x < 11) {
                tunnel.set(x, 0, 0, Block.AIR).set(x, 1, 0, Block.AIR);
            }
        }
        tunnel.set(4, 0, 0, CUSTOM).set(4, 1, 0, CUSTOM);

        PathResult plugged = fly(tunnel, MobTraversalProfile.DEFAULT);
        PathResult named = fly(tunnel, naming(CUSTOM, TerrainType.OPEN));

        assertFalse(plugged.found(), plugged::toString);
        assertTrue(named.found(), named::toString);
        assertTrue(named.nodes().stream()
                        .anyMatch(node -> node.graphX() == 4), named::toString);
    }

    @Test
    void theHookReachesTheSwimmingEvaluator() {
        TestWorld pool = new TestWorld();
        for (int x = -1; x <= 9; x++) {
            for (int y = 0; y <= 3; y++) {
                for (int z = -1; z <= 1; z++) pool.set(x, y, z, Block.WATER);
            }
        }
        for (int y = 0; y <= 3; y++) {
            for (int z = -1; z <= 1; z++) pool.set(4, y, z, CUSTOM);
        }

        assertFalse(swim(pool, MobTraversalProfile.DEFAULT).found(),
                "an unnamed wool wall dams the channel");
        PathResult named = swim(pool, naming(CUSTOM, TerrainType.WATER));

        assertTrue(named.found(), named::toString);
        assertTrue(named.nodes().stream()
                        .anyMatch(node -> node.graphX() == 4), named::toString);
    }

    /**
     * The node cache fixes a cell's type on first use, and the search-scoped
     * memo fixes a block state's type across cells. One search therefore only
     * ever sees one answer per block, whatever the hook does afterwards.
     */
    @Test
    void oneSearchAsksAHookOnceForEachDistinctBlockState() {
        TestWorld world = corridorPavedWith(CUSTOM);
        List<Block> asked = new ArrayList<>();
        MobTraversalProfile flipping = MobTraversalProfile.builder("flipping")
                .classification(block -> {
                    asked.add(block);
                    return asked.size() % 2 == 0 ? TerrainType.DAMAGING : null;
                })
                .build();

        PathResult result = walk(world, flipping);

        assertNotEquals(PathStatus.INVALID_REQUEST, result.status());
        assertFalse(asked.isEmpty(), "the hook has to be reached at all");
        assertEquals(new HashSet<>(asked).size(), asked.size(),
                "a block state asked twice in one search could answer twice");
    }

    /**
     * One profile is shared by every mob using it, while the memo that fixes
     * its answers belongs to a single search. Nothing is therefore written
     * across threads.
     */
    @Test
    void aSharedHookedProfileIsSafeAcrossConcurrentSearches() {
        TestWorld world = corridorPavedWith(CUSTOM);
        MobTraversalProfile shared = MobTraversalProfile.builder("shared")
                .malus(TerrainType.DAMAGING, 4)
                .classification(block ->
                        block.compare(CUSTOM) ? TerrainType.DAMAGING : null)
                .build();

        List<PathResult> results = IntStream.range(0, 256).parallel()
                .mapToObj(ignored -> walk(world, shared)).toList();

        PathResult first = results.getFirst();
        assertTrue(first.found(), first::toString);
        assertTrue(results.stream().allMatch(result ->
                        result.nodes().equals(first.nodes())
                                && result.nodeCosts().equals(first.nodeCosts())),
                "concurrent searches sharing one profile must agree");
    }

    @Test
    void aBuilderCarriesTheHookThroughEveryDerivedProfile() {
        TerrainClassification hook = block -> null;
        MobTraversalProfile base = MobTraversalProfile.builder("base")
                .classification(hook).build();
        MobTraversalProfile derived = MobTraversalProfile.builder("derived")
                .from(base).malus(TerrainType.WATER, 0).build();

        assertSame(hook, base.classification());
        assertSame(hook, derived.classification(),
                "the request factory rebuilds a profile with from() on every "
                        + "search, so a dropped hook would never be applied");
    }

    @Test
    void requestModifiersPreserveTheHook() {
        TerrainClassification hook = block -> null;
        NavigationProfile profile = ground().withMobProfile(
                MobTraversalProfile.builder("modified")
                        .from(MobTraversalProfile.DEFAULT)
                        .classification(hook).build());

        NavigationProfile modified = NavigationModifiers.builder()
                .terrainCost(TerrainType.WATER, 0).canFloat(true).build()
                .applyTo(profile);

        assertSame(hook, modified.mobProfile().classification());
    }

    @Test
    void profilesWithDifferentHooksNeverCompareEqual() {
        assertNotEquals(
                MobTraversalProfile.builder("m").classification(b -> null).build(),
                MobTraversalProfile.builder("m").classification(b -> null).build(),
                "two lambdas are two classes, so neither can inherit the "
                        + "other's plan");
        assertNotEquals(MobTraversalProfile.DEFAULT,
                MobTraversalProfile.builder("default")
                        .from(MobTraversalProfile.DEFAULT)
                        .classification(b -> null).build());
    }

    @Test
    void oneHookInstanceSharedByTwoProfilesComparesEqual() {
        TerrainClassification hook = block -> null;

        assertEquals(MobTraversalProfile.builder("m").classification(hook).build(),
                MobTraversalProfile.builder("m").classification(hook).build());
        assertEquals(
                MobTraversalProfile.builder("m").classification(hook).build().hashCode(),
                MobTraversalProfile.builder("m").classification(hook).build().hashCode());
    }

    @Test
    void aDeclaredKeyDecidesSharingWithoutTouchingEquals() {
        assertNotEquals(new Modded(1), new Modded(1));

        assertEquals(profileWith(new Modded(1)), profileWith(new Modded(1)));
        assertEquals(profileWith(new Modded(1)).hashCode(),
                profileWith(new Modded(1)).hashCode());
        assertNotEquals(profileWith(new Modded(1)), profileWith(new Modded(2)));
        assertEquals(profileWith(keyed(1)), profileWith(keyed(1)));
        assertNotEquals(profileWith(keyed(1)), profileWith(keyed(2)));
    }

    /**
     * The key is compared alongside the implementation class, so a hook that
     * names a value another one already uses stays on its own plan rather than
     * inheriting a route built under other terrain semantics.
     */
    @Test
    void equalKeysDeclaredByDifferentImplementationsStayApart() {
        Object shared = new ModdedKey(1);
        TerrainClassification damaging = TerrainClassification.keyed(
                block -> block.compare(CUSTOM) ? TerrainType.DAMAGING : null,
                shared);
        TerrainClassification lava = TerrainClassification.keyed(
                block -> block.compare(CUSTOM) ? TerrainType.LAVA : null,
                shared);

        assertNotEquals(profileWith(damaging), profileWith(lava));
    }

    @Test
    void requestsWithDifferentHooksNeverShareARegion() {
        assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                promoteThirdActor(() -> keyed(1), () -> keyed(1),
                        () -> keyed(2)),
                "a shared region would serve a plan computed under different "
                        + "terrain semantics");
        assertEquals(SharedMeshSource.INDIVIDUAL_SEARCH,
                promoteThirdActor(() -> block -> null, () -> block -> null,
                        () -> block -> null),
                "bare lambdas compare by identity and stay apart");
    }

    @Test
    void requestsWithEqualKeyedHooksReachOneSharedRegion() {
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                promoteThirdActor(() -> keyed(1), () -> keyed(1),
                        () -> keyed(1)));
        assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                promoteThirdActor(() -> TerrainClassification.NONE,
                        () -> TerrainClassification.NONE,
                        () -> TerrainClassification.NONE),
                "an absent hook must not cost the sharing that already worked");
    }

    /** One cell wide, so the only route is over the paving. */
    private static TestWorld corridorPavedWith(Block block) {
        TestWorld world = new TestWorld().floor(-1, 9, 0, 0, 0, Block.STONE);
        for (int x = 2; x <= 6; x++) world.set(x, 0, 0, block);
        return world;
    }

    private PathResult walk(TestWorld world, MobTraversalProfile profile) {
        return pathfinder.findPath(world, new Pos(0.5, 1, 0.5), new Pos(8.5, 1, 0.5), NARROW, profile, GroundSearchLimits.builder().maxPathLength(64).reachRange(0).maxVisitedMultiplier(8).build(), GroundCapabilities.STANDARD, SearchControl.NONE);
    }

    private static PathResult fly(TestWorld world, MobTraversalProfile mob) {
        return new EntityPathfinder().findPath(NavigationRequest.builder(world,
                        new Vec(0.5, 0.5, 0.5), new Vec(8.5, 0.5, 0.5),
                        new BoundingBox(0.6, 0.6, 0.6),
                        NavigationProfile.builder(NavigationMode.FLYING, mob, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build())
                .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
    }

    private static PathResult swim(TestWorld world, MobTraversalProfile mob) {
        return new EntityPathfinder().findPath(NavigationRequest.builder(world,
                        new Vec(0.5, 1.5, 0.5), new Vec(8.5, 1.5, 0.5),
                        new BoundingBox(0.6, 0.6, 0.6),
                        NavigationProfile.builder(NavigationMode.WATER, mob, GroundCapabilities.STANDARD).allowBreaching(false).prefersShallowWater(false).avoidSun(false).build())
                .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
    }

    private static MobTraversalProfile naming(Block block, TerrainType type) {
        return MobTraversalProfile.builder("naming")
                .classification(candidate -> candidate.compare(block) ? type : null)
                .build();
    }

    private static MobTraversalProfile profileWith(
            TerrainClassification classification) {
        return MobTraversalProfile.builder("modded")
                .classification(classification).build();
    }

    private static TerrainClassification keyed(int revision) {
        return TerrainClassification.keyed(
                block -> block.compare(CUSTOM) ? TerrainType.DAMAGING : null,
                new ModdedKey(revision));
    }

    private record ModdedKey(int revision) {}

    /** Declares a sharing key while keeping the inherited identity equality. */
    private static final class Modded implements TerrainClassification {
        private final int revision;

        Modded(int revision) {
            this.revision = revision;
        }

        @Override
        public TerrainType classify(Block block) {
            return block.compare(CUSTOM) ? TerrainType.DAMAGING : null;
        }

        @Override
        public Object equalityKey() {
            return new ModdedKey(revision);
        }
    }

    private static NavigationProfile ground() {
        return BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
    }

    private static SharedMeshSource promoteThirdActor(
            Supplier<TerrainClassification> first,
            Supplier<TerrainClassification> second,
            Supplier<TerrainClassification> third) {
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator(
                        (actor, request) -> plan(request),
                        SharedMeshPolicy.builder()
                                .promotionRequests(2).retentionRequests(1)
                                .build());
        TestWorld world = new TestWorld();
        try {
            coordinator.request("seed-1", "target", 1, 0,
                    classified(world, first.get())).plan().join();
            coordinator.request("seed-2", "target", 1, 1,
                    classified(world, second.get())).plan().join();
            return coordinator.request("third", "target", 1, 2,
                    classified(world, third.get())).source();
        } finally {
            coordinator.close();
        }
    }

    private static NavigationRequest classified(
            TestWorld world, TerrainClassification classification) {
        NavigationProfile profile = ground().withMobProfile(
                MobTraversalProfile.builder("modded")
                        .from(ground().mobProfile())
                        .classification(classification).build());
        return NavigationRequest.builder(world, new Pos(1.5, 1, 1.5),
                        new Pos(10.5, 1, 1.5), NARROW, profile)
                .maxPathLength(128).nodeSearchRange(128).build();
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
