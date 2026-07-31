package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.GroundCapabilities;import ca.atlasengine.pathfinding.profile.NavigationProfile;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.EntityTraversalState;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.adaptive.SharedMeshRetention;
import ca.atlasengine.pathfinding.adaptive.SharedMeshPolicy;
import ca.atlasengine.pathfinding.internal.adaptive.SharedMeshCoordinator;
import ca.atlasengine.pathfinding.adaptive.SharedMeshHandle;
import ca.atlasengine.pathfinding.adaptive.SharedMeshSource;
import ca.atlasengine.pathfinding.NavigationPlan;
import ca.atlasengine.pathfinding.internal.adaptive.NavigationPlanParity;
import ca.atlasengine.pathfinding.influence.AllowedNavigationAreas;
import ca.atlasengine.pathfinding.influence.BlockAvoidanceInfluence;
import ca.atlasengine.pathfinding.influence.EntityFearInfluence;
import ca.atlasengine.pathfinding.influence.EntitySnapshot;
import ca.atlasengine.pathfinding.influence.InfluenceResult;
import ca.atlasengine.pathfinding.influence.NavigationArea;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.influence.ReturnRadiusInfluence;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Randomized differential between the shared route mesh and ordinary
 * single-entity A*.
 *
 * <p>Every scenario drives enough compatible traffic through the coordinator
 * to promote its region, then re-runs a fresh {@link EntityPathfinder} search
 * for each request that was answered from a published shared plan. The two
 * must be semantically identical. Scenario seeds are derived from
 * {@code -Dpathfinding.meshParitySeed} so any failure replays exactly; the
 * sweep width is {@code -Dpathfinding.meshParityScenarios}.</p>
 */
class AdaptiveMeshDifferentialTest {
    private static final String SEED_PROPERTY = "pathfinding.meshParitySeed";
    private static final String SCENARIO_PROPERTY =
            "pathfinding.meshParityScenarios";
    private static final String PURSUIT_PROPERTY =
            "pathfinding.meshPursuitScenarios";
    private static final String TOGGLE_PROPERTY =
            "pathfinding.meshToggleScenarios";
    private static final long DEFAULT_SEED = 0x5A17_C0DE_BEEFL;
    private static final int DEFAULT_SCENARIOS = 48;
    private static final int DEFAULT_PURSUIT_SCENARIOS = 12;
    private static final int DEFAULT_TOGGLE_SCENARIOS = 8;

    private static final int SPAN = 20;
    private static final int ACTORS = 4;
    private static final int TARGETS = 2;
    private static final int ROUNDS = 4;

    private static final int PURSUERS = 8;
    private static final int PURSUIT_ROUNDS = 12;
    private static final int PURSUIT_TARGET_CELLS = 4;

    private static final int TOGGLE_ROUNDS = 12;
    /** Room for one region per traversal variant, so nothing is squeezed. */
    private static final int TOGGLE_REGIONS = 8;
    /** Half that, so a key that grows per variant must starve someone. */
    private static final int SQUEEZED_REGIONS = 2;

    @Test
    void publishedSharedPlansEqualIndividualPathfindingAcrossRandomWorlds() {
        long baseSeed = Long.getLong(SEED_PROPERTY, DEFAULT_SEED);
        int scenarios = Integer.getInteger(
                SCENARIO_PROPERTY, DEFAULT_SCENARIOS);
        Totals totals = new Totals();
        for (int index = 0; index < scenarios; index++) {
            long seed = baseSeed + index;
            try {
                runScenario(seed, totals);
            } catch (AssertionError | RuntimeException failure) {
                throw new AssertionError("mesh parity differential failed; "
                        + "reproduce with -D" + SEED_PROPERTY + "=" + seed
                        + " -D" + SCENARIO_PROPERTY + "=1", failure);
            }
        }

        assertTrue(totals.promotedScenarios >= scenarios * 3 / 4,
                () -> "regions promoted in only " + totals.promotedScenarios
                        + " of " + scenarios + " scenarios; the differential "
                        + "never reached the shared-mesh state");
        assertTrue(totals.sharedPlans >= scenarios * 4,
                () -> "only " + totals.sharedPlans + " shared plans were "
                        + "published across " + scenarios + " scenarios");
        assertTrue(totals.movements.size() >= 6,
                () -> "compared plans covered only " + totals.movements);
        assertTrue(totals.freshInfluencePromoted
                        >= totals.freshInfluenceScenarios * 3 / 4,
                () -> "regions promoted in only "
                        + totals.freshInfluencePromoted + " of "
                        + totals.freshInfluenceScenarios + " scenarios that "
                        + "rebuild an equal influence per request; influence "
                        + "value equality is not reaching the region key");
        System.out.println("adaptive mesh differential: seed=" + baseSeed
                + " worlds=" + scenarios
                + " requests=" + totals.requests
                + " promotedScenarios=" + totals.promotedScenarios
                + " retainedMeshNodes=" + totals.retainedNodes
                + " sharedPlansCompared=" + totals.sharedPlans
                + " freshInfluenceScenarios=" + totals.freshInfluenceScenarios
                + " freshInfluencePromoted=" + totals.freshInfluencePromoted
                + " freshInfluenceSharedPlans="
                + totals.freshInfluenceSharedPlans
                + " movements=" + totals.movements);
    }

    /**
     * The pursuit half of the sweep: one target key that keeps moving, and a
     * swarm of followers that either hold station or move with it. Every
     * entity position is a live continuous double, so nothing repeats bit for
     * bit except an actor that has genuinely not moved.
     *
     * <p>A destination is merged to its search-observable cell, so a target
     * that jitters or re-enters a cell reuses one field instead of allocating
     * a node per position. A source is never merged, so a follower that has
     * moved always pays its own A*; it must still never be forced out of the
     * region into untracked planning by another entity's movement.</p>
     */
    @Test
    void pursuitTrafficReusesTargetFieldsWithoutDivergingFromIndividualSearch() {
        long baseSeed = Long.getLong(SEED_PROPERTY, DEFAULT_SEED);
        int scenarios = Integer.getInteger(
                PURSUIT_PROPERTY, DEFAULT_PURSUIT_SCENARIOS);
        Pursuit engaged = new Pursuit("engaged");
        Pursuit chasing = new Pursuit("chasing");
        for (int index = 0; index < scenarios; index++) {
            long seed = baseSeed + 0x7000 + index;
            try {
                runPursuit(seed, false, engaged);
                runPursuit(seed, true, chasing);
            } catch (AssertionError | RuntimeException failure) {
                throw new AssertionError("pursuit differential failed; "
                        + "reproduce with -D" + SEED_PROPERTY + "="
                        + (seed - 0x7000) + " -D" + PURSUIT_PROPERTY + "=1",
                        failure);
            }
        }
        System.out.println("adaptive mesh pursuit differential: seed="
                + baseSeed + " worlds=" + (scenarios * 2)
                + " " + engaged + " " + chasing);

        assertEquals(0, engaged.untracked + chasing.untracked,
                () -> "a moving target pushed pursuers out of their region "
                        + "into untracked A*: " + engaged + " " + chasing);
        assertTrue(engaged.shared >= engaged.requests / 4,
                () -> "a stationary pursuer never reused a moving target's "
                        + "field: " + engaged);
        assertTrue(engaged.compared >= engaged.shared,
                () -> "unverified shared plans: " + engaged);
        assertTrue(chasing.certification >= chasing.requests / 4,
                () -> "a moving pursuer must keep certifying its own exact "
                        + "source inside the region: " + chasing);
    }

    /**
     * The workload the split source key exists for: a population that holds
     * its stations while {@code onGround} and {@code inWater} toggle beneath
     * it, as one mob does every time it jumps or wades in.
     *
     * <p>Nothing the graph is built from changes, so every variant belongs in
     * one region. The start node each variant resolves does change, and the
     * parity gate cannot see it: {@code requestCompatible} compares the raw
     * request start, which is bit-identical across a toggle. Separation must
     * therefore come from the source key, and the sweep proves it by
     * re-running individual A* behind every plan a variant was served.</p>
     *
     * <p>Half the stations plan from the air, where a grounded snapshot rounds
     * to the cell the actor occupies and an airborne one walks down to the
     * ground, so the two genuinely resolve different start nodes from the same
     * exact position. The pinned run repeats the identical traffic with one
     * snapshot, and the squeezed run repeats it against a region budget too
     * small to give every variant its own.</p>
     */
    @Test
    void togglingTraversalStateSharesOneRegionWithoutDivergingFromIndividualSearch() {
        long baseSeed = Long.getLong(SEED_PROPERTY, DEFAULT_SEED);
        int scenarios = Integer.getInteger(
                TOGGLE_PROPERTY, DEFAULT_TOGGLE_SCENARIOS);
        Toggle toggling = new Toggle("toggling");
        Toggle pinned = new Toggle("pinned");
        Toggle squeezed = new Toggle("squeezed");
        for (int index = 0; index < scenarios; index++) {
            long seed = baseSeed + 0xB000 + index;
            try {
                runToggle(seed, true, TOGGLE_REGIONS, toggling);
                runToggle(seed, false, TOGGLE_REGIONS, pinned);
                runToggle(seed, true, SQUEEZED_REGIONS, squeezed);
            } catch (RuntimeException failure) {
                throw new AssertionError("traversal-toggle differential "
                        + "failed; reproduce with -D" + SEED_PROPERTY + "="
                        + (seed - 0xB000) + " -D" + TOGGLE_PROPERTY + "=1",
                        failure);
            }
        }
        System.out.println("adaptive mesh traversal-toggle differential: seed="
                + baseSeed + " worlds=" + scenarios + " " + toggling
                + " " + pinned + " " + squeezed);

        assertEquals(0, toggling.divergences,
                () -> "a toggled traversal snapshot was answered with another "
                        + "snapshot's route: " + toggling.firstDivergence);
        assertEquals(0, pinned.divergences,
                () -> "the pinned control diverged: "
                        + pinned.firstDivergence);
        assertEquals(0, squeezed.divergences,
                () -> "the squeezed run diverged: "
                        + squeezed.firstDivergence);
        assertEquals(pinned.regions, toggling.regions,
                () -> "a mob that jumps or wades must occupy exactly as many "
                        + "regions as one that does not: " + toggling
                        + " against " + pinned);
        assertEquals(0, squeezed.untracked,
                () -> "toggling traversal state exhausted a region budget and "
                        + "degraded requests to untracked A*: " + squeezed);
        assertEquals(toggling.regions, squeezed.regions,
                () -> "a quarter of the region budget still has to be enough: "
                        + squeezed + " against " + toggling);
        assertEquals(toggling.shared, squeezed.shared,
                () -> "reuse fell away once the region budget tightened: "
                        + squeezed + " against " + toggling);
        assertTrue(toggling.shared >= pinned.shared / 2,
                () -> "sharing one region bought no reuse for a toggling "
                        + "population: " + toggling + " against " + pinned);
        assertTrue(toggling.compared >= toggling.shared,
                () -> "unverified shared plans: " + toggling);
    }

    private static void runToggle(
            long seed, boolean toggling, int maximumRegions, Toggle out) {
        Random random = new Random(seed);
        Scenario scenario = scenario(random);
        EntityPathfinder oracle = new EntityPathfinder();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) ->
                        CompletableFuture.completedFuture(
                                NavigationPlan.from(submitted, oracle.findPath(
                                        submitted, SearchControl.NONE))),
                        SharedMeshPolicy.builder()
                                .promotionRequests(2 + random.nextInt(3))
                                .retentionRequests(1)
                                .maximumRegions(maximumRegions)
                                .build());
        List<Pos> chosen = pick(
                new ArrayList<>(scenario.candidates), ACTORS, random);
        // One footprint for the whole population, so the traversal snapshot is
        // the only request property left that can partition a region.
        Actor shape = scenario.actors.getFirst();
        List<Point> stations = new ArrayList<>();
        for (int index = 0; index < chosen.size(); index++) {
            Pos station = chosen.get(index);
            stations.add(index % 2 == 0
                    ? station : station.add(0, 2 + random.nextInt(3), 0));
        }
        List<SharedMeshHandle> handles = new ArrayList<>();
        long tick = 0;
        try {
            for (int round = 0; round < TOGGLE_ROUNDS; round++) {
                EntityTraversalState state = toggling
                        ? toggled(scenario.state, round) : scenario.state;
                for (int index = 0; index < stations.size(); index++) {
                    Point start = stations.get(index);
                    for (int target = 0;
                         target < scenario.targets.size(); target++) {
                        NavigationRequest request = scenario.request(shape,
                                start, scenario.targets.get(target), state);
                        SharedMeshHandle handle = coordinator.request(
                                "toggler-" + index + '-' + target,
                                "target-" + target, 1, tick++, request);
                        handles.add(handle);
                        out.verify(handle, request, oracle);
                    }
                }
            }
            out.regions += coordinator.stats().regions();
        } finally {
            handles.forEach(SharedMeshHandle::close);
            coordinator.close();
        }
    }

    /** The two live flags flipped through their four combinations. */
    private static EntityTraversalState toggled(
            EntityTraversalState base, int round) {
        return new EntityTraversalState(
                (round & 1) == 0 ? base.onGround() : !base.onGround(),
                (round & 2) == 0 ? base.inWater() : !base.inWater(),
                base.standableFluids(), base.minBuildHeight(),
                base.maxBuildHeight(), base.pathfindingStartCandidates());
    }

    private static void runPursuit(
            long seed, boolean pursuersMove, Pursuit pursuit) {
        Random random = new Random(seed);
        Scenario scenario = scenario(random);
        EntityPathfinder oracle = new EntityPathfinder();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) ->
                        CompletableFuture.completedFuture(
                                NavigationPlan.from(submitted, oracle.findPath(
                                        submitted, SearchControl.NONE))),
                        SharedMeshPolicy.builder()
                                .promotionRequests(2 + random.nextInt(3))
                                .retentionRequests(1)
                                // Room for twice as many destinations as the
                                // target ever occupies. Only a key that grows
                                // with every position can overflow it.
                                .maximumTargetsPerRegion(
                                        PURSUIT_TARGET_CELLS * 2)
                                .build());
        List<Pos> cells = pick(new ArrayList<>(scenario.candidates),
                PURSUIT_TARGET_CELLS, random);
        List<Pos> stations = pick(new ArrayList<>(scenario.candidates),
                PURSUERS, random);
        List<SharedMeshHandle> handles = new ArrayList<>();
        long tick = 0;
        try {
            for (int round = 0; round < PURSUIT_ROUNDS; round++) {
                Point target = jitter(cells.get(round % cells.size()), random);
                for (int index = 0; index < stations.size(); index++) {
                    Pos station = stations.get(index);
                    Point start = pursuersMove
                            ? jitter(station, random) : station;
                    NavigationRequest request = scenario.pursuitRequest(
                            start, target, index);
                    SharedMeshHandle handle = coordinator.request(
                            "pursuer-" + index, "player", 1, tick++, request);
                    handles.add(handle);
                    pursuit.record(handle.source());
                    if (verifyShared(handle, request, oracle,
                            pursuit.movements)) pursuit.compared++;
                }
            }
            pursuit.retainedTargets += coordinator.stats().targets();
        } finally {
            handles.forEach(SharedMeshHandle::close);
            coordinator.close();
        }
    }

    /**
     * A lambda influence carries identity equality only, so it can merge two
     * requests exclusively through a declared key. The sweep drives real A*
     * through such a region and re-verifies every published plan, and checks
     * that a recipe capturing a different toll is never answered from it.
     */
    @Test
    void keyedLambdaInfluencesShareOneRegionAndStillMatchIndividualSearch() {
        long baseSeed = Long.getLong(SEED_PROPERTY, DEFAULT_SEED);
        Set<PathNode.Movement> movements =
                EnumSet.noneOf(PathNode.Movement.class);
        int shared = 0;
        for (int index = 0; index < 8; index++) {
            Random random = new Random(baseSeed + 0x9000 + index);
            Scenario base = scenario(random);
            double penalty = 2 + random.nextInt(4);
            Point centre = new Vec(SPAN / 2.0, 1, SPAN / 2.0);
            Pos target = base.targets.getFirst();
            EntityPathfinder oracle = new EntityPathfinder();
            SharedMeshCoordinator coordinator =
                    new SharedMeshCoordinator((actor, submitted) ->
                            CompletableFuture.completedFuture(
                                    NavigationPlan.from(submitted,
                                            oracle.findPath(submitted,
                                                    SearchControl.NONE))),
                            SharedMeshPolicy.builder()
                                    .promotionRequests(2).retentionRequests(1)
                                    .build());
            List<SharedMeshHandle> handles = new ArrayList<>();
            long tick = 0;
            try {
                for (int round = 0; round < ROUNDS; round++) {
                    for (Actor actor : base.actors) {
                        NavigationRequest request = base.tolled(
                                actor, target, centre, penalty);
                        SharedMeshHandle handle = coordinator.request(
                                actor.key, "target", 1, tick++, request);
                        handles.add(handle);
                        if (verifyShared(handle, request, oracle, movements)) {
                            shared++;
                        }
                    }
                }
                Actor first = base.actors.getFirst();
                SharedMeshHandle other = coordinator.request(
                        "different-toll", "target", 1, tick,
                        base.tolled(first, target, centre, penalty + 1));
                handles.add(other);
                assertNotEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                        other.source(),
                        "a lambda that captured a different toll was answered "
                                + "from a region keyed on the first one");
            } finally {
                handles.forEach(SharedMeshHandle::close);
                coordinator.close();
            }
        }
        assertTrue(shared > 0,
                "keyed lambda influences never merged into one region");
        System.out.println("adaptive mesh keyed-influence differential: "
                + "sharedPlansCompared=" + shared
                + " movements=" + movements);
    }

    /**
     * A lambda cannot carry value equality, so the key names everything it
     * captured.
     */
    private static NavigationInfluence toll(Point centre, double penalty) {
        return NavigationInfluence.keyed(
                (blocks, point, box) -> point.distance(centre) > 6
                        ? InfluenceResult.penalty(penalty, "toll")
                        : InfluenceResult.NONE,
                List.of("toll", centre, penalty));
    }

    /** A live sub-block offset that never leaves the entity's own cell. */
    private static Point jitter(Pos cell, Random random) {
        return new Vec(cell.blockX() + 0.2 + random.nextDouble() * 0.6,
                cell.y(),
                cell.blockZ() + 0.2 + random.nextDouble() * 0.6);
    }

    /**
     * Minimized from the sweep. Two footprints of identical width, height, and
     * depth still search different graphs when their anchors differ, because
     * the evaluators read the box origin. Neither actor may be answered with
     * the other's certified plan, and mixing them may not break planning.
     */
    @Test
    void equalSizedFootprintsWithDifferentAnchorsNeverShareCertifiedPlans() {
        TestWorld world = new TestWorld().floor(0, 12, -3, 3, 0, Block.STONE);
        NavigationProfile profile =
                BuiltinNavigationProfiles.forEntityType(EntityType.ZOMBIE);
        BoundingBox centered = new BoundingBox(0.6, 1.8, 0.6);
        BoundingBox anchored =
                new BoundingBox(0.6, 1.8, 0.6, new Vec(0, 0, 0));
        assertEquals(centered.width(), anchored.width());
        assertEquals(centered.height(), anchored.height());
        assertEquals(centered.depth(), anchored.depth());

        EntityPathfinder pathfinder = new EntityPathfinder();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) ->
                        CompletableFuture.completedFuture(
                                NavigationPlan.from(submitted,
                                        pathfinder.findPath(submitted,
                                                SearchControl.NONE))),
                        SharedMeshPolicy.builder()
                                .promotionRequests(2).retentionRequests(1)
                                .build());
        Pos start = new Pos(0.5, 1, 0.5);
        Pos target = new Pos(10.5, 1, 0.5);
        NavigationRequest first = request(world, start, target,
                centered, profile);
        NavigationRequest second = request(world, start, target,
                anchored, profile);
        try {
            coordinator.request("a", "t", 1, 0, first).plan().join();
            coordinator.request("b", "t", 1, 1, first).plan().join();
            SharedMeshHandle shared =
                    coordinator.request("c", "t", 1, 2, first);
            assertEquals(SharedMeshSource.SHARED_TARGET_FIELD,
                    shared.source());

            SharedMeshHandle other =
                    coordinator.request("d", "t", 1, 3, second);
            NavigationPlan plan = other.plan().join();
            assertEquals(anchored, plan.boundingBox(),
                    "an actor was handed a plan certified for another "
                            + "footprint anchor");
            assertTrue(NavigationPlanParity.semanticallyEquivalent(
                    NavigationPlan.from(second, pathfinder.findPath(
                            second, SearchControl.NONE)), plan));

            NavigationRequest elsewhere = request(world, start,
                    new Pos(9.5, 1, 2.5), anchored, profile);
            coordinator.request("e", "t", 1, 4, elsewhere).plan().join();
            coordinator.request("f", "t", 1, 5, first).plan().join();
        } finally {
            coordinator.close();
        }
    }

    private static NavigationRequest request(
            TestWorld world, Pos start, Pos target, BoundingBox box,
            NavigationProfile profile) {
        return NavigationRequest.builder(world, start, target, box, profile)
                .maxPathLength(48).nodeSearchRange(48)
                .maxVisitedMultiplier(4).build();
    }

    private static void runScenario(long seed, Totals totals) {
        Random random = new Random(seed);
        Scenario scenario = scenario(random);
        int sharedPlansBefore = totals.sharedPlans;
        if (scenario.freshInfluences) totals.freshInfluenceScenarios++;
        EntityPathfinder oracle = new EntityPathfinder();
        AtomicInteger searches = new AtomicInteger();
        SharedMeshPolicy config = SharedMeshPolicy.builder()
                .promotionRequests(2 + random.nextInt(3))
                .retentionRequests(1)
                .build();
        SharedMeshCoordinator coordinator =
                new SharedMeshCoordinator((actor, submitted) -> {
                    searches.incrementAndGet();
                    return CompletableFuture.completedFuture(
                            NavigationPlan.from(submitted, oracle.findPath(
                                    submitted, SearchControl.NONE)));
                }, config);
        List<SharedMeshHandle> handles = new ArrayList<>();
        long tick = 0;
        try {
            for (int round = 0; round < ROUNDS; round++) {
                for (Actor actor : scenario.actors) {
                    for (int target = 0;
                         target < scenario.targets.size(); target++) {
                        NavigationRequest request = scenario.request(
                                actor, scenario.targets.get(target));
                        SharedMeshHandle handle = coordinator.request(
                                actor.key + '-' + target,
                                "target-" + target, 1, tick++, request);
                        handles.add(handle);
                        totals.requests++;
                        compare(handle, request, oracle, totals);
                    }
                }
            }
            SharedMeshRetention stats =
                    coordinator.stats();
            if (stats.promotedRegions() > 0) totals.promotedScenarios++;
            if (scenario.freshInfluences) {
                if (stats.promotedRegions() > 0) totals.freshInfluencePromoted++;
                totals.freshInfluenceSharedPlans +=
                        totals.sharedPlans - sharedPlansBefore;
            }
            totals.retainedNodes += stats.retainedNodes();
            assertTrue(searches.get() <= totals.requests,
                    "adaptive planning issued more searches than requests");
        } finally {
            handles.forEach(SharedMeshHandle::close);
            coordinator.close();
        }
    }

    private static void compare(
            SharedMeshHandle handle, NavigationRequest request,
            EntityPathfinder oracle, Totals totals) {
        if (verifyShared(handle, request, oracle, totals.movements)) {
            totals.sharedPlans++;
        }
    }

    private static boolean verifyShared(
            SharedMeshHandle handle, NavigationRequest request,
            EntityPathfinder oracle, Set<PathNode.Movement> movements) {
        NavigationPlan plan;
        try {
            plan = handle.plan().join();
        } catch (CompletionException failure) {
            throw new AssertionError("adaptive planning failed for a request "
                    + "that ordinary A* answers (source=" + handle.source()
                    + ')', failure);
        }
        if (handle.source() != SharedMeshSource.SHARED_TARGET_FIELD) {
            return false;
        }
        NavigationPlan individual = NavigationPlan.from(
                request, oracle.findPath(request, SearchControl.NONE));
        assertTrue(NavigationPlanParity.requestCompatible(request, plan),
                () -> "shared plan is not compatible with the request it "
                        + "answered: " + describe(request, individual, plan));
        assertTrue(NavigationPlanParity.semanticallyEquivalent(
                        individual, plan),
                () -> "shared plan diverges from individual A*: "
                        + describe(request, individual, plan));
        assertEquals(individual.nodeCosts(), plan.nodeCosts(),
                () -> "shared plan prices its route differently from "
                        + "individual A*: " + describe(request, individual, plan));
        plan.nodes().forEach(node -> movements.add(node.movement()));
        return true;
    }

    private static String describe(
            NavigationRequest request, NavigationPlan individual,
            NavigationPlan shared) {
        return "start=" + request.start() + " target=" + request.target()
                + " box=" + request.boundingBox()
                + " mode=" + request.profile().mode()
                + "\n  individual status=" + individual.status()
                + " box=" + individual.boundingBox()
                + " nodes=" + individual.nodes()
                + "\n  shared     status=" + shared.status()
                + " box=" + shared.boundingBox()
                + " nodes=" + shared.nodes();
    }

    private static Scenario scenario(Random random) {
        Topology topology = Topology.values()[
                random.nextInt(Topology.values().length)];
        World world = world(topology, random);
        Family family = topology.families.get(
                random.nextInt(topology.families.size()));
        List<BoundingBox> boxes = boxes(family, random);
        List<Pos> positions = new ArrayList<>(world.candidates);
        List<Pos> starts = pick(positions, ACTORS, random);
        List<Pos> targets = pick(positions, TARGETS, random);
        List<Actor> actors = new ArrayList<>();
        for (int index = 0; index < starts.size(); index++) {
            actors.add(new Actor("actor-" + index, starts.get(index),
                    boxes.get(index % boxes.size()),
                    16 + random.nextInt(25), 1 + random.nextInt(3)));
        }
        Supplier<List<NavigationInfluence>> recipe = influences(world, random);
        // An integrator that rebuilds influences per entity hands the
        // coordinator objects that are equal but never identical.
        boolean fresh = random.nextBoolean() && !recipe.get().isEmpty();
        return new Scenario(world.blocks, family.profile(), family.state,
                fresh ? recipe : pinned(recipe), fresh, actors, targets,
                world.candidates);
    }

    private static List<Pos> pick(
            List<Pos> positions, int count, Random random) {
        List<Pos> chosen = new ArrayList<>();
        for (int index = 0; index < count && !positions.isEmpty(); index++) {
            chosen.add(positions.remove(random.nextInt(positions.size())));
        }
        return chosen;
    }

    /**
     * One to three actor footprints. Some scenarios give every actor the same
     * box so a single region collects the richest possible mesh; others mix
     * footprints that differ in size, in anchor, or in both.
     */
    private static List<BoundingBox> boxes(Family family, Random random) {
        List<BoundingBox> boxes = new ArrayList<>();
        boxes.add(box(family, random));
        if (random.nextBoolean()) {
            BoundingBox first = boxes.getFirst();
            boxes.add(random.nextBoolean()
                    ? new BoundingBox(first.width(), first.height(),
                            first.depth(), new Vec(0, 0, 0))
                    : box(family, random));
        }
        if (random.nextInt(4) == 0) boxes.add(box(family, random));
        return boxes;
    }

    private static BoundingBox box(Family family, Random random) {
        double width = switch (random.nextInt(5)) {
            case 0 -> 0.6;
            case 1 -> 0.7;
            case 2 -> 0.98;
            case 3 -> 1.4;
            default -> 0.4;
        };
        double height = family.volumetric
                ? width : 0.6 + random.nextInt(3) * 0.6;
        return new BoundingBox(width, height, width);
    }

    /**
     * A recipe rather than a value. Every random draw is made once, so each
     * call builds a list that is equal to the last one without sharing a
     * single object with it.
     */
    private static Supplier<List<NavigationInfluence>> influences(
            World world, Random random) {
        return switch (random.nextInt(6)) {
            case 1 -> {
                int radius = 1 + random.nextInt(3);
                boolean forbidden = random.nextBoolean();
                yield () -> List.of(new BlockAvoidanceInfluence(
                        Set.of(Block.FIRE, Block.LAVA, Block.CACTUS), radius,
                        forbidden, 4));
            }
            case 2 -> {
                boolean forbidden = random.nextBoolean();
                yield () -> List.of(new NavigationZoneInfluence(
                        new Vec(6, world.minY, 6), new Vec(11, world.maxY, 11),
                        forbidden, 6, "zone"));
            }
            case 3 -> {
                double radius = 8 + random.nextInt(16);
                yield () -> List.of(new ReturnRadiusInfluence(
                        new Vec(SPAN / 2.0, world.minY + 1, SPAN / 2.0),
                        new Vec(SPAN / 2.0, world.minY + 1, SPAN / 2.0),
                        radius));
            }
            case 4 -> () -> List.of(AllowedNavigationAreas.of(
                    new NavigationArea(new Vec(-1, world.minY - 2, -1),
                            new Vec(SPAN + 1, world.maxY + 2, SPAN + 1),
                            "field")));
            case 5 -> {
                UUID threat = new UUID(random.nextLong(), random.nextLong());
                double radius = 3 + random.nextInt(4);
                yield () -> List.of(new EntityFearInfluence(
                        List.of(new EntitySnapshot(threat, EntityType.CREEPER,
                                new Vec(9.5, world.minY + 1, 9.5))),
                        radius, 2, 12));
            }
            default -> List::of;
        };
    }

    private static Supplier<List<NavigationInfluence>> pinned(
            Supplier<List<NavigationInfluence>> recipe) {
        List<NavigationInfluence> influences = recipe.get();
        return () -> influences;
    }

    private static World world(Topology topology, Random random) {
        TestWorld world = new TestWorld();
        List<Pos> candidates = new ArrayList<>();
        switch (topology) {
            case OPEN -> {
                world.floor(0, SPAN - 1, 0, SPAN - 1, 0, Block.STONE);
                forEachCell((x, z) ->
                        candidates.add(new Pos(x + 0.5, 1, z + 0.5)));
            }
            case MAZE -> {
                world.floor(0, SPAN - 1, 0, SPAN - 1, 0, Block.STONE);
                boolean[][] wall = new boolean[SPAN][SPAN];
                for (int x = 0; x < SPAN; x++) {
                    for (int z = 0; z < SPAN; z++) {
                        wall[x][z] = (x % 2 == 1 || z % 2 == 1)
                                && random.nextInt(100) < 45;
                        if (wall[x][z]) world.column(x, z, 1, 3, Block.STONE);
                    }
                }
                forEachCell((x, z) -> {
                    if (!wall[x][z]) candidates.add(new Pos(x + 0.5, 1, z + 0.5));
                });
            }
            case RIDGE -> {
                int[][] height = new int[SPAN][SPAN];
                boolean[][] tower = new boolean[SPAN][SPAN];
                for (int x = 0; x < SPAN; x++) {
                    for (int z = 0; z < SPAN; z++) {
                        height[x][z] = random.nextInt(4);
                        world.column(x, z, 0, height[x][z], Block.STONE);
                    }
                }
                Block rung = Block.LADDER.withProperty("facing", "west");
                for (int index = 0; index < 10; index++) {
                    int x = 2 + random.nextInt(SPAN - 3);
                    int z = 1 + random.nextInt(SPAN - 2);
                    int top = height[x][z] + 3 + random.nextInt(4);
                    world.column(x, z, height[x][z] + 1, top, Block.STONE);
                    world.column(x - 1, z, height[x - 1][z] + 1, top, rung);
                    tower[x][z] = true;
                }
                forEachCell((x, z) -> {
                    if (!tower[x][z]) {
                        candidates.add(new Pos(
                                x + 0.5, height[x][z] + 1, z + 0.5));
                    }
                });
            }
            case WATER -> {
                world.floor(0, SPAN - 1, 0, SPAN - 1, 0, Block.STONE);
                for (int x = 0; x < SPAN; x++) {
                    for (int z = 0; z < SPAN; z++) {
                        world.column(x, z, 1, 6, Block.WATER);
                        if (random.nextInt(100) < 8) {
                            world.column(x, z, 1, 4 + random.nextInt(5),
                                    Block.STONE);
                        }
                    }
                }
                forEachCell((x, z) -> {
                    candidates.add(new Pos(x + 0.5, 4.5, z + 0.5));
                    candidates.add(new Pos(x + 0.5, 6.5, z + 0.5));
                });
            }
            case MIXED -> {
                world.floor(0, SPAN - 1, 0, SPAN - 1, 0, Block.STONE);
                for (int x = 0; x < SPAN; x++) {
                    for (int z = 0; z < SPAN; z++) {
                        Block feature = switch (random.nextInt(14)) {
                            case 0 -> Block.STONE_SLAB;
                            case 1 -> Block.OAK_STAIRS;
                            case 2 -> Block.OAK_FENCE;
                            case 3 -> Block.FIRE;
                            case 4 -> Block.WATER;
                            case 5 -> Block.STONE;
                            default -> null;
                        };
                        if (feature != null) world.set(x, 1, z, feature);
                        if (feature == Block.WATER) world.set(x, 2, z, feature);
                    }
                }
                forEachCell((x, z) ->
                        candidates.add(new Pos(x + 0.5, 1, z + 0.5)));
            }
        }
        return new World(world, candidates,
                topology == Topology.WATER ? 1 : 0,
                topology == Topology.WATER ? 6 : 4);
    }

    private static void forEachCell(CellVisitor visitor) {
        for (int x = 0; x < SPAN; x++) {
            for (int z = 0; z < SPAN; z++) visitor.visit(x, z);
        }
    }

    @FunctionalInterface
    private interface CellVisitor {
        void visit(int x, int z);
    }

    private record World(
            TestWorld blocks, List<Pos> candidates, int minY, int maxY) {
    }

    private record Actor(
            String key, Pos start, BoundingBox box, double pathLength,
            int visitedMultiplier) {
    }

    private record Scenario(
            TestWorld blocks, NavigationProfile profile,
            EntityTraversalState state,
            Supplier<List<NavigationInfluence>> influences,
            boolean freshInfluences, List<Actor> actors, List<Pos> targets,
            List<Pos> candidates) {
        NavigationRequest request(Actor actor, Point target) {
            return request(actor, actor.start, target);
        }

        NavigationRequest tolled(
                Actor actor, Point target, Point centre, double penalty) {
            return NavigationRequest.builder(
                            blocks, actor.start, target, actor.box, profile)
                    .maxPathLength(actor.pathLength)
                    .nodeSearchRange(actor.pathLength)
                    .maxVisitedMultiplier(actor.visitedMultiplier)
                    .influences(List.of(toll(centre, penalty)))
                    .entityState(state).build();
        }

        /** One member of a pursuing swarm, borrowing an actor's footprint. */
        NavigationRequest pursuitRequest(
                Point start, Point target, int index) {
            return request(actors.get(index % actors.size()), start, target);
        }

        /** The same request under a substituted live traversal snapshot. */
        NavigationRequest request(Actor actor, Point start, Point target,
                                  EntityTraversalState liveState) {
            return NavigationRequest.builder(
                            blocks, start, target, actor.box, profile)
                    .maxPathLength(actor.pathLength)
                    .nodeSearchRange(actor.pathLength)
                    .maxVisitedMultiplier(actor.visitedMultiplier)
                    .influences(influences.get())
                    .entityState(liveState).build();
        }

        private NavigationRequest request(
                Actor actor, Point start, Point target) {
            return request(actor, start, target, state);
        }
    }

    private record Family(
            EntityType type, boolean volumetric, EntityTraversalState state,
            Capability capability) {
        NavigationProfile profile() {
            NavigationProfile base =
                    BuiltinNavigationProfiles.forEntityType(type);
            return switch (capability) {
                case NONE -> base;
                case CLIMB -> withGround(base,
                        base.groundCapabilities().withClimbables(
                                ClimbableCapabilities.STANDARD));
                case JUMP -> withGround(base,
                        base.groundCapabilities().withPlatformJump(
                                PlatformJumpCapabilities.builder().maxHorizontalDistance(3).maxRise(1).maxDrop(1).apexClearance(1).build()));
            };
        }

        private static NavigationProfile withGround(
                NavigationProfile base, GroundCapabilities capabilities) {
            return NavigationProfile.builder(base.mode(), base.mobProfile(), capabilities).allowBreaching(base.allowBreaching()).prefersShallowWater(base.prefersShallowWater()).avoidSun(base.avoidSun()).directPathIgnoresFluids(base.directPathIgnoresFluids()).pathToTargetsBelowSurface(base.pathToTargetsBelowSurface()).build();
        }
    }

    private enum Capability {
        NONE, CLIMB, JUMP
    }

    private static final EntityTraversalState SUBMERGED =
            new EntityTraversalState(
                    false, true, Set.of(), -64, 319, List.of());

    private static Family ground(EntityType type, Capability capability) {
        return new Family(type, false, EntityTraversalState.GROUNDED,
                capability);
    }

    private enum Topology {
        OPEN(List.of(
                ground(EntityType.ZOMBIE, Capability.NONE),
                ground(EntityType.SPIDER, Capability.NONE),
                new Family(EntityType.BEE, true,
                        EntityTraversalState.GROUNDED, Capability.NONE))),
        MAZE(List.of(
                ground(EntityType.ZOMBIE, Capability.NONE),
                ground(EntityType.VILLAGER, Capability.NONE),
                ground(EntityType.SPIDER, Capability.JUMP),
                new Family(EntityType.BEE, true,
                        EntityTraversalState.GROUNDED, Capability.NONE))),
        RIDGE(List.of(
                ground(EntityType.ZOMBIE, Capability.NONE),
                ground(EntityType.ZOMBIE, Capability.CLIMB),
                ground(EntityType.COW, Capability.JUMP),
                ground(EntityType.WARDEN, Capability.NONE))),
        WATER(List.of(
                new Family(EntityType.COD, true, SUBMERGED, Capability.NONE),
                new Family(EntityType.DOLPHIN, true, SUBMERGED,
                        Capability.NONE),
                new Family(EntityType.BEE, true, EntityTraversalState.GROUNDED,
                        Capability.NONE))),
        MIXED(List.of(
                ground(EntityType.ZOMBIE, Capability.NONE),
                ground(EntityType.AXOLOTL, Capability.NONE),
                ground(EntityType.FROG, Capability.NONE),
                ground(EntityType.STRIDER, Capability.NONE)));

        private final List<Family> families;

        Topology(List<Family> families) {
            this.families = families;
        }
    }

    private static final class Pursuit {
        final String name;
        final Set<PathNode.Movement> movements =
                EnumSet.noneOf(PathNode.Movement.class);
        int requests;
        int shared;
        int individual;
        int certification;
        int untracked;
        int compared;
        int retainedTargets;

        Pursuit(String name) {
            this.name = name;
        }

        void record(SharedMeshSource source) {
            requests++;
            switch (source) {
                case SHARED_TARGET_FIELD -> shared++;
                case INDIVIDUAL_SEARCH -> individual++;
                case PARITY_CERTIFICATION_SEARCH -> certification++;
                case UNTRACKED_FALLBACK -> untracked++;
            }
        }

        @Override
        public String toString() {
            return name + "{requests=" + requests
                    + " shared=" + shared
                    + " individual=" + individual
                    + " certification=" + certification
                    + " untracked=" + untracked
                    + " plansCompared=" + compared
                    + " retainedTargetNodes=" + retainedTargets
                    + " movements=" + movements + '}';
        }
    }

    /**
     * One traversal-toggle workload. Divergences are counted rather than
     * thrown so a whole sweep reports how many of the plans it compared were
     * wrong, not merely that the first one was.
     */
    private static final class Toggle {
        final String name;
        final Set<PathNode.Movement> movements =
                EnumSet.noneOf(PathNode.Movement.class);
        int requests;
        int shared;
        int individual;
        int certification;
        int untracked;
        int regions;
        int compared;
        int divergences;
        String firstDivergence;

        Toggle(String name) {
            this.name = name;
        }

        void verify(SharedMeshHandle handle, NavigationRequest request,
                    EntityPathfinder oracle) {
            requests++;
            switch (handle.source()) {
                case SHARED_TARGET_FIELD -> shared++;
                case INDIVIDUAL_SEARCH -> individual++;
                case PARITY_CERTIFICATION_SEARCH -> certification++;
                case UNTRACKED_FALLBACK -> untracked++;
            }
            try {
                if (verifyShared(handle, request, oracle, movements)) {
                    compared++;
                }
            } catch (AssertionError failure) {
                divergences++;
                if (firstDivergence == null) {
                    firstDivergence = failure.getMessage();
                }
            }
        }

        @Override
        public String toString() {
            return name + "{requests=" + requests
                    + " regions=" + regions
                    + " shared=" + shared
                    + " sharedRate=" + rate()
                    + " individual=" + individual
                    + " certification=" + certification
                    + " untrackedFallback=" + untracked
                    + " plansCompared=" + compared
                    + " divergences=" + divergences
                    + " movements=" + movements + '}';
        }

        private String rate() {
            return requests == 0 ? "n/a"
                    : String.format("%.1f%%", 100.0 * shared / requests);
        }
    }

    private static final class Totals {
        final Set<PathNode.Movement> movements =
                EnumSet.noneOf(PathNode.Movement.class);
        int requests;
        int promotedScenarios;
        int retainedNodes;
        int sharedPlans;
        int freshInfluenceScenarios;
        int freshInfluencePromoted;
        int freshInfluenceSharedPlans;
    }
}
