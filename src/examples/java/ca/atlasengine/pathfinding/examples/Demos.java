package ca.atlasengine.pathfinding.examples;

import net.kyori.adventure.text.Component;
import net.kyori.adventure.text.format.NamedTextColor;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;
import ca.atlasengine.pathfinding.profile.GroundCapabilities;
import ca.atlasengine.pathfinding.metrics.NavigationMetricsSnapshot;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationModifiers;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.NavigationSystem;
import ca.atlasengine.pathfinding.result.PathResult;
import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;
import ca.atlasengine.pathfinding.adaptive.SharedMeshStatus;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.influence.NavigationZoneInfluence;
import ca.atlasengine.pathfinding.terrain.BlockManipulationCapabilities;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import ca.atlasengine.pathfinding.terrain.TerrainClassification;
import ca.atlasengine.pathfinding.terrain.TerrainType;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.entity.Player;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CompletableFuture;

/** One method per {@code /nav} demo. */
public final class Demos {
    private static final double CROWD_RADIUS = 16;
    private static final int SHED_BURST = 64;

    private Demos() {
    }

    /** The minimum case: spawn a mob, send it to a point. */
    public static void walk(DemoSession session) {
        session.player().teleport(DemoWorld.MAZE_VIEW);
        session.spawn(EntityType.ZOMBIE, "walk", DemoWorld.MAZE_START)
                .moveTo(DemoWorld.MAZE_GOAL);
        say(session, "A zombie is routing the maze to the emerald block.");
    }

    /**
     * Retargeting on every tick. The controller keeps following the route it
     * has until a replacement lands, so the mob never stalls waiting.
     */
    public static void chase(DemoSession session) {
        Player player = session.player();
        player.teleport(DemoWorld.PLAZA);
        session.spawn(EntityType.ZOMBIE, "chase", DemoWorld.CHASE_START)
                .pursue(player::getPosition);
        say(session, "Run. It calls moveTo(player) every tick.");
    }

    /** Platform jumping: off in every baseline profile, on here. */
    public static void jump(DemoSession session) {
        session.player().teleport(DemoWorld.JUMP_VIEW);
        NavigationProfile base = BuiltinNavigationProfiles
                .forEntityType(EntityType.ZOMBIE);
        // Physical ability is never inferred from terrain. A mob crosses the
        // gap only because a profile said its body can.
        NavigationProfile jumping = base.withGroundCapabilities(
                base.groundCapabilities().withPlatformJump(
                        PlatformJumpCapabilities.acrossGaps(1)));

        session.spawn(EntityType.ZOMBIE, "jump: enabled",
                        DemoWorld.JUMP_START, jumping)
                .moveTo(DemoWorld.JUMP_GOAL);
        session.spawn(EntityType.ZOMBIE, "jump: default",
                        DemoWorld.JUMP_START_PLAIN)
                .moveTo(DemoWorld.JUMP_GOAL_PLAIN);
        say(session, "The default mob stops at the lip with a PARTIAL route.");
    }

    /** Door manipulation, including closing behind, which baseline never does. */
    public static void doors(DemoSession session) {
        session.player().teleport(DemoWorld.DOORS_VIEW);
        NavigationProfile base = BuiltinNavigationProfiles
                .forEntityType(EntityType.ZOMBIE);
        NavigationProfile polite = base.withMobProfile(
                MobTraversalProfile.builder("polite_zombie")
                        .from(base.mobProfile())
                        .blockManipulation(BlockManipulationCapabilities
                                .STANDARD.closingBehind())
                        .build());

        session.spawn(EntityType.ZOMBIE, "doors: enabled",
                        DemoWorld.DOORS_START, polite)
                .moveTo(DemoWorld.DOORS_GOAL);
        session.spawn(EntityType.ZOMBIE, "doors: default",
                        DemoWorld.DOORS_START_PLAIN)
                .moveTo(DemoWorld.DOORS_GOAL_PLAIN);
        say(session, "Built-in mobs open doors and leave them open; closing "
                + "behind is opt-in.");
    }

    /**
     * Planned climbing. Minecraft physics understands ladders, but its ground
     * A* never generates a vertical edge through one, so this stays off.
     */
    public static void climb(DemoSession session) {
        session.player().teleport(DemoWorld.CLIMB_VIEW);
        NavigationProfile base = BuiltinNavigationProfiles
                .forEntityType(EntityType.ZOMBIE);
        NavigationProfile climbing = base.withGroundCapabilities(
                base.groundCapabilities().withClimbables(
                        ClimbableCapabilities.STANDARD));

        session.spawn(EntityType.ZOMBIE, "climb: enabled",
                        DemoWorld.CLIMB_START, climbing)
                .moveTo(DemoWorld.CLIMB_GOAL);
        session.spawn(EntityType.ZOMBIE, "climb: default",
                        DemoWorld.CLIMB_START_PLAIN)
                .moveTo(DemoWorld.CLIMB_GOAL_PLAIN);
        say(session, "The default mob never plans a route into the ladder.");
    }

    /** The navigation family follows the entity type, with no extra wiring. */
    public static void swim(DemoSession session) {
        session.player().teleport(DemoWorld.POND_VIEW);
        session.spawn(EntityType.COD, "swim", DemoWorld.SWIM_START)
                .moveTo(DemoWorld.SWIM_GOAL);
        say(session, "A cod gets the three-dimensional swim graph and rises "
                + "over the submerged wall.");
    }

    public static void fly(DemoSession session) {
        session.player().teleport(DemoWorld.FLY_VIEW);
        session.spawn(EntityType.BEE, "fly", DemoWorld.FLY_START)
                .moveTo(DemoWorld.FLY_GOAL);
        say(session, "A bee gets the flying graph and clears the wall in "
                + "three dimensions.");
    }

    /**
     * A mob the game does not ship, composed from four immutable pieces that
     * answer four separate questions.
     */
    public static void custom(DemoSession session) {
        session.player().teleport(DemoWorld.CUSTOM_VIEW);

        // What a block the library has never heard of actually is, named in
        // the baseline vocabulary. null declines, so baseline is untouched.
        TerrainClassification arcane = TerrainClassification.keyed(
                block -> DemoWorld.HAZARD.key().equals(block.key())
                        ? TerrainType.DAMAGING : null,
                "examples:arcane@1");

        // What terrain costs, and what the mob may pass or open.
        MobTraversalProfile courier = MobTraversalProfile.builder("courier")
                .from(MobTraversalProfile.ANIMAL)
                .malus(TerrainType.DAMAGING, 12)   // singed, not stopped
                .canOpenDoors(true)
                .classification(arcane)
                .build();

        // What the body can physically do.
        GroundCapabilities body = GroundCapabilities.STANDARD
                .withPlatformJump(PlatformJumpCapabilities.acrossGaps(1));

        NavigationProfile profile = NavigationProfile.builder(NavigationMode.GROUND, courier, body).allowBreaching(false).prefersShallowWater(// allowBreaching
                false).avoidSun(// prefersShallowWater
                false).build();  // avoidSun

        // What this one request wants, on top of all of that.
        NavigationInfluence closedRoad = NavigationZoneInfluence.blocks(
                DemoWorld.CLOSED_ROAD_FIRST, DemoWorld.CLOSED_ROAD_LAST,
                true, 0, "closed-road");

        session.spawn(EntityType.HUSK, "custom", DemoWorld.CUSTOM_START,
                        profile)
                .controller()
                .moveTo(DemoWorld.CUSTOM_GOAL, List.of(closedRoad),
                        NavigationModifiers.NONE);
        say(session, "It skirts the magenta wool it prices as damaging, jumps "
                + "the gap, and refuses the gold zone.");
    }

    /** Many mobs converging on one target, planned through the shared mesh. */
    public static void crowd(DemoSession session, int count) {
        Player player = session.player();
        player.teleport(DemoWorld.PLAZA);
        for (int index = 0; index < count; index++) {
            double angle = 2 * Math.PI * index / count;
            Pos at = new Pos(
                    DemoWorld.PLAZA.x() + Math.cos(angle) * CROWD_RADIUS,
                    DemoWorld.GROUND,
                    DemoWorld.PLAZA.z() + Math.sin(angle) * CROWD_RADIUS);
            session.spawnPursuer(EntityType.ZOMBIE, at, player, 0);
        }
        say(session, count + " pursuers are planning through the shared mesh. "
                + "Walk around, then run /nav metrics: a field is reused when "
                + "you return to a cell some pursuer already planned to.");
    }

    /**
     * Backpressure. A shed submission completes normally with an empty result
     * whose status is SHED: the caller keeps the route it already has and asks
     * again later. This is the one place a second system exists, because a
     * shed is what a full queue looks like and the demo system is not full.
     */
    public static void shed(DemoSession session) {
        Player player = session.player();
        NavigationProfile profile = BuiltinNavigationProfiles
                .forEntityType(EntityType.ZOMBIE);
        NavigationRequest request = NavigationRequest.builder(
                        player.getInstance(), DemoWorld.MAZE_START,
                        DemoWorld.MAZE_GOAL, new BoundingBox(0.6, 1.95, 0.6),
                        profile)
                .maxPathLength(64)
                .build();

        long shed;
        long routed;
        try (NavigationSystem tiny = NavigationSystem.builder()
                .parallelism(1).queueCapacity(1).build()) {
            List<CompletableFuture<PathResult>> burst = new ArrayList<>();
            for (int index = 0; index < SHED_BURST; index++) {
                burst.add(tiny.submit(request));
            }
            List<PathResult> results = burst.stream()
                    .map(CompletableFuture::join).toList();
            shed = results.stream().filter(PathResult::shed).count();
            routed = results.stream().filter(PathResult::found).count();
        }

        header(session, "backpressure");
        say(session, SHED_BURST + " submissions against a queue of one: "
                + routed + " routed, " + shed + " shed.");
        say(session, "A shed future completes normally. It is not a failure "
                + "and not an exception: keep the current route, retry later.");
    }

    public static void metrics(DemoSession session) {
        NavigationSystem navigation = session.navigation();
        NavigationMetricsSnapshot snapshot = navigation.metricsSnapshot();
        NavigationMetricsSnapshot.SearchCounts searches = snapshot.searches();
        SharedMeshStatus mesh = navigation.sharedMesh().status();

        header(session, "metrics");
        say(session, String.format(Locale.ROOT,
                "searches  submitted=%d completed=%d shed=%d superseded=%d "
                        + "inFlight=%d",
                searches.submitted(), searches.completed(), searches.rejected(),
                searches.superseded(), searches.inFlight()));
        say(session, String.format(Locale.ROOT,
                "latency   p50=%dus p90=%dus p99=%dus | queueWait p99=%dus",
                snapshot.latency().p50Micros(), snapshot.latency().p90Micros(),
                snapshot.latency().p99Micros(),
                snapshot.queueWait().p99Micros()));
        say(session, String.format(Locale.ROOT,
                "queue     active=%d queued=%d/%d parallelism=%d",
                snapshot.queue().activeSearches(),
                snapshot.queue().queuedSearches(),
                snapshot.queue().queueCapacity(),
                snapshot.queue().parallelism()));
        say(session, String.format(Locale.ROOT,
                "outcomes  completed=%d stuck=%d unreachable=%d failed=%d "
                        + "travelStallRate=%.3f",
                snapshot.controllerOutcomes().completed(),
                snapshot.controllerOutcomes().stuck(),
                snapshot.controllerOutcomes().unreachableTargets(),
                snapshot.controllerOutcomes().failed(),
                snapshot.controllerOutcomes().travelStallRate()));
        say(session, String.format(Locale.ROOT,
                "mesh      enabled=%b requests=%d sharedHitRate=%.3f "
                        + "(of all adaptive dispatch %.3f) regions=%d "
                        + "exhausted=%b stalePlans=%d",
                mesh.enabled(), mesh.meshRequests(), mesh.sharedHitRate(),
                snapshot.adaptiveSources().sharedHitRate(),
                mesh.regions().regions(), mesh.regionsExhausted(),
                snapshot.stalePlans()));
        say(session, "Counters are cumulative. A rate is the difference "
                + "between two snapshots.");
    }

    public static void stop(DemoSession session) {
        say(session, "Closed " + session.stop() + " controllers and despawned "
                + "their mobs.");
    }

    private static void header(DemoSession session, String text) {
        session.player().sendMessage(
                Component.text("[nav] " + text, NamedTextColor.AQUA));
    }

    private static void say(DemoSession session, String text) {
        session.player().sendMessage(
                Component.text(text, NamedTextColor.GRAY));
    }
}
