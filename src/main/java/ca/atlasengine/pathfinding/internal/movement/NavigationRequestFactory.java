package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.internal.Footprint;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.search.StartNodeSampler;
import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.LivingEntity;
import net.minestom.server.entity.attribute.Attribute;
import net.minestom.server.instance.Instance;

import java.util.List;
import java.util.SplittableRandom;
import java.util.random.RandomGenerator;

/**
 * Assembles search requests from the entity's live state.
 */
public final class NavigationRequestFactory {
    private static final double DEFAULT_SEARCH_RANGE = 16;
    /** Mean extent below which a flyer's start is sampled around it. */
    private static final double SMALL_FLYER_EXTENT = 1;
    private static final int SMALL_FLYER_START_SAMPLES = 10;
    private final Entity entity;
    private final boolean refreshBuiltinProfile;
    private final RandomGenerator startNodeRandom;

    public NavigationRequestFactory(Entity entity, boolean refreshBuiltinProfile,
                                    RandomGenerator startNodeRandom) {
        this.entity = entity;
        this.refreshBuiltinProfile = refreshBuiltinProfile;
        this.startNodeRandom = startNodeRandom;
    }

    public NavigationRequest build(NavigationProfile profile, Instance instance,
                                   Point start, Point destination, BoundingBox box,
                                   List<NavigationInfluence> influences,
                                   double searchRangeMultiplier, boolean entityInWater) {
        double searchRange = searchRange(entity, refreshBuiltinProfile)
                * searchRangeMultiplier;
        return NavigationRequest.builder(instance, start, destination, box,
                        snapshotRailState(profile, instance, start))
                .maxPathLength(searchRange)
                .reachRange(0)
                .maxVisitedMultiplier(searchRangeMultiplier)
                .influences(influences)
                .entityState(snapshotEntityState(profile, instance, entityInWater))
                .nodeSearchRange(searchRange)
                .build();
    }

    static RandomGenerator entityRandom(Entity entity) {
        long seed = entity.getUuid().getMostSignificantBits()
                ^ Long.rotateLeft(entity.getUuid().getLeastSignificantBits(), 29);
        return new SplittableRandom(seed);
    }

    public static RandomGenerator entityRandomOrNull(Entity entity) {
        return entity == null ? null : entityRandom(entity);
    }

    private static NavigationProfile snapshotRailState(
            NavigationProfile profile, Instance instance, Point start) {
        boolean onRail = BlockTraversalData.isRailAt(instance, start);
        MobTraversalProfile base = profile.mobProfile();
        if (base.currentlyOnRail() == onRail) return profile;
        MobTraversalProfile snapshot = MobTraversalProfile.builder(base.name() + "_request")
                .from(base)
                .currentlyOnRail(onRail)
                .build();
        return profile.withMobProfile(snapshot);
    }

    /**
     * How far the entity searches: its live follow range, raised to the
     * baseline navigation's own minimum when it navigates as its built-in type.
     */
    public static double searchRange(Entity entity, boolean builtinProfile) {
        double followRange = entity instanceof LivingEntity living
                ? living.getAttribute(Attribute.FOLLOW_RANGE).getValue()
                : DEFAULT_SEARCH_RANGE;
        return Math.max(followRange, builtinProfile
                ? BuiltinNavigationProfiles.requiredPathLength(entity)
                : DEFAULT_SEARCH_RANGE);
    }

    private EntityTraversalState snapshotEntityState(
            NavigationProfile profile, Instance instance, boolean entityInWater) {
        return new EntityTraversalState(
                entity.isOnGround(), entityInWater,
                profile.mobProfile().standableFluids(),
                instance.getCachedDimensionType().minY(),
                instance.getCachedDimensionType().maxY() - 1,
                startCandidates(profile));
    }

    private List<Point> startCandidates(NavigationProfile profile) {
        if (profile.mode() != NavigationMode.FLYING) return List.of();
        BoundingBox box = entity.getBoundingBox();
        if (Footprint.meanExtent(box) >= SMALL_FLYER_EXTENT) return List.of();
        return StartNodeSampler.sampleSmallEntity(entity.getPosition(), box,
                startNodeRandom, SMALL_FLYER_START_SAMPLES);
    }
}
