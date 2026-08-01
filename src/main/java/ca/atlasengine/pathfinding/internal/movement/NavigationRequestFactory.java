package ca.atlasengine.pathfinding.internal.movement;

import ca.atlasengine.pathfinding.terrain.BlockTraversalData;
import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;
import ca.atlasengine.pathfinding.search.EntityTraversalState;
import ca.atlasengine.pathfinding.profile.NavigationMode;
import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.search.NavigationRequest;
import ca.atlasengine.pathfinding.search.StartNodeSampler;
import ca.atlasengine.pathfinding.influence.NavigationInfluence;
import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Point;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.LivingEntity;
import net.minestom.server.entity.attribute.Attribute;
import net.minestom.server.instance.Instance;
import net.minestom.server.instance.block.Block;

import java.util.List;
import java.util.Set;
import java.util.SplittableRandom;
import java.util.random.RandomGenerator;

/**
 * Assembles search requests from the entity's live state.
 */
public final class NavigationRequestFactory {
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
        double searchRange = liveNodeSearchRange() * searchRangeMultiplier;
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
        boolean onRail = BlockTraversalData.isRail(instance.getBlock(start))
                || BlockTraversalData.isRail(instance.getBlock(start.add(0, -1, 0)));
        MobTraversalProfile base = profile.mobProfile();
        if (base.currentlyOnRail() == onRail) return profile;
        MobTraversalProfile snapshot = MobTraversalProfile.builder(base.name() + "_request")
                .from(base)
                .currentlyOnRail(onRail)
                .build();
        return profile.withMobProfile(snapshot);
    }

    private double liveNodeSearchRange() {
        double followRange = entity instanceof LivingEntity living
                ? living.getAttribute(Attribute.FOLLOW_RANGE).getValue()
                : 16;
        return Math.max(
                followRange,
                refreshBuiltinProfile
                        ? BuiltinNavigationProfiles.requiredPathLength(entity)
                        : 16);
    }

    private EntityTraversalState snapshotEntityState(
            NavigationProfile profile, Instance instance, boolean entityInWater) {
        Set<Block> standable = profile.mobProfile().standsOnLava()
                ? Set.of(Block.LAVA) : Set.of();
        return new EntityTraversalState(
                entity.isOnGround(), entityInWater, standable,
                instance.getCachedDimensionType().minY(),
                instance.getCachedDimensionType().maxY(),
                startCandidates(profile));
    }

    private List<Point> startCandidates(NavigationProfile profile) {
        if (profile.mode() != NavigationMode.FLYING) return List.of();
        var box = entity.getBoundingBox();
        double meanSize = (box.width() + box.height() + box.depth()) / 3;
        if (meanSize >= 1) return List.of();
        return StartNodeSampler.sampleSmallEntity(
                entity.getPosition(), box, startNodeRandom, 10);
    }


}
