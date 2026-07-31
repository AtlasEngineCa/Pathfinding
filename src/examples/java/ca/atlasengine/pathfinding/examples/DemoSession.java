package ca.atlasengine.pathfinding.examples;

import ca.atlasengine.pathfinding.profile.NavigationProfile;
import ca.atlasengine.pathfinding.NavigationSystem;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.Entity;
import net.minestom.server.entity.EntityType;
import net.minestom.server.entity.Player;
import net.minestom.server.instance.Instance;

import java.util.ArrayList;
import java.util.List;

/**
 * Everything one player's demos spawned, so {@code /nav stop} can close every
 * controller and despawn every mob without hunting for them.
 */
public final class DemoSession {
    static final double SPEED = 0.18;

    private final NavigationSystem navigation;
    private final Instance instance;
    private final Player player;
    private final List<AutoCloseable> spawned = new ArrayList<>();

    public DemoSession(NavigationSystem navigation, Instance instance,
                       Player player) {
        this.navigation = navigation;
        this.instance = instance;
        this.player = player;
    }

    public Player player() {
        return player;
    }

    public NavigationSystem navigation() {
        return navigation;
    }

    /** Spawns a mob navigating with the baseline profile for its type. */
    public DemoMob spawn(EntityType type, String label, Pos at) {
        DemoMob mob = new DemoMob(type, label);
        mob.setInstance(instance, at).join();
        return track(mob.navigateWith(navigation.controller(mob, SPEED)));
    }

    /** Spawns a mob navigating with an explicitly composed profile. */
    public DemoMob spawn(EntityType type, String label, Pos at,
                         NavigationProfile profile) {
        DemoMob mob = new DemoMob(type, label);
        mob.setInstance(instance, at).join();
        return track(mob.navigateWith(
                navigation.controller(mob, profile, SPEED)));
    }

    /** Spawns a mob that pursues {@code quarry} through the shared mesh. */
    public PursuitMob spawnPursuer(EntityType type, Pos at, Entity quarry,
                                   long currentTick) {
        PursuitMob mob = new PursuitMob(type);
        mob.setInstance(instance, at).join();
        return track(mob.pursue(navigation.sharedMesh().pursue(
                mob, quarry, 1, currentTick)));
    }

    /** Closes every controller this session created, then despawns its mobs. */
    public int stop() {
        int closed = spawned.size();
        for (AutoCloseable mob : spawned) {
            try {
                mob.close();
            } catch (Exception unreachable) {
                throw new IllegalStateException(unreachable);
            }
        }
        spawned.clear();
        // The mesh keeps a target field per pursued entity; release it with
        // the pursuers rather than waiting for idle expiry.
        navigation.sharedMesh().forgetTarget(player.getUuid());
        return closed;
    }

    private <T extends AutoCloseable> T track(T mob) {
        spawned.add(mob);
        return mob;
    }
}
