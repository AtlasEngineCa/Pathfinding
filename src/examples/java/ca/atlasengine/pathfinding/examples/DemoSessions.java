package ca.atlasengine.pathfinding.examples;

import ca.atlasengine.pathfinding.NavigationSystem;
import net.minestom.server.entity.Player;
import net.minestom.server.instance.Instance;

import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;

/**
 * One {@link DemoSession} per connected player, so a disconnect closes the
 * controllers that player's demos left running.
 */
public final class DemoSessions {
    private final Map<UUID, DemoSession> byPlayer = new ConcurrentHashMap<>();
    private final NavigationSystem navigation;
    private final Instance instance;

    public DemoSessions(NavigationSystem navigation, Instance instance) {
        this.navigation = navigation;
        this.instance = instance;
    }

    public DemoSession of(Player player) {
        return byPlayer.computeIfAbsent(player.getUuid(),
                id -> new DemoSession(navigation, instance, player));
    }

    public void end(Player player) {
        DemoSession session = byPlayer.remove(player.getUuid());
        if (session != null) session.stop();
    }
}
