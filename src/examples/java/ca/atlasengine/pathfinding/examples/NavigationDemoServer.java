package ca.atlasengine.pathfinding.examples;

import net.kyori.adventure.text.Component;
import net.kyori.adventure.text.format.NamedTextColor;
import ca.atlasengine.pathfinding.NavigationSystem;
import ca.atlasengine.pathfinding.adaptive.SharedMeshOptions;
import net.minestom.server.Auth;
import net.minestom.server.MinecraftServer;
import net.minestom.server.entity.GameMode;
import net.minestom.server.event.GlobalEventHandler;
import net.minestom.server.event.player.AsyncPlayerConfigurationEvent;
import net.minestom.server.event.player.PlayerDisconnectEvent;
import net.minestom.server.event.player.PlayerSpawnEvent;
import net.minestom.server.instance.InstanceContainer;
import net.minestom.server.instance.LightingChunk;
import net.minestom.server.instance.block.Block;
import net.minestom.server.utils.time.TimeUnit;

/**
 * A runnable server demonstrating every navigation capability from chat.
 *
 * <pre>{@code ../Minestom/gradlew runExamples}</pre>
 *
 * Join {@code localhost:25565} in offline mode and type {@code /nav}.
 */
public final class NavigationDemoServer {
    private static final int PORT = 25565;

    private NavigationDemoServer() {
    }

    public static void main(String[] args) {
        MinecraftServer.setCompressionThreshold(0);
        MinecraftServer server = MinecraftServer.init(new Auth.Offline());

        InstanceContainer instance = MinecraftServer.getInstanceManager()
                .createInstanceContainer();
        instance.setChunkSupplier(LightingChunk::new);
        instance.setGenerator(unit -> unit.modifier()
                .fillHeight(0, DemoWorld.GROUND, Block.STONE));
        DemoWorld.build(instance);

        // One system per server. It owns the bounded worker pool every
        // controller plans on, so it outlives every mob and is closed once,
        // at shutdown. Shared-mesh planning is off by default; only /nav crowd
        // reaches it, because every other entry point on this system plans one
        // individual A* search per request.
        NavigationSystem navigation = NavigationSystem.builder()
                .parallelism(4)
                .queueCapacity(128)
                .movementPerTick(DemoSession.SPEED)
                .sharedMesh(SharedMeshOptions.ENABLED)
                .build();
        DemoSessions sessions = new DemoSessions(navigation, instance);

        // Idle region expiry and promotion hysteresis, once per world tick.
        instance.scheduler()
                .buildTask(new SharedMeshTick(navigation))
                .repeat(1, TimeUnit.SERVER_TICK)
                .schedule();

        GlobalEventHandler events = MinecraftServer.getGlobalEventHandler();
        events.addListener(AsyncPlayerConfigurationEvent.class, event -> {
            event.setSpawningInstance(instance);
            event.getPlayer().setRespawnPoint(DemoWorld.SPAWN);
        });
        events.addListener(PlayerSpawnEvent.class, event -> {
            event.getPlayer().setGameMode(GameMode.CREATIVE);
            event.getPlayer().setPermissionLevel(2);
            event.getPlayer().sendMessage(Component.text(
                    "Minestom pathfinding demo. Type /nav for the demo list.",
                    NamedTextColor.AQUA));
        });
        events.addListener(PlayerDisconnectEvent.class,
                event -> sessions.end(event.getPlayer()));

        MinecraftServer.getCommandManager().register(new NavCommand(sessions));
        MinecraftServer.getSchedulerManager()
                .buildShutdownTask(navigation::close);

        server.start("0.0.0.0", PORT);
    }

    /** Counts world ticks for {@link NavigationSystem#tickAdaptiveNavigation}. */
    private static final class SharedMeshTick implements Runnable {
        private final NavigationSystem navigation;
        private long tick;

        private SharedMeshTick(NavigationSystem navigation) {
            this.navigation = navigation;
        }

        @Override
        public void run() {
            navigation.sharedMesh().tick(++tick);
        }
    }
}
