package ca.atlasengine.pathfinding.examples;

import net.kyori.adventure.text.Component;
import ca.atlasengine.pathfinding.EntityNavigationController;
import ca.atlasengine.pathfinding.NavigationState;
import ca.atlasengine.pathfinding.PathDebugRenderer;
import net.minestom.server.component.DataComponents;
import net.minestom.server.coordinate.Point;
import net.minestom.server.entity.EntityCreature;
import net.minestom.server.entity.EntityType;

import java.util.function.Supplier;

/**
 * A demo mob and the one controller that navigates it.
 *
 * <p>The controller is ticked from {@link #update(long)}, which Minestom runs
 * on the tick thread that owns this entity. Nothing else may call it: worker
 * threads only ever read the immutable request a controller submitted.</p>
 */
public final class DemoMob extends EntityCreature implements AutoCloseable {
    private static final PathDebugRenderer DEBUG_PATH = new PathDebugRenderer();

    private EntityNavigationController controller;
    private Supplier<Point> quarry;

    public DemoMob(EntityType type, String label) {
        super(type);
        set(DataComponents.CUSTOM_NAME, Component.text(label));
        getEntityMeta().setCustomNameVisible(true);
    }

    /** Binds the single controller this mob owns for its whole life. */
    public DemoMob navigateWith(EntityNavigationController controller) {
        this.controller = controller;
        return this;
    }

    public void moveTo(Point target) {
        controller.moveTo(target);
    }

    /**
     * Re-reads the destination on every tick. Retargeting this often is
     * supported: a route already being followed is kept until its replacement
     * lands, so the mob never stands still waiting for a search.
     */
    public void pursue(Supplier<Point> target) {
        quarry = target;
    }

    public NavigationState state() {
        return controller.state();
    }

    public EntityNavigationController controller() {
        return controller;
    }

    @Override
    public void update(long time) {
        if (controller != null) {
            if (quarry != null) controller.moveTo(quarry.get());
            controller.tick();
            DEBUG_PATH.render(controller);
        }
        super.update(time);
    }

    /** Closes the controller and despawns the mob. The system outlives both. */
    @Override
    public void close() {
        if (controller != null) controller.close();
        remove();
    }
}
