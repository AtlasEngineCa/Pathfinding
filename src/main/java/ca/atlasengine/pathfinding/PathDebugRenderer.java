package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.result.PathNode;import net.minestom.server.coordinate.Point;
import net.minestom.server.coordinate.Vec;
import net.minestom.server.instance.Instance;
import net.minestom.server.network.packet.server.play.ParticlePacket;
import net.minestom.server.particle.Particle;

import java.util.List;

/**
 * Draws the route a controller is walking, so "why did it go there?" is a
 * question you can answer by looking.
 *
 * <p>Call {@link #render(EntityNavigationController)} from the same tick
 * thread that ticks the controller, as often as you want to see it. Nothing
 * here is required for navigation, and nothing here is on the search path:
 * this reads the controller's published route and sends particles.</p>
 *
 * <p>Particles are sent to everyone who can see the instance. Rendering every
 * mob on a busy server is a lot of packets, so render the one you are
 * debugging.</p>
 */
public final class PathDebugRenderer {
    /** Nodes already walked. */
    private final Particle walked;
    /** Nodes still to walk. */
    private final Particle remaining;
    /** The node the follower is heading for right now. */
    private final Particle current;
    /** Nodes reached by a jump or a climb rather than a step. */
    private final Particle special;
    private final int perNode;

    public PathDebugRenderer() {
        this(Particle.WAX_ON, Particle.HAPPY_VILLAGER, Particle.FLAME,
                Particle.ELECTRIC_SPARK, 1);
    }

    public PathDebugRenderer(Particle walked, Particle remaining,
                             Particle current, Particle special, int perNode) {
        if (walked == null || remaining == null || current == null
                || special == null || perNode < 1) {
            throw new IllegalArgumentException("invalid debug renderer");
        }
        this.walked = walked;
        this.remaining = remaining;
        this.current = current;
        this.special = special;
        this.perNode = perNode;
    }

    /**
     * Draws the controller's current route, or nothing when it has none.
     *
     * @return the number of nodes drawn
     */
    public int render(EntityNavigationController controller) {
        if (controller == null) throw new IllegalArgumentException("controller");
        Instance instance = controller.entity().getInstance();
        if (instance == null) return 0;
        return render(instance, controller.nodes(), controller.nodeIndex());
    }

    /**
     * Draws an arbitrary route, for a plan that is cached or not yet followed.
     */
    public int render(Instance instance, NavigationPlan plan) {
        if (instance == null || plan == null) {
            throw new IllegalArgumentException("instance and plan");
        }
        return render(instance, plan.result().nodes(), 0);
    }

    private int render(Instance instance, List<PathNode> nodes, int nodeIndex) {
        for (int index = 0; index < nodes.size(); index++) {
            PathNode node = nodes.get(index);
            Particle particle;
            if (isSpecial(node)) {
                particle = special;
            } else if (index == nodeIndex) {
                particle = current;
            } else {
                particle = index < nodeIndex ? walked : remaining;
            }
            // Node positions are entity bottom-center, so lift the marker to
            // where it is visible rather than buried in the floor.
            Point at = new Vec(node.x(), node.y() + 0.15, node.z());
            instance.sendGroupedPacket(new ParticlePacket(
                    particle, at, Vec.ZERO, 0f, perNode));
        }
        return nodes.size();
    }

    private static boolean isSpecial(PathNode node) {
        return node.movement() == PathNode.Movement.JUMP
                || node.movement() == PathNode.Movement.CLIMB;
    }
}
