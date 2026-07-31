package ca.atlasengine.pathfinding.examples;

import ca.atlasengine.pathfinding.profile.ClimbableCapabilities;import ca.atlasengine.pathfinding.profile.PlatformJumpCapabilities;import net.kyori.adventure.text.Component;
import net.kyori.adventure.text.format.NamedTextColor;
import net.minestom.server.command.CommandSender;
import net.minestom.server.command.builder.Command;
import net.minestom.server.command.builder.CommandContext;
import net.minestom.server.command.builder.arguments.ArgumentType;
import net.minestom.server.entity.Player;

import java.util.List;

/** {@code /nav <demo>}: every capability, driven from chat. */
public final class NavCommand extends Command {
    private static final List<String> USAGE = List.of(
            "walk    a zombie routes the maze to a point",
            "chase   a zombie pursues you, retargeting every tick",
            "jump    PlatformJumpCapabilities across a gap (opt-in)",
            "doors   BlockManipulationCapabilities, closing behind (opt-in)",
            "climb   ClimbableCapabilities up a ladder (opt-in)",
            "swim    a cod on the three-dimensional swim graph",
            "fly     a bee on the flying graph",
            "custom  a mob composed from profile, body, terrain, influence",
            "crowd <n>  n pursuers planning through the shared mesh",
            "shed    what backpressure looks like to a caller",
            "metrics searches, latency, mesh hit rate, stall rate",
            "stop    close every controller and despawn its mob");

    private final DemoSessions sessions;

    public NavCommand(DemoSessions sessions) {
        super("nav");
        this.sessions = sessions;

        var demo = ArgumentType.Word("demo").from(
                "walk", "chase", "jump", "doors", "climb", "swim", "fly",
                "custom", "crowd", "shed", "metrics", "stop", "help");
        var count = ArgumentType.Integer("count");

        setDefaultExecutor((sender, context) -> help(sender));
        addSyntax((sender, context) -> run(sender, context.get(demo), 12), demo);
        addSyntax((sender, context) ->
                run(sender, context.get(demo), context.get(count)), demo, count);
    }

    private void run(CommandSender sender, String demo, int count) {
        if (!(sender instanceof Player player)) {
            sender.sendMessage(Component.text("/nav is played, not typed at a "
                    + "console.", NamedTextColor.RED));
            return;
        }
        DemoSession session = sessions.of(player);
        switch (demo) {
            case "walk" -> Demos.walk(session);
            case "chase" -> Demos.chase(session);
            case "jump" -> Demos.jump(session);
            case "doors" -> Demos.doors(session);
            case "climb" -> Demos.climb(session);
            case "swim" -> Demos.swim(session);
            case "fly" -> Demos.fly(session);
            case "custom" -> Demos.custom(session);
            case "crowd" -> Demos.crowd(session, Math.clamp(count, 1, 64));
            case "shed" -> Demos.shed(session);
            case "metrics" -> Demos.metrics(session);
            case "stop" -> Demos.stop(session);
            default -> help(player);
        }
    }

    private static void help(CommandSender sender) {
        sender.sendMessage(Component.text("[nav] demos", NamedTextColor.AQUA));
        for (String line : USAGE) {
            sender.sendMessage(
                    Component.text("  /nav " + line, NamedTextColor.GRAY));
        }
    }
}
