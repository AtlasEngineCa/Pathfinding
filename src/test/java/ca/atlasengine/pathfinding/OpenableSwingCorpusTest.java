package ca.atlasengine.pathfinding;

import ca.atlasengine.pathfinding.profile.BuiltinNavigationProfiles;import ca.atlasengine.pathfinding.result.PathNode;import ca.atlasengine.pathfinding.result.PathResult;import ca.atlasengine.pathfinding.search.EntityPathfinder;import ca.atlasengine.pathfinding.search.NavigationRequest;import ca.atlasengine.pathfinding.search.SearchControl;import ca.atlasengine.pathfinding.terrain.MobTraversalProfile;
import net.minestom.server.collision.BoundingBox;
import net.minestom.server.coordinate.Pos;
import net.minestom.server.entity.EntityType;
import net.minestom.server.instance.block.Block;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Route corpus over every openable orientation and a mixed terrain field,
 * planned for a profile that opens nothing and for one that opens doors.
 *
 * <p>The expected rows preserve the routes this library produced before
 * openables were read in their swung state, except where the swing model or
 * physical head clearance deliberately changes them.</p>
 */
class OpenableSwingCorpusTest {
    private static final BoundingBox[] BOXES = {
            new BoundingBox(0.6, 1.95, 0.6),
            new BoundingBox(1.4, 1.95, 1.4)
    };

    @Test
    void aProfileThatOpensNothingRoutesExactlyAsItAlwaysDid() {
        assertEquals(EXPECTED_OPENS_NOTHING,
                corpus(MobTraversalProfile.builder("corpus_shut").build()));
    }

    @Test
    void aDoorOpenerOnlyChangesWhereItsOwnPanelStandsInTheWay() {
        assertEquals(EXPECTED_OPENS_DOORS, corpus(MobTraversalProfile
                .builder("corpus_doors").canOpenDoors(true).build()));
    }

    private static List<String> corpus(MobTraversalProfile profile) {
        List<String> rows = new ArrayList<>();
        for (Map.Entry<String, TestWorld> world : worlds().entrySet()) {
            for (BoundingBox box : BOXES) {
                rows.add(world.getKey() + "|w" + box.width() + "|"
                        + route(world.getValue(), box, profile));
            }
        }
        return rows;
    }

    private static String route(TestWorld world, BoundingBox box,
                                MobTraversalProfile profile) {
        PathResult result = new EntityPathfinder().findPath(
                NavigationRequest.builder(world, new Pos(0.5, 1, 0.5),
                                new Pos(6.5, 1, 0.5), box,
                                BuiltinNavigationProfiles
                                        .forEntityType(EntityType.ZOMBIE)
                                        .withMobProfile(profile))
                        .maxPathLength(32).maxVisitedMultiplier(8).build(),
                SearchControl.NONE);
        StringBuilder builder = new StringBuilder(result.status().name());
        for (PathNode node : result.nodes()) {
            builder.append(';').append(node.graphX()).append(',')
                    .append(node.graphY()).append(',').append(node.graphZ());
        }
        return builder.toString();
    }

    private static Map<String, TestWorld> worlds() {
        Map<String, TestWorld> map = new LinkedHashMap<>();
        map.put("plain", corridor());
        map.put("door_east_left", door(corridor(), "east", "left", "false"));
        map.put("door_north_right",
                door(corridor(), "north", "right", "false"));
        map.put("door_open", door(corridor(), "east", "left", "true"));
        map.put("iron_door", door(corridor(), "east", "left", "false",
                Block.IRON_DOOR));
        map.put("trapdoor_east_bottom",
                corridor().set(3, 1, 0, trapdoor("east", "bottom", "false")));
        map.put("trapdoor_north_top",
                corridor().set(3, 1, 0, trapdoor("north", "top", "false")));
        map.put("trapdoor_open_east",
                corridor().set(3, 1, 0, trapdoor("east", "bottom", "true")));
        map.put("trapdoor_head",
                corridor().set(3, 2, 0, trapdoor("west", "bottom", "false")));
        map.put("gate_closed", corridor().set(3, 1, 0, gate("false")));
        map.put("gate_open", corridor().set(3, 1, 0, gate("true")));
        map.put("fence", corridor().set(3, 1, 0, Block.OAK_FENCE));
        map.put("mixed", mixed());
        map.put("wide_gap_trapdoor", gap(trapdoor("north", "bottom", "false")));
        map.put("wide_gap_door", gapDoor());
        return map;
    }

    private static Block trapdoor(String facing, String half, String open) {
        return Block.OAK_TRAPDOOR.withProperty("facing", facing)
                .withProperty("half", half).withProperty("open", open);
    }

    private static Block gate(String open) {
        return Block.OAK_FENCE_GATE.withProperty("facing", "north")
                .withProperty("open", open);
    }

    private static TestWorld door(TestWorld world, String facing,
                                  String hinge, String open) {
        return door(world, facing, hinge, open, Block.OAK_DOOR);
    }

    private static TestWorld door(TestWorld world, String facing, String hinge,
                                  String open, Block material) {
        Block base = material.withProperty("facing", facing)
                .withProperty("hinge", hinge).withProperty("open", open);
        world.set(3, 1, 0, base.withProperty("half", "lower"));
        world.set(3, 2, 0, base.withProperty("half", "upper"));
        return world;
    }

    private static TestWorld corridor() {
        TestWorld world = new TestWorld().floor(-2, 8, -1, 1, 0, Block.STONE);
        for (int x = -2; x <= 8; x++) {
            world.column(x, -1, 1, 4, Block.STONE);
            world.column(x, 1, 1, 4, Block.STONE);
        }
        return world;
    }

    private static TestWorld mixed() {
        TestWorld world = new TestWorld().floor(-2, 8, -4, 4, 0, Block.STONE);
        world.set(1, 1, 0, Block.OAK_STAIRS);
        world.set(2, 1, 0, Block.STONE_SLAB);
        world.set(3, 1, 1, Block.WATER);
        world.set(3, 1, -1, Block.LAVA);
        world.set(4, 1, 0, Block.OAK_FENCE);
        world.set(5, 1, 1, Block.LILY_PAD);
        world.set(5, 1, -1, Block.POWDER_SNOW);
        world.set(2, 1, 2, Block.RAIL);
        world.set(6, 1, 2, Block.CACTUS);
        return world;
    }

    private static TestWorld gap(Block openable) {
        TestWorld world = new TestWorld().floor(-2, 8, -4, 4, 0, Block.STONE);
        for (int z = -4; z <= 4; z++) {
            if (z >= -1 && z <= 1) continue;
            world.column(3, z, 1, 4, Block.STONE);
        }
        for (int z = -1; z <= 1; z++) world.set(3, 1, z, openable);
        return world;
    }

    private static TestWorld gapDoor() {
        TestWorld world = gap(Block.AIR);
        Block base = Block.OAK_DOOR.withProperty("facing", "east")
                .withProperty("hinge", "left").withProperty("open", "false");
        for (int z = -1; z <= 1; z++) {
            world.set(3, 1, z, base.withProperty("half", "lower"));
            world.set(3, 2, z, base.withProperty("half", "upper"));
        }
        return world;
    }

    private static final List<String> EXPECTED_OPENS_NOTHING = List.of(
            "plain|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "plain|w1.4|PARTIAL;0,1,0",
            "door_east_left|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "door_east_left|w1.4|PARTIAL;0,1,0",
            "door_north_right|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "door_north_right|w1.4|PARTIAL;0,1,0",
            "door_open|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "door_open|w1.4|PARTIAL;0,1,0",
            "iron_door|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "iron_door|w1.4|PARTIAL;0,1,0",
            "trapdoor_east_bottom|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "trapdoor_east_bottom|w1.4|PARTIAL;0,1,0",
            "trapdoor_north_top|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "trapdoor_north_top|w1.4|PARTIAL;0,1,0",
            "trapdoor_open_east|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "trapdoor_open_east|w1.4|PARTIAL;0,1,0",
            "trapdoor_head|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "trapdoor_head|w1.4|PARTIAL;0,1,0",
            "gate_closed|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "gate_closed|w1.4|PARTIAL;0,1,0",
            "gate_open|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "gate_open|w1.4|PARTIAL;0,1,0",
            "fence|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "fence|w1.4|PARTIAL;0,1,0",
            "mixed|w0.6|FOUND;0,1,0;0,1,-1;1,1,-2;2,1,-3;3,1,-3;4,1,-3;5,1,-2;6,1,-2;6,1,-1;6,1,0",
            "mixed|w1.4|FOUND;-1,1,-1;-1,1,-2;0,1,-3;0,1,-4;1,1,-4;2,1,-4;3,1,-4;4,1,-4;5,1,-4;6,1,-3;6,1,-2;6,1,-1;6,1,0",
            "wide_gap_trapdoor|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "wide_gap_trapdoor|w1.4|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "wide_gap_door|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "wide_gap_door|w1.4|PARTIAL;0,1,0;1,1,0"
    );

    private static final List<String> EXPECTED_OPENS_DOORS = List.of(
            "plain|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "plain|w1.4|PARTIAL;0,1,0",
            "door_east_left|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "door_east_left|w1.4|PARTIAL;0,1,0",
            // Changed: hung across the corridor rather than in it, this
            // door swings its panel over the whole lane. The route used
            // to cross and wedge the mob against its own door.
            "door_north_right|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0;3,1,0",
            "door_north_right|w1.4|PARTIAL;0,1,0",
            "door_open|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "door_open|w1.4|PARTIAL;0,1,0",
            "iron_door|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "iron_door|w1.4|PARTIAL;0,1,0",
            "trapdoor_east_bottom|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "trapdoor_east_bottom|w1.4|PARTIAL;0,1,0",
            "trapdoor_north_top|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "trapdoor_north_top|w1.4|PARTIAL;0,1,0",
            "trapdoor_open_east|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "trapdoor_open_east|w1.4|PARTIAL;0,1,0",
            "trapdoor_head|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "trapdoor_head|w1.4|PARTIAL;0,1,0",
            "gate_closed|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "gate_closed|w1.4|PARTIAL;0,1,0",
            "gate_open|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "gate_open|w1.4|PARTIAL;0,1,0",
            "fence|w0.6|PARTIAL;0,1,0;1,1,0;2,1,0",
            "fence|w1.4|PARTIAL;0,1,0",
            "mixed|w0.6|FOUND;0,1,0;0,1,-1;1,1,-2;2,1,-3;3,1,-3;4,1,-3;5,1,-2;6,1,-2;6,1,-1;6,1,0",
            "mixed|w1.4|FOUND;-1,1,-1;-1,1,-2;0,1,-3;0,1,-4;1,1,-4;2,1,-4;3,1,-4;4,1,-4;5,1,-4;6,1,-3;6,1,-2;6,1,-1;6,1,0",
            "wide_gap_trapdoor|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "wide_gap_trapdoor|w1.4|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            "wide_gap_door|w0.6|FOUND;0,1,0;1,1,0;2,1,0;3,1,0;4,1,0;5,1,0;6,1,0",
            // Changed: a two-cell footprint centred on the gap covers the
            // middle door's panel, so no orientation of it leaves room.
            "wide_gap_door|w1.4|PARTIAL;0,1,0;1,1,0"
    );
}
