<div id="top"></div>

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/AtlasEngineCa/Pathfinding">
    <h3 align="center">Pathfinding</h3>
  </a>

  <p align="center">
    Entity navigation for Minestom
    <br />
    <br />
    <br />
    <a href="https://github.com/AtlasEngineCa/Pathfinding/issues">Report Bug</a>
    ·
    <a href="https://github.com/AtlasEngineCa/Pathfinding/issues">Request Feature</a>
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project

A standalone, deterministic entity-navigation library for
[Minestom](https://github.com/Minestom/Minestom).

Path computation is independent of entities, instances, schedulers, and ticks.
It uses real block collision shapes, immutable results, and hard search limits,
and is safe to call concurrently: a pathfinder holds no mutable search state.

Supported navigation modes for Minecraft 26.2 gameplay:
- Ground
- Water
- Flying
- Amphibious
- Wall-climbing

Platform jumping, planned climbing, block manipulation, and shared-mesh planning
are opt-in and never inferred from terrain.

<!-- GETTING STARTED -->
## Getting Started

A full, runnable example server can be found [here](https://github.com/AtlasEngineCa/Pathfinding/tree/master/src/examples/java)

One `NavigationSystem` per server, one controller per mob, and you call
`tick()`:

```java
// Once per server.
NavigationSystem navigation = NavigationSystem.create();

// Once per mob.
EntityNavigationController controller = navigation.controller(mob);
controller.moveTo(destination);

// Every tick, from the entity or instance tick thread.
controller.tick();

// When the mob despawns, and at shutdown.
controller.close();
navigation.close();
```

Searching happens off the tick thread, following happens on it, and you never
wait for a path. `moveTo` is cheap enough to call every tick: the mob keeps its
current route until the replacement lands, so a chased target never stands
still.

`controller.state()` reports progress: `COMPLETED` arrived, `PARTIAL` ran out
short of the goal, `STUCK` cannot walk the route it has.

That is the whole everyday API; see the [wiki](https://github.com/AtlasEngineCa/Pathfinding/wiki).

### Adding as a dependency

Add the following to your `build.gradle.kts` file:

```
repositories {
    maven("https://reposilite.atlasengine.ca/public")
}
```

Add the library as a dependency
```
dependencies {
    implementation("ca.atlasengine:pathfinding:<version>")
}
```

The lastest version number can be found [here](https://reposilite.atlasengine.ca/#/public/ca/atlasengine/pathfinding)

### Demo server

```
../Minestom/gradlew runExamples
```

Join `localhost:25565` in offline mode and type `/nav`: `walk`, `chase`, `jump`,
`doors`, `climb`, `swim`, `fly`, `custom`, `crowd <n>`, `shed`, `metrics`,
`stop`. Each teleports you to a course and spawns the mobs for that capability.

## [Documentation](https://github.com/AtlasEngineCa/Pathfinding/wiki)

The wiki starts with the simplest use and goes deeper:
[Getting Started](https://github.com/AtlasEngineCa/Pathfinding/wiki/Getting-Started),
[Reacting to Navigation](https://github.com/AtlasEngineCa/Pathfinding/wiki/Reacting-to-Navigation),
[Seeing the Path](https://github.com/AtlasEngineCa/Pathfinding/wiki/Seeing-the-Path),
[Zones and Influences](https://github.com/AtlasEngineCa/Pathfinding/wiki/Zones-and-Influences),
[Jumping, Climbing and Doors](https://github.com/AtlasEngineCa/Pathfinding/wiki/Jumping,-Climbing-and-Doors),
[Custom Mobs](https://github.com/AtlasEngineCa/Pathfinding/wiki/Custom-Mobs),
[Terrain and World Changes](https://github.com/AtlasEngineCa/Pathfinding/wiki/Terrain-and-World-Changes),
[Planning Without the Follower](https://github.com/AtlasEngineCa/Pathfinding/wiki/Planning-Without-the-Follower),
[Running at Scale](https://github.com/AtlasEngineCa/Pathfinding/wiki/Running-at-Scale),
[Shared Mesh](https://github.com/AtlasEngineCa/Pathfinding/wiki/Shared-Mesh).

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/AtlasEngineCa/Pathfinding.svg?style=for-the-badge
[contributors-url]: https://github.com/AtlasEngineCa/Pathfinding/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/AtlasEngineCa/Pathfinding.svg?style=for-the-badge
[forks-url]: https://github.com/AtlasEngineCa/Pathfinding/network/members
[stars-shield]: https://img.shields.io/github/stars/AtlasEngineCa/Pathfinding.svg?style=for-the-badge
[stars-url]: https://github.com/AtlasEngineCa/Pathfinding/stargazers
[issues-shield]: https://img.shields.io/github/issues/AtlasEngineCa/Pathfinding.svg?style=for-the-badge
[issues-url]: https://github.com/AtlasEngineCa/Pathfinding/issues
[license-shield]: https://img.shields.io/github/license/AtlasEngineCa/Pathfinding?style=for-the-badge
[license-url]: https://github.com/AtlasEngineCa/Pathfinding/blob/master/LICENSE
