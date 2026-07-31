pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
    }
}

rootProject.name = "minestom-pathfinding"

// Use the adjacent checkout during development. A source archive remains
// buildable elsewhere when a published minestomVersion is supplied.
if (file("../Minestom/settings.gradle.kts").exists()) {
    includeBuild("../Minestom")
}
