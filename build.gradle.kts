plugins {
    `java-library`
    `maven-publish`
    id("me.champeau.jmh") version "0.7.3"
}

jmh {
    jmhVersion = "1.37"
    warmupIterations = 3
    iterations = 5
    fork = 1
    timeOnIteration = "750ms"
    warmup = "750ms"
    benchmarkMode = listOf("avgt")
    timeUnit = "us"
    profilers = listOf("gc")
    failOnError = true
    resultFormat = "JSON"
    resultsFile = layout.buildDirectory.file("reports/jmh/results.json")
}

group = "ca.atlasengine"
version = "10.2.0"

repositories {
    mavenCentral()
}

java {
    toolchain.languageVersion = JavaLanguageVersion.of(25)
    withSourcesJar()
    withJavadocJar()
}

publishing {
    publications.create<MavenPublication>("maven") {
        groupId = "ca.atlasengine"
        artifactId = "pathfinding"
        version = project.version.toString()

        from(components["java"])
    }

    repositories {
        maven {
            name = "AtlasEngine"
            url = uri("https://reposilite.atlasengine.ca/public")
            credentials(PasswordCredentials::class)
            authentication {
                create<BasicAuthentication>("basic")
            }
        }
    }
}

dependencies {
    api("net.minestom:minestom:${providers.gradleProperty("minestomVersion").getOrElse("dev")}")

    testImplementation("net.minestom:testing:${providers.gradleProperty("minestomVersion").getOrElse("dev")}")
    testImplementation(platform("org.junit:junit-bom:6.0.1"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
    systemProperty("minestom.inside-test", "true")
    maxHeapSize = "1g"
}

tasks.withType<JavaCompile>().configureEach {
    options.encoding = "UTF-8"
    options.release = 25
}

// Runnable demo server. It compiles against the published packages only, so
// `build` failing here means the public API moved under an integration.
val examples = sourceSets.create("examples")
configurations[examples.implementationConfigurationName]
    .extendsFrom(configurations.api.get())

dependencies {
    add(examples.implementationConfigurationName, sourceSets.main.get().output)
    add(examples.runtimeOnlyConfigurationName,
        "ch.qos.logback:logback-classic:1.5.37")
}

tasks.register<JavaExec>("runExamples") {
    description = "Boots the demo navigation server on localhost:25565"
    group = "application"
    classpath = examples.runtimeClasspath
    mainClass = "ca.atlasengine.pathfinding.examples.NavigationDemoServer"
    maxHeapSize = "1g"
    standardInput = System.`in`
}

// Examples are teaching code, so they must break the build rather than rot.
tasks.named("build") { dependsOn(examples.classesTaskName) }

// Ordinary builds compile against the adjacent ../Minestom checkout, which is
// whatever commit that working copy sits on. Consumers instead resolve the
// version named in gradle.properties, which is what the published POM records.
// Nothing normally compiles against that version, so an API present in the
// checkout but absent from the release would ship broken. This stages the main
// sources into a directory with no Minestom sibling, where the composite
// substitution cannot apply, and compiles them against the real artifact.
val publishedCheckDir = layout.buildDirectory.dir("published-check")
val minestomRelease =
    providers.gradleProperty("minestomVersion").getOrElse("dev")

val stagePublishedCheck = tasks.register<Sync>("stagePublishedCheck") {
    description = "Stages main sources for the published-dependency check"
    group = LifecycleBasePlugin.VERIFICATION_GROUP
    into(publishedCheckDir)
    from(layout.projectDirectory.dir("src/main/java")) { into("src/main/java") }
    // Without this the staged build script survives a version change and the
    // check silently re-verifies whatever it verified last time.
    inputs.property("minestomRelease", minestomRelease)
    doLast {
        val dir = publishedCheckDir.get().asFile
        // No includeBuild here: this build has no Minestom sibling to substitute.
        File(dir, "settings.gradle.kts").writeText(
            """
            rootProject.name = "published-dependency-check"
            """.trimIndent() + "\n"
        )
        File(dir, "build.gradle.kts").writeText(
            """
            plugins { `java-library` }
            repositories { mavenCentral() }
            java { toolchain.languageVersion = JavaLanguageVersion.of(25) }
            dependencies { api("net.minestom:minestom:$minestomRelease") }
            tasks.withType<JavaCompile>().configureEach {
                options.encoding = "UTF-8"
                options.release = 25
            }
            // Proves the check resolved the release rather than a substitute.
            tasks.register("reportResolvedMinestom") {
                val files = configurations.named("compileClasspath")
                doLast {
                    files.get().filter { it.name.contains("minestom") }
                        .forEach { println("resolved: " + it.name) }
                }
            }
            """.trimIndent() + "\n"
        )
    }
}

val verifyPublishedDependency =
    tasks.register<GradleBuild>("verifyPublishedDependency") {
    description =
        "Compiles the library against the published net.minestom:minestom:$minestomRelease"
    group = LifecycleBasePlugin.VERIFICATION_GROUP
    dependsOn(stagePublishedCheck)
    dir = publishedCheckDir.get().asFile
    tasks = listOf("reportResolvedMinestom", "compileJava")
}

// The published POM is a promise about a version nothing else compiles
// against, so check it every build rather than on request.
tasks.named("check") { dependsOn(verifyPublishedDependency) }

tasks.register<JavaExec>("profilePathfinding") {
    description = "Profiles representative pathfinding workloads with JFR"
    group = "verification"
    dependsOn(tasks.testClasses)
    classpath = sourceSets.test.get().runtimeClasspath
    mainClass = "ca.atlasengine.pathfinding.PathfindingProfileMain"
    maxHeapSize = "1g"
    args(layout.buildDirectory.file("profiles/pathfinding.jfr").get().asFile)
}

tasks.register<JavaExec>("profileClimbing") {
    description = "Profiles the guarded 32-block climbable workload with JFR"
    group = "verification"
    dependsOn(tasks.testClasses)
    classpath = sourceSets.test.get().runtimeClasspath
    mainClass = "ca.atlasengine.pathfinding.PathfindingProfileMain"
    maxHeapSize = "1g"
    args(layout.buildDirectory.file("profiles/climbing.jfr").get().asFile,
            "climbing")
}

tasks.register<Test>("pathfindingAccuracy") {
    description = "Runs deterministic optimality, terrain, output, and live E2E accuracy fixtures"
    group = LifecycleBasePlugin.VERIFICATION_GROUP
    dependsOn(tasks.testClasses)
    testClassesDirs = sourceSets.test.get().output.classesDirs
    classpath = sourceSets.test.get().runtimeClasspath
    useJUnitPlatform()
    systemProperty("minestom.inside-test", "true")
    maxHeapSize = "1g"
    filter {
        includeTestsMatching("ca.atlasengine.pathfinding.HardMapOptimalityTest")
        includeTestsMatching("ca.atlasengine.pathfinding.PathOutputAuditTest")
        includeTestsMatching("ca.atlasengine.pathfinding.AdvancedTerrainTest")
        includeTestsMatching("ca.atlasengine.pathfinding.GroundNodePathfinderTest")
        includeTestsMatching(
                "ca.atlasengine.pathfinding.internal.search.SpatialPathfinderTest")
        includeTestsMatching("ca.atlasengine.pathfinding.MinestomNavigationE2ETest")
    }
}
