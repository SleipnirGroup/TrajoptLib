plugins {
    // `java-library`
    `java`
    `application`
}

repositories {
    // Use Maven Central for resolving dependencies.
    mavenCentral()
}

dependencies {
    // Use JUnit Jupiter for testing.
    testImplementation("org.junit.jupiter:junit-jupiter:5.8.1")
}

java {
}

application {
    mainClassName = "org.team2363.helixtrajectory.Main"
}

tasks.withType(JavaExec::class.java).configureEach {
    systemProperty("java.library.path", "/Users/jlbabilino/Documents/TripleHelix/Programming/Repositories/HelixTrajectoryJ/helixtrajectorycpp/build/src/cpp")
}

tasks.named<Test>("test") {
    // Use JUnit Platform for unit tests.
    useJUnitPlatform()
}