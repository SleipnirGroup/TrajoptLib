plugins {
    `java-library`
    `maven-publish`
}

repositories {
    mavenCentral()
}

dependencies {
    testImplementation("org.junit.jupiter:junit-jupiter:5.8.1")
}

java {
    withJavadocJar()
    withSourcesJar()
}

tasks.named<Test>("test") {
    useJUnitPlatform()
    systemProperty("java.library.path", "/Users/jlbabilino/Documents/TripleHelix/Programming/Repositories/HelixTrajectoryJ/helixtrajectorycpp/build/src/cpp")
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "org.team2363"
            artifactId = "helixtrajectory"
            version = "0.0.0"

            from(components["java"])
        }
    }
    repositories {
        maven {
            name = "github"
            // id = "github"
            url = uri("https://maven.pkg.github.com/jlbabilino/HelixTrajectory")
        }
    }
}