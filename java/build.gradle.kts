plugins {
    `java-library`
    `maven-publish`
}

repositories {
    mavenCentral()
}

dependencies {
    implementation("com.fasterxml.jackson.core:jackson-databind:2.15.2")

    testImplementation("org.junit.jupiter:junit-jupiter:5.8.1")
}

java {
    withJavadocJar()
    withSourcesJar()
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "org.sleipnirgroup"
            artifactId = "trajoptlib" + project.properties["platform_id"];
            version = "0.0.0-pre0"

            from(components["java"])
        }
    }
    repositories {
        maven {
            name = "github"
            url = uri("https://maven.pkg.github.com/SleipnirGroup/TrajoptLib")
            credentials(PasswordCredentials::class)
        }
    }
}
