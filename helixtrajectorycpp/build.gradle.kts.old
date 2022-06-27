plugins {
    `cpp-library`
}

repositories {
}

dependencies {
    // implementation(project(":helixtrajectorycpp"))
}

library {
    linkage.set(listOf(Linkage.SHARED))
    privateHeaders {
        from("${org.gradle.internal.jvm.Jvm.current().javaHome}/include")
        from("${org.gradle.internal.jvm.Jvm.current().javaHome}/include/darwin")
        from("${rootProject.projectDir}/casadi/include")
        from("${projectDir}/src/main/headers")
    }
    tasks.withType(CppCompile::class.java).configureEach {
        compilerArgs.addAll(toolChain.map { toolChain ->
            when (toolChain) {
                is Gcc, is Clang -> listOf(
                        "-std=c++11"
                    )
                is VisualCpp -> listOf()
                else -> listOf()
            }
        })
    }
    tasks.withType(LinkSharedLibrary::class.java).configureEach {
        linkerArgs.addAll(toolChain.map { toolChain ->
            when (toolChain) {
                is Gcc, is Clang -> listOf(
                        "-L${rootProject.projectDir}/casadi",
                        "-lcasadi",
                        "-lcasadi_nlpsol_ipopt"
                    )
                is VisualCpp -> listOf()
                else -> listOf()
            }
        })
    }
    tasks.withType(CreateStaticLibrary::class.java).configureEach {
        staticLibArgs.addAll(toolChain.map { toolChain ->
            when (toolChain) {
                is Gcc, is Clang -> listOf(
                        "-L${rootProject.projectDir}/casadi",
                        "-lcasadi",
                        "-lcasadi_nlpsol_ipopt"
                    )
                is VisualCpp -> listOf()
                else -> listOf()
            }
        })
    }
}