import java.nio.file.Paths

// This script is used to warm the Gradle cache by downloading the Flutter dependencies
// used during the build. This script is invoked when `flutter precache` is run.
//
// Command:
//  gradle -b <flutter-sdk>packages/flutter_tools/gradle/resolve_dependencies.gradle
//      resolveDependencies
//
// This way, Gradle can run with the `--offline` flag later on to eliminate any
// network request during the build process.
//
// This includes:
//   1. The embedding
//   2. libflutter.so

val storageUrl: String = System.getenv("FLUTTER_STORAGE_BASE_URL") ?: "https://storage.googleapis.com"

val flutterRoot = projectDir.parentFile?.parentFile?.parentFile
requireNotNull(flutterRoot) { "Flutter root directory not found!" }

require(flutterRoot.isDirectory) { "Flutter root is not a valid directory!" }

val engineVersion =
    Paths.get(
        flutterRoot.absolutePath,
        "bin",
        "internal",
        "engine.version"
    )
        .toFile()
        .readText()
        .trim()

var engineRealm =
    Paths.get(
        flutterRoot.absolutePath,
        "bin",
        "internal",
        "engine.realm"
    )
        .toFile()
        .readText()
        .trim()

if (engineRealm.isNotEmpty()) {
    engineRealm += "/"
}

repositories {
    google()
    mavenCentral()
    maven {
        url = uri("$storageUrl/${engineRealm}download.flutter.io")
    }
}

configurations {
    create("flutterRelease") {
        extendsFrom(configurations.getByName("releaseImplementation"))
    }
    create("flutterDebug") {
        extendsFrom(configurations.getByName("debugImplementation"))
    }
    create("flutterProfile") {
        extendsFrom(configurations.getByName("debugImplementation"))
    }
}

dependencies {
    "flutterRelease"("io.flutter:flutter_embedding_release:1.0.0-$engineVersion")
    "flutterRelease"("io.flutter:armeabi_v7a_release:1.0.0-$engineVersion")
    "flutterRelease"("io.flutter:arm64_v8a_release:1.0.0-$engineVersion")

    "flutterProfile"("io.flutter:flutter_embedding_profile:1.0.0-$engineVersion")
    "flutterProfile"("io.flutter:armeabi_v7a_profile:1.0.0-$engineVersion")
    "flutterProfile"("io.flutter:arm64_v8a_profile:1.0.0-$engineVersion")

    "flutterDebug"("io.flutter:flutter_embedding_debug:1.0.0-$engineVersion")
    "flutterDebug"("io.flutter:armeabi_v7a_debug:1.0.0-$engineVersion")
    "flutterDebug"("io.flutter:arm64_v8a_debug:1.0.0-$engineVersion")
    "flutterDebug"("io.flutter:x86_debug:1.0.0-$engineVersion")
    "flutterDebug"("io.flutter:x86_64_debug:1.0.0-$engineVersion")
}

tasks.register("resolveDependencies") {
    doLast {
        configurations.forEach { configuration ->
            if (configuration.name.startsWith("flutter")) {
                configuration.resolve()
            }
        }
    }
}
