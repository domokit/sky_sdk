import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import org.gradle.internal.impldep.com.google.gson.annotations.SerializedName
import java.io.File

object NativePluginLoader {
    private const val flutterPluginsDependenciesFile = ".flutter-plugins-dependencies"

    private var parsedFlutterPluginsDependencies: DependenciesMetadata? = null

    private val json: Json = Json { ignoreUnknownKeys = true }

    /**
     * Gets the list of plugins that support the Android platform.
     * The list contains map elements with the following content:
     * {
     *     "name": "plugin-a",
     *     "path": "/path/to/plugin-a",
     *     "dependencies": ["plugin-b", "plugin-c"],
     *     "native_build": true
     *     "dev_dependency": false
     * }
     */
    fun getPlugins(flutterSourceDirectory: File): List<Plugin> {
        val nativePlugins = mutableListOf<Plugin>()
        val depsMetadata = getDependenciesMetadata(flutterSourceDirectory) ?: return nativePlugins

        val androidPlugins = depsMetadata.plugins.android
        // Includes the Flutter plugins that support the Android platform.
        for (androidBuild in androidPlugins) {
            // The property types can be found in _filterPluginsByPlatform defined in
            // packages/flutter_tools/lib/src/flutter_plugins.dart.

            // Skip plugins that have no native build (such as a Dart-only implementation
            // of a federated androidBuild).
            val needsBuild = androidBuild.nativeBuild ?: true
            if (needsBuild) {
                nativePlugins.add(androidBuild)
            }
        }
        return nativePlugins
    }

    /**
     * Parses `<project-src>/.flutter-plugins-dependencies`
     */
    fun getDependenciesMetadata(flutterSourceDirectory: File): DependenciesMetadata? {
        // Consider a `.flutter-plugins-dependencies` file with the following content:
        // {
        //     "plugins": {
        //       "android": [
        //         {
        //           "name": "plugin-a",
        //           "path": "/path/to/plugin-a",
        //           "dependencies": ["plugin-b", "plugin-c"],
        //           "native_build": true
        //           "dev_dependency": false
        //         },
        //         {
        //           "name": "plugin-b",
        //           "path": "/path/to/plugin-b",
        //           "dependencies": ["plugin-c"],
        //           "native_build": true
        //           "dev_dependency": false
        //         },
        //         {
        //           "name": "plugin-c",
        //           "path": "/path/to/plugin-c",
        //           "dependencies": [],
        //           "native_build": true
        //           "dev_dependency": false
        //         },
        //         {
        //           "name": "plugin-d",
        //           "path": "/path/to/plugin-d",
        //           "dependencies": [],
        //           "native_build": true
        //           "dev_dependency": true
        //         },
        //       ],
        //     },
        //     "dependencyGraph": [
        //       {
        //         "name": "plugin-a",
        //         "dependencies": ["plugin-b","plugin-c"]
        //       },
        //       {
        //         "name": "plugin-b",
        //         "dependencies": ["plugin-c"]
        //       },
        //       {
        //         "name": "plugin-c",
        //         "dependencies": []
        //       },
        //       {
        //         "name": "plugin-d",
        //         "dependencies": []
        //       }
        //     ]
        // }
        // This means, `plugin-a` depends on `plugin-b` and `plugin-c`.
        // `plugin-b` depends on `plugin-c`.
        // `plugin-c` doesn't depend on anything.
        // `plugin-d` also doesn't depend on anything, but it is a dev
        // dependency to the Flutter project, so it is marked as such.

        if (parsedFlutterPluginsDependencies != null) {
            return parsedFlutterPluginsDependencies
        }

        val pluginsDependencyFile = File(flutterSourceDirectory, flutterPluginsDependenciesFile)
        if (pluginsDependencyFile.exists()) {
            val jsonContents = pluginsDependencyFile.readText()
            parsedFlutterPluginsDependencies = json.decodeFromString(jsonContents)
            return parsedFlutterPluginsDependencies
        }
        return null
    }
}

@Serializable
data class DependenciesMetadata(
    val plugins: Plugins,
)

@Serializable
data class Plugins(
    val android: List<Plugin>,
)

@Serializable
data class Plugin(
    @SerializedName("name") val name: String,
    @SerializedName("path") val path: String,
    @SerializedName("dependencies") val dependencies: List<String>,
    @SerializedName("native_build") val nativeBuild: Boolean? = null, // Optional field
    @SerializedName("dev_dependency") val devDependency: Boolean? = null, // Required field
)
