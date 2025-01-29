import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import java.io.File

/*
object NativePluginLoader {
    private const val FLUTTER_PLUGINS_DEPENDENCIES_FILE_NAME = ".flutter-plugins-dependencies"
    private val json: Json = Json { ignoreUnknownKeys = true }

    private var parsedFlutterPluginsDependencies: FlutterPluginsDependencies? = null

    /**
     * Returns the list of plugins that support the Android platform.
     *
     * In particular, it omits plugins that have no native build (such as a Dart-only implementation
     * of a federated android plugin).
     */
    fun getPlugins(flutterSourceDirectory: File): List<APlugin> {
        val flutterPluginsDependencies = getDependenciesMetadata(flutterSourceDirectory) ?: return listOf()
        return flutterPluginsDependencies.plugins.android.filter { it.nativeBuild == true }
    }

    /**
     * Parses `<project-src>/.flutter-plugins-dependencies` into a concrete data class and returns it.
     *
     * Consider a `.flutter-plugins-dependencies` file with the following content:
     *
     * ```json
     * {
     *     "plugins": {
     *       "android": [
     *         {
     *           "name": "plugin-a",
     *           "path": "/path/to/plugin-a",
     *           "dependencies": ["plugin-b", "plugin-c"],
     *           "native_build": true
     *           "dev_dependency": false
     *         },
     *         {
     *           "name": "plugin-b",
     *           "path": "/path/to/plugin-b",
     *           "dependencies": ["plugin-c"],
     *           "native_build": true
     *           "dev_dependency": false
     *         },
     *         {
     *           "name": "plugin-c",
     *           "path": "/path/to/plugin-c",
     *           "dependencies": [],
     *           "native_build": true
     *           "dev_dependency": false
     *         },
     *         {
     *           "name": "plugin-d",
     *           "path": "/path/to/plugin-d",
     *           "dependencies": [],
     *           "native_build": true
     *           "dev_dependency": true
     *         },
     *       ],
     *     },
     *     "dependencyGraph": [
     *       {
     *         "name": "plugin-a",
     *         "dependencies": ["plugin-b","plugin-c"]
     *       },
     *       {
     *         "name": "plugin-b",
     *         "dependencies": ["plugin-c"]
     *       },
     *       {
     *         "name": "plugin-c",
     *         "dependencies": []
     *       },
     *       {
     *         "name": "plugin-d",
     *         "dependencies": []
     *       }
     *     ]
     * }
     * ```
     *
     * This means, `plugin-a` depends on `plugin-b` and `plugin-c`.
     * - `plugin-b` depends on `plugin-c`.
     * - `plugin-c` doesn't depend on anything.
     * - `plugin-d` also doesn't depend on anything, but it is a dev dependency to the Flutter project, so it is marked as such.
     */
    fun getDependenciesMetadata(flutterSourceDirectory: File): FlutterPluginsDependencies? {
        if (parsedFlutterPluginsDependencies != null) {
            return parsedFlutterPluginsDependencies
        }

        val pluginsDependencyFile = File(flutterSourceDirectory, FLUTTER_PLUGINS_DEPENDENCIES_FILE_NAME)
        if (pluginsDependencyFile.exists()) {
            val jsonContents = pluginsDependencyFile.readText()
            parsedFlutterPluginsDependencies = json.decodeFromString<FlutterPluginsDependencies>(jsonContents)
            return parsedFlutterPluginsDependencies
        }
        return null
    }
}

@Serializable
data class FlutterPluginsDependencies(
    @SerialName("plugins") val plugins: Plugins,
    @SerialName("dependencyGraph") val dependencyGraph: List<DependencyEntry>
)

@Serializable
data class DependencyEntry(
    @SerialName("name") val name: String,
    @SerialName("dependencies") val dependencies: List<String>,
)

@Serializable
data class Plugins(
    @SerialName("android") val android: List<APlugin>,
    // There are also plugins for other platforms in this JSON, but we're not interested in them.
)

/**
 * The word "plugin" is quite overloaded in this area. There are Gradle plugins and Flutter plugins.
 *
 * Example JSON form of this class might look like this:
 *
 * ```json
 * {
 *     "name": "plugin-a",
 *     "path": "/path/to/plugin-a",
 *     "dependencies": ["plugin-b", "plugin-c"],
 *     "native_build": true
 *     "dev_dependency": false
 * }
 * ```
 */
@Serializable
data class APlugin(
    @SerialName("name") val name: String,
    @SerialName("path") val path: String,
    @SerialName("dependencies") val dependencies: List<String>,
    /**
     * Serial name of this field must match `_kFlutterPluginsHasNativeBuildKey` defined in
     * `packages/flutter_tools/lib/src/flutter_plugins.dart`.
     */
    @SerialName("native_build") val nativeBuild: Boolean? = null,
    @SerialName("dev_dependency") val devDependency: Boolean? = null,
)

*/
