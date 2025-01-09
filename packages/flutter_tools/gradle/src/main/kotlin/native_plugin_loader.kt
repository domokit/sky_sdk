import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.jsonObject
import java.io.File

object NativePluginLoader {

    // This string must match _kFlutterPluginsHasNativeBuildKey defined in
    // packages/flutter_tools/lib/src/flutter_plugins.dart.
    private const val nativeBuildKey = "native_build"
    private const val flutterPluginsDependenciesFile = ".flutter-plugins-dependencies"

    private var parsedFlutterPluginsDependencies: FlutterPluginsDependencies? = null

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
    fun getPlugins(flutterSourceDirectory: File): List<APlugin> {
        println("NativePluginLoader.getPlugins() called")

        val flutterPluginsDependencies = getDependenciesMetadata(flutterSourceDirectory) ?: return listOf()

        println("Got flutterPluginsDependencies: $flutterPluginsDependencies")

        val nativePlugins = flutterPluginsDependencies.plugins.android.filter { it.nativeBuild == true }
        return nativePlugins

//        // Includes the Flutter plugins that support the Android platform.
//        for (androidBuild in flutterPluginsDependencies.plugins.android) {
//            println("HERE 3")
//            // require(androidBuild is Map<*, *>)
//            println("HERE 4")
//
//            // The property types can be found in _filterPluginsByPlatform defined in
//            // packages/flutter_tools/lib/src/flutter_plugins.dart.
//            val androidPluginName = androidBuild["name"]
//            println("HERE 4aaa, type: ${androidPluginName!!.javaClass.name}")
//            require(androidPluginName is String) { "plugin.name is not a String but ${androidPluginName.javaClass.name} instead" }
//            println("HERE 4a")
//            val androidPluginPath = androidBuild["path"]
//            require(androidPluginPath is String) { "plugin.path is not a String but ${androidPluginPath?.javaClass?.name} instead" }
//            println("HERE 4b")
//            val androidPluginDependencies = androidBuild["dependencies"]
//            require(androidPluginDependencies is List<*>) { "plugin.path is not a List but ${androidPluginDependencies?.javaClass?.name} instead" }
//            println("HERE 4c")
//            val androidPluginDevDependency = androidBuild["dev_dependency"]
//            require(androidPluginDevDependency is Boolean) { "plugin.path is not a Boolean" }
//
//            println("HERE 5")
//
//            // Skip plugins that have no native build (such as a Dart-only implementation
//            // of a federated androidBuild).
//            println("yo, androidBuild type is: ${androidBuild.javaClass.name}")
//            // val needsBuild = if (androidBuild.containsKey(nativeBuildKey)) androidBuild[nativeBuildKey] else true
//            if (androidBuild.nativeBuild == true) {
//                println("yo, androidBuild type is: ${androidBuild.javaClass.name}")
//                nativePlugins.add(androidBuild)
//            }
//        }
//
//        println("HERE 10")
//
//        return nativePlugins
    }

    /**
     * Parses `<project-src>/.flutter-plugins-dependencies`
     */
    fun getDependenciesMetadata(flutterSourceDirectory: File): FlutterPluginsDependencies? {
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
            val jsonElement: JsonElement = json.parseToJsonElement(jsonContents)
            val jsonObject: JsonObject = jsonElement.jsonObject

//            val map: Map<String, Any> = mutableMapOf()
//            for ((key, value) in jsonObject) {
//                when (value) {
//                    is JsonPrimitive
//                }
//            }

//            println("HERE 1")
            val obj = json.decodeFromString<FlutterPluginsDependencies>(jsonContents)
            parsedFlutterPluginsDependencies = obj
//            val a = jsonObject.toMap()
//            println("HERE 2, typeof a: ${a.javaClass.name}, a: $a")
//            a["dependencyGraph"]
//            return a

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

// The word "plugin" is quite overloaded in this area. There are Gradle plugins and Flutter plugins.
@Serializable
data class APlugin(
    @SerialName("name") val name: String,
    @SerialName("path") val path: String,
    @SerialName("dependencies") val dependencies: List<String>,
    @SerialName("native_build") val nativeBuild: Boolean? = null, // Optional field
    @SerialName("dev_dependency") val devDependency: Boolean? = null, // Required field
)
