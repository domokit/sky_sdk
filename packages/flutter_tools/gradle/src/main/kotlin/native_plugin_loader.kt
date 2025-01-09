import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.jsonObject
import java.io.File

object NativePluginLoader {

    // This string must match _kFlutterPluginsHasNativeBuildKey defined in
    // packages/flutter_tools/lib/src/flutter_plugins.dart.
    private const val nativeBuildKey = "native_build"
    private const val flutterPluginsDependenciesFile = ".flutter-plugins-dependencies"

    private var parsedFlutterPluginsDependencies: Map<String, Any>? = null

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
    fun getPlugins(flutterSourceDirectory: File): List<Map<String, Any>> {
        println("HERE 1")

        val nativePlugins: MutableList<Map<String, Any>> = mutableListOf()
        val depsMetadata = getDependenciesMetadata(flutterSourceDirectory) ?: return nativePlugins


        val allPlugins = depsMetadata["plugins"]
        require(allPlugins is Map<*, *>)
        val onlyAndroidPlugins = allPlugins["android"]
        require(onlyAndroidPlugins is List<*>)

        println("HERE 2")

        // Includes the Flutter plugins that support the Android platform.
        for (androidBuild in onlyAndroidPlugins) {
            println("HERE 3")
            require(androidBuild is Map<*, *>)
            println("HERE 4")

            // The property types can be found in _filterPluginsByPlatform defined in
            // packages/flutter_tools/lib/src/flutter_plugins.dart.
            val androidPluginName = androidBuild["name"]
            println("HERE 4aaa, type: ${androidPluginName!!.javaClass.name}")
            require(androidPluginName is String) { "plugin.name is not a String but ${androidPluginName.javaClass.name} instead" }
            println("HERE 4a")
            val androidPluginPath = androidBuild["path"]
            require(androidPluginPath is String) { "plugin.path is not a String but ${androidPluginPath?.javaClass?.name} instead" }
            println("HERE 4b")
            val androidPluginDependencies = androidBuild["dependencies"]
            require(androidPluginDependencies is List<*>) { "plugin.path is not a List but ${androidPluginDependencies?.javaClass?.name} instead" }
            println("HERE 4c")
            val androidPluginDevDependency = androidBuild["dev_dependency"]
            require(androidPluginDevDependency is Boolean) { "plugin.path is not a Boolean" }

            println("HERE 5")

            // Skip plugins that have no native build (such as a Dart-only implementation
            // of a federated androidBuild).
            println("yo, androidBuild type is: ${androidBuild.javaClass.name}")
            val needsBuild = if (androidBuild.containsKey(nativeBuildKey)) androidBuild[nativeBuildKey] else true
            if (needsBuild == true) {
                println("yo, androidBuild type is: ${androidBuild.javaClass.name}")
                nativePlugins.add(androidBuild as Map<String, Any>)
            }
        }

        println("HERE 10")

        return nativePlugins
    }

    /**
     * Parses `<project-src>/.flutter-plugins-dependencies`
     */
    fun getDependenciesMetadata(flutterSourceDirectory: File): Map<String, Any>? {
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
            // FIXME(bartekpacia): THIS IS BROKEN
            parsedFlutterPluginsDependencies = jsonObject.toMap()
//            val a = jsonObject.toMap()
//            println("HERE 2, typeof a: ${a.javaClass.name}, a: $a")
//            a["dependencyGraph"]
//            return a

            return parsedFlutterPluginsDependencies
        }
        return null
    }
}

//@Serializable
//data class DependenciesMetadata(
//    val plugins: Plugins,
//)
//
//@Serializable
//data class Plugins(
//    val android: List<Plugin>,
//)
//
//@Serializable
//data class Plugin(
//    @SerializedName("name") val name: String,
//    @SerializedName("path") val path: String,
//    @SerializedName("dependencies") val dependencies: List<String>,
//    @SerializedName("native_build") val nativeBuild: Boolean? = null, // Optional field
//    @SerializedName("dev_dependency") val devDependency: Boolean? = null, // Required field
//)
