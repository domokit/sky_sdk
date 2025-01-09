import org.gradle.api.Plugin
import org.gradle.api.initialization.Settings
import org.gradle.kotlin.dsl.apply
import org.gradle.kotlin.dsl.extra
import java.io.File
import java.nio.file.Paths
import java.util.Properties
import NativePluginLoader

class FlutterAppPluginLoaderPlugin : Plugin<Settings> {

    override fun apply(settings: Settings): Unit {
        val flutterProjectRoot = settings.settingsDir.parentFile

        if (!settings.extra.has("flutterSdkPath")) {
            val properties = Properties()
            val localPropertiesFile = File(settings.rootProject.projectDir, "local.properties")
            localPropertiesFile.inputStream().use { properties.load(it) }
            settings.extra["flutterSdkPath"] = properties.getProperty("flutter.sdk")
            requireNotNull(settings.extra["flutterSdkPath"]) { "flutter.sdk not set in local.properties" }
        }

        // Load shared gradle functions
        settings.apply(
            from = Paths.get(
                settings.extra["flutterSdkPath"] as String,
                "packages",
                "flutter_tools",
                "gradle",
                "src",
                "main",
                "groovy",
                "native_plugin_loader.groovy"
            )
        )

        val nativePluginLoader: NativePluginLoader = settings.extra["nativePluginLoader"] as NativePluginLoader
        val nativePlugins: List<Map<String, Any>> = nativePluginLoader.getPlugins(flutterProjectRoot)
        nativePlugins.forEach { androidPlugin ->
            val pluginDirectory = File(androidPlugin["path"] as String, "android")
            check(pluginDirectory.exists())
            settings.include(":${androidPlugin["name"]}")
            settings.project(":${androidPlugin["name"]}").projectDir = pluginDirectory
        }
    }
}
