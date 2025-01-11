import org.gradle.api.Plugin
import org.gradle.api.initialization.Settings
import org.gradle.kotlin.dsl.extra
import java.io.File
import java.util.Properties

class FlutterAppPluginLoaderPlugin : Plugin<Settings> {
    override fun apply(settings: Settings) {
        val flutterProjectRoot = settings.settingsDir.parentFile

        if (!settings.extra.has("flutterSdkPath")) {
            val properties = Properties()
            val localPropertiesFile = File(settings.rootProject.projectDir, "local.properties")
            localPropertiesFile.inputStream().use { properties.load(it) }
            settings.extra["flutterSdkPath"] = properties.getProperty("flutter.sdk")
            requireNotNull(settings.extra["flutterSdkPath"]) { "flutter.sdk not set in local.properties" }
        }

        val nativePlugins: List<APlugin> = NativePluginLoader.getPlugins(flutterProjectRoot)
        nativePlugins.forEach { androidPlugin ->
            val pluginDirectory = File(androidPlugin.path, "android")
            check(pluginDirectory.exists())
            settings.include(":${androidPlugin.name}")
            settings.project(":${androidPlugin.name}").projectDir = pluginDirectory
        }
    }
}
