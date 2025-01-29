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

        // val nativePlugins = NativePluginLoaderProxy.nativePluginLoader.getPlugins(flutterProjectRoot)
        // check(nativePlugins != null) { "nativePlugins cannot be null" }

        val nativePlugins = OldNativePluginLoader.getInstance().getPlugins(flutterProjectRoot)

        // val nativePlugins: List<APlugin> = NativePluginLoader.getPlugins(flutterProjectRoot)
        nativePlugins.forEach { androidPlugin: Map<String, Any> ->
            val androidPluginPath = androidPlugin["path"] as? String
            checkNotNull(androidPluginPath) { "androidPluginPath path cannot be null" }
            val androidPluginName = androidPlugin["name"] as? String
            checkNotNull(androidPluginName) { "androidPluginName cannot be null" }

            val pluginDirectory = File(androidPluginPath, "android")
            check(pluginDirectory.exists())
            settings.include(":${androidPluginName}")
            settings.project(":${androidPluginName}").projectDir = pluginDirectory
        }
    }
}
