package com.flutter.gradle

import com.android.build.api.dsl.ApplicationExtension
import org.gradle.api.Project

// TODO(gmackall): maybe migrate this to a package-level function when FGP conversion is done.
object BaseApplicationNameHandler {
    internal const val DEFAULT_BASE_APPLICATION_NAME: String = "android.app.Application"

    internal const val GRADLE_BASE_APPLICATION_NAME_PROPERTY: String = "base-application-name"

    @JvmStatic fun setBaseName(project: Project) {
        // Only set the base application name for apps, skip otherwise (LibraryExtension, DynamicFeatureExtension).
        val androidComponentsExtension: ApplicationExtension =
            project.extensions.findByType(ApplicationExtension::class.java) ?: return

        // Setting to android.app.Application is the same as omitting the attribute.
        var baseApplicationName: String = DEFAULT_BASE_APPLICATION_NAME

        // Respect this property if it set by the Flutter tool.
        if (project.hasProperty(GRADLE_BASE_APPLICATION_NAME_PROPERTY)) {
            baseApplicationName = project.property(GRADLE_BASE_APPLICATION_NAME_PROPERTY).toString()
        }

        androidComponentsExtension.defaultConfig.manifestPlaceholders["applicationName"] =
            baseApplicationName
    }
}
