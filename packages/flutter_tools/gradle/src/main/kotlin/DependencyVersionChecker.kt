package com.flutter.gradle

import androidx.annotation.VisibleForTesting
import com.android.build.api.AndroidPluginVersion
import com.android.build.api.variant.AndroidComponentsExtension
import org.gradle.api.JavaVersion
import org.gradle.api.Project
import org.gradle.kotlin.dsl.extra
import org.jetbrains.kotlin.gradle.plugin.KotlinAndroidPluginWrapper

object DependencyVersionChecker {
    @VisibleForTesting internal const val GRADLE_NAME: String = "Gradle"

    @VisibleForTesting internal const val JAVA_NAME: String = "Java"

    @VisibleForTesting internal const val AGP_NAME: String = "Android Gradle Plugin"

    @VisibleForTesting internal const val KGP_NAME: String = "Kotlin"

    // String constant that defines the name of the Gradle extra property that we set when
    // detecting that the project is using versions outside of Flutter's support range.
    // https://docs.gradle.org/current/kotlin-dsl/gradle/org.gradle.api/-project/index.html#-2107180640%2FProperties%2F-1867656071.
    @VisibleForTesting internal const val OUT_OF_SUPPORT_RANGE_PROPERTY = "usesUnsupportedDependencyVersions"

    // The following messages represent best effort guesses at where a Flutter developer should
    // look to upgrade a dependency that is below the corresponding threshold. Developers can
    // change some of these locations, so they are not guaranteed to be accurate.
    @VisibleForTesting internal fun getPotentialGradleFix(projectDirectory: String): String {
        return "Your project's gradle version is typically " +
            "defined in the gradle wrapper file. By default, this can be found at " +
            "$projectDirectory/gradle/wrapper/gradle-wrapper.properties. \n" +
            "For more information, see https://docs.gradle.org/current/userguide/gradle_wrapper.html.\n"
    }

    // The potential java fix does not make use of the project directory,
    // so it left as a constant.
    @VisibleForTesting internal const val POTENTIAL_JAVA_FIX: String =
        "The Java version used by Flutter can be " +
            "set with `flutter config --jdk-dir=<path>`. \nFor more information about how Flutter " +
            "chooses which version of Java to use, see the --jdk-dir section of the " +
            "output of `flutter config -h`.\n"

    @VisibleForTesting internal fun getPotentialAGPFix(projectDirectory: String): String {
        return "Your project's AGP version is typically " +
            "defined in the plugins block of the `settings.gradle` file " +
            "($projectDirectory/settings.gradle), by a plugin with the id of " +
            "com.android.application. \nIf you don't see a plugins block, your project " +
            "was likely created with an older template version. In this case it is most " +
            "likely defined in the top-level build.gradle file " +
            "($projectDirectory/build.gradle) by the following line in the dependencies" +
            " block of the buildscript: \"classpath 'com.android.tools.build:gradle:<version>'\".\n"
    }

    @VisibleForTesting internal fun getPotentialKGPFix(projectDirectory: String): String {
        return "Your project's KGP version is typically " +
            "defined in the plugins block of the `settings.gradle` file " +
            "($projectDirectory/settings.gradle), by a plugin with the id of " +
            "org.jetbrains.kotlin.android. \nIf you don't see a plugins block, your project " +
            "was likely created with an older template version, in which case it is most " +
            "likely defined in the top-level build.gradle file " +
            "($projectDirectory/build.gradle) by the ext.kotlin_version property.\n"
    }

    // The following versions define our support policy for Gradle, Java, AGP, and KGP.
    // Before updating any "error" version, ensure that you have updated the corresponding
    // "warn" version for a full release to provide advanced warning. See
    // flutter.dev/go/android-dependency-versions for more.
    @VisibleForTesting internal val warnGradleVersion: Version = Version(7, 4, 2)

    @VisibleForTesting internal val errorGradleVersion: Version = Version(7, 0, 2)

    @VisibleForTesting internal val warnJavaVersion: JavaVersion = JavaVersion.VERSION_11

    @VisibleForTesting internal val errorJavaVersion: JavaVersion = JavaVersion.VERSION_1_1

    @VisibleForTesting internal val warnAGPVersion: AndroidPluginVersion = AndroidPluginVersion(7, 3, 1)

    @VisibleForTesting internal val errorAGPVersion: AndroidPluginVersion = AndroidPluginVersion(7, 0, 0)

    @VisibleForTesting internal val warnKGPVersion: Version = Version(1, 8, 10)

    @VisibleForTesting internal val errorKGPVersion: Version = Version(1, 7, 0)

    /**
     * Checks if the project's Android build time dependencies are each within the respective
     * version range that we support. When we can't find a version for a given dependency
     * we treat it as within the range for the purpose of this check.
     */
    @JvmStatic fun checkDependencyVersions(project: Project) {
        project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, false)

        checkGradleVersion(getGradleVersion(project), project)
        checkJavaVersion(getJavaVersion(), project)
        val agpVersion: AndroidPluginVersion? = getAGPVersion(project)
        if (agpVersion != null) {
            checkAGPVersion(agpVersion, project)
        } else {
            project.logger.error(
                "Warning: unable to detect project AGP version. Skipping " +
                    "version checking. \nThis may be because you have applied AGP after the Flutter Gradle Plugin."
            )
        }

        val kgpVersion: Version? = getKGPVersion(project)
        if (kgpVersion != null) {
            checkKGPVersion(kgpVersion, project)
        }
        // KGP is not required, so don't log any warning if we can't find the version.
    }

    // https://docs.gradle.org/current/kotlin-dsl/gradle/org.gradle.api.invocation/-gradle/index.html#-837060600%2FFunctions%2F-1793262594
    @VisibleForTesting internal fun getGradleVersion(project: Project): Version {
        val untrimmedGradleVersion: String = project.gradle.gradleVersion
        // Trim to handle candidate gradle versions (example 7.6-rc-4). This means we treat all
        // candidate versions of gradle as the same as their base version
        // (i.e., "7.6"="7.6-rc-4").
        return Version.fromString(untrimmedGradleVersion.substringBefore('-'))
    }

    // https://docs.gradle.org/current/kotlin-dsl/gradle/org.gradle.api/-java-version/index.html#-1790786897%2FFunctions%2F-1793262594
    @VisibleForTesting internal fun getJavaVersion(): JavaVersion {
        return JavaVersion.current()
    }

    @VisibleForTesting internal fun getAGPVersion(project: Project): AndroidPluginVersion? {
        val androidPluginVersion: AndroidPluginVersion? =
            project.extensions.findByType(
                AndroidComponentsExtension::class.java
            )?.pluginVersion
        return androidPluginVersion
    }

    // TODO(gmackall): AGP has a getKotlinAndroidPluginVersion(), and KGP has a
    //                 getKotlinPluginVersion(). Consider replacing this implementation with one of
    //                 those.
    @VisibleForTesting internal fun getKGPVersion(project: Project): Version? {
        val kotlinVersionProperty = "kotlin_version"
        val firstKotlinVersionFieldName = "pluginVersion"
        val secondKotlinVersionFieldName = "kotlinPluginVersion"
        // This property corresponds to application of the Kotlin Gradle plugin in the
        // top-level build.gradle file.
        if (project.hasProperty(kotlinVersionProperty)) {
            return Version.fromString(project.properties[kotlinVersionProperty] as String)
        }
        val kotlinPlugin =
            project.plugins
                .findPlugin(KotlinAndroidPluginWrapper::class.java)
        val versionField =
            kotlinPlugin?.javaClass?.kotlin?.members?.first {
                it.name == firstKotlinVersionFieldName || it.name == secondKotlinVersionFieldName
            }
        val versionString = versionField?.call(kotlinPlugin)
        return if (versionString == null) {
            null
        } else {
            Version.fromString(versionString as String)
        }
    }

    @VisibleForTesting internal fun getErrorMessage(
        dependencyName: String,
        versionString: String,
        errorVersion: String,
        potentialFix: String
    ): String {
        return "Error: Your project's $dependencyName version ($versionString) is lower " +
            "than Flutter's minimum supported version of $errorVersion. Please upgrade " +
            "your $dependencyName version. \nAlternatively, use the flag " +
            "\"--android-skip-build-dependency-validation\" to bypass this check.\n\n" +
            "Potential fix: $potentialFix"
    }

    @VisibleForTesting internal fun getWarnMessage(
        dependencyName: String,
        versionString: String,
        warnVersion: String,
        potentialFix: String
    ): String {
        return "Warning: Flutter support for your project's $dependencyName version " +
            "($versionString) will soon be dropped. Please upgrade your $dependencyName " +
            "version to a version of at least $warnVersion soon." +
            "\nAlternatively, use the flag \"--android-skip-build-dependency-validation\"" +
            " to bypass this check.\n\nPotential fix: $potentialFix"
    }

    @VisibleForTesting internal fun checkGradleVersion(
        version: Version,
        project: Project
    ) {
        if (version < errorGradleVersion) {
            val errorMessage: String =
                getErrorMessage(
                    GRADLE_NAME,
                    version.toString(),
                    errorGradleVersion.toString(),
                    getPotentialGradleFix(project.rootDir.path)
                )
            project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true)
            throw DependencyValidationException(errorMessage)
        } else if (version < warnGradleVersion) {
            val warnMessage: String =
                getWarnMessage(
                    GRADLE_NAME,
                    version.toString(),
                    warnGradleVersion.toString(),
                    getPotentialGradleFix(project.rootDir.path)
                )
            project.logger.error(warnMessage)
        }
    }

    @VisibleForTesting internal fun checkJavaVersion(
        version: JavaVersion,
        project: Project
    ) {
        if (version < errorJavaVersion) {
            val errorMessage: String =
                getErrorMessage(
                    JAVA_NAME,
                    version.toString(),
                    errorJavaVersion.toString(),
                    POTENTIAL_JAVA_FIX
                )
            project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true)
            throw DependencyValidationException(errorMessage)
        } else if (version < warnJavaVersion) {
            val warnMessage: String =
                getWarnMessage(
                    JAVA_NAME,
                    version.toString(),
                    warnJavaVersion.toString(),
                    POTENTIAL_JAVA_FIX
                )
            project.logger.error(warnMessage)
        }
    }

    @VisibleForTesting internal fun checkAGPVersion(
        androidPluginVersion: AndroidPluginVersion,
        project: Project
    ) {
        if (androidPluginVersion < errorAGPVersion) {
            val errorMessage: String =
                getErrorMessage(
                    AGP_NAME,
                    androidPluginVersion.toString(),
                    errorAGPVersion.toString(),
                    getPotentialAGPFix(project.rootDir.path)
                )
            project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true)
            throw DependencyValidationException(errorMessage)
        } else if (androidPluginVersion < warnAGPVersion) {
            val warnMessage: String =
                getWarnMessage(
                    AGP_NAME,
                    androidPluginVersion.toString(),
                    warnAGPVersion.toString(),
                    getPotentialAGPFix(project.rootDir.path)
                )
            project.logger.error(warnMessage)
        }
    }

    @VisibleForTesting internal fun checkKGPVersion(
        version: Version,
        project: Project
    ) {
        if (version < errorKGPVersion) {
            val errorMessage: String =
                getErrorMessage(
                    KGP_NAME,
                    version.toString(),
                    errorKGPVersion.toString(),
                    getPotentialKGPFix(project.rootDir.path)
                )
            project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true)
            throw DependencyValidationException(errorMessage)
        } else if (version < warnKGPVersion) {
            val warnMessage: String =
                getWarnMessage(
                    KGP_NAME,
                    version.toString(),
                    warnKGPVersion.toString(),
                    getPotentialKGPFix(project.rootDir.path)
                )
            project.logger.error(warnMessage)
        }
    }
}

// Helper class to parse the versions that are provided as plain strings (Gradle, Kotlin) and
// perform easy comparisons. All versions will have a major, minor, and patch value. These values
// default to 0 when they are not provided or are otherwise unparseable.
// For example the version strings "8.2", "8.2.2hfd", and "8.2.0" would parse to the same version.
internal class Version(val major: Int, val minor: Int, val patch: Int) : Comparable<Version> {
    companion object {
        fun fromString(version: String): Version {
            val asList: List<String> = version.split(".")
            val convertedToNumbers: List<Int> = asList.map { it.toIntOrNull() ?: 0 }
            return Version(
                major = convertedToNumbers.getOrElse(0) { 0 },
                minor = convertedToNumbers.getOrElse(1) { 0 },
                patch = convertedToNumbers.getOrElse(2) { 0 }
            )
        }
    }

    override fun compareTo(other: Version): Int {
        if (major != other.major) {
            return major - other.major
        }
        if (minor != other.minor) {
            return minor - other.minor
        }
        if (patch != other.patch) {
            return patch - other.patch
        }
        return 0
    }

    override fun toString(): String {
        return "$major.$minor.$patch"
    }
}

// Custom error for when the dependency_version_checker.kts script finds a dependency out of
// the defined support range.
@VisibleForTesting internal class DependencyValidationException(
    message: String? = null,
    cause: Throwable? = null
) : Exception(message, cause)
