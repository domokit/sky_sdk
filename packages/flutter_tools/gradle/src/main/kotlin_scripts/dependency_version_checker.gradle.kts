// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import org.gradle.api.JavaVersion
import org.jetbrains.kotlin.gradle.plugin.KotlinAndroidPluginWrapper

// This buildscript block supplies dependencies for this file's own import
// declarations above. It exists solely for compatibility with projects that
// have not migrated to declaratively apply the Flutter Gradle Plugin;
// for those that have, FGP's `build.gradle.kts`  takes care of this.
buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        // When bumping, also update:
        //  * ndkVersion in FlutterExtension in packages/flutter_tools/gradle/src/main/groovy/flutter.groovy
        //  * AGP version in the buildscript block in packages/flutter_tools/gradle/src/main/groovy/flutter.groovy
        //  * AGP version constants in packages/flutter_tools/lib/src/android/gradle_utils.dart
        //  * AGP version in dependencies block in packages/flutter_tools/gradle/build.gradle.kts
        classpath("com.android.tools.build:gradle:7.3.0")
        classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:1.7.10")
    }
}

apply<FlutterDependencyCheckerPlugin>()

class FlutterDependencyCheckerPlugin : Plugin<Project> {
    override fun apply(project: Project) {
        DependencyVersionChecker.checkDependencyVersions(project)
    }
}

class DependencyVersionChecker {
    companion object {
        private const val GRADLE_NAME: String = "Gradle"
        private const val JAVA_NAME: String = "Java"
        private const val AGP_NAME: String = "Android Gradle Plugin"
        private const val KGP_NAME: String = "Kotlin"

        // String constant that defines the name of the Gradle extra property that we set when
        // detecting that the project is using versions outside of Flutter's support range.
        // https://docs.gradle.org/current/kotlin-dsl/gradle/org.gradle.api/-project/index.html#-2107180640%2FProperties%2F-1867656071.
        private const val OUT_OF_SUPPORT_RANGE_PROPERTY = "usesUnsupportedDependencyVersions"

        // The following messages represent best effort guesses at where a Flutter developer should
        // look to upgrade a dependency that is below the corresponding threshold. Developers can
        // change some of these locations, so they are not guaranteed to be accurate.
        private fun getPotentialGradleFix(projectDirectory: String): String {
            return "Your project's gradle version is typically " +
                "defined in the gradle wrapper file. By default, this can be found at " +
                "$projectDirectory/gradle/wrapper/gradle-wrapper.properties. \n" +
                "For more information, see https://docs.gradle.org/current/userguide/gradle_wrapper.html.\n"
        }

        // The potential java fix does not make use of the project directory,
        // so it left as a constant.
        private const val POTENTIAL_JAVA_FIX: String =
            "The Java version used by Flutter can be " +
                "set with `flutter config --jdk-dir=<path>`. \nFor more information about how Flutter " +
                "chooses which version of Java to use, see the --jdk-dir section of the " +
                "output of `flutter config -h`.\n"

        private fun getPotentialAGPFix(projectDirectory: String): String {
            return "Your project's AGP version is typically " +
                "defined in the plugins block of the `settings.gradle` file " +
                "($projectDirectory/settings.gradle), by a plugin with the id of " +
                "com.android.application. \nIf you don't see a plugins block, your project " +
                "was likely created with an older template version. In this case it is most " +
                "likely defined in the top-level build.gradle file " +
                "($projectDirectory/build.gradle) by the following line in the dependencies" +
                " block of the buildscript: \"classpath 'com.android.tools.build:gradle:<version>'\".\n"
        }

        private fun getPotentialKGPFix(projectDirectory: String): String {
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
        val warnGradleVersion: Version = Version(7, 4, 2)
        val errorGradleVersion: Version = Version(7, 0, 2)

        val warnJavaVersion: JavaVersion = JavaVersion.VERSION_11
        val errorJavaVersion: JavaVersion = JavaVersion.VERSION_1_1

        val warnAGPVersion: Version = Version(7, 3, 1)
        val errorAGPVersion: Version = Version(7, 0, 0)

        val warnKGPVersion: Version = Version(1, 8, 10)
        val errorKGPVersion: Version = Version(1, 7, 0)

        /**
         * Checks if the project's Android build time dependencies are each within the respective
         * version range that we support. When we can't find a version for a given dependency
         * we treat it as within the range for the purpose of this check.
         */
        fun checkDependencyVersions(project: Project) {
            project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, false)
            var agpVersion: Version?
            var kgpVersion: Version?

            checkGradleVersion(getGradleVersion(project), project)
            checkJavaVersion(getJavaVersion(), project)
            agpVersion = getAGPVersion(project)
            if (agpVersion != null) {
                checkAGPVersion(agpVersion, project)
            } else {
                project.logger.error(
                    "Warning: unable to detect project AGP version. Skipping " +
                        "version checking. \nThis may be because you have applied AGP after the Flutter Gradle Plugin."
                )
            }

            kgpVersion = getKGPVersion(project)
            if (kgpVersion != null) {
                checkKGPVersion(kgpVersion, project)
            }
            // KGP is not required, so don't log any warning if we can't find the version.
        }

        // https://docs.gradle.org/current/kotlin-dsl/gradle/org.gradle.api.invocation/-gradle/index.html#-837060600%2FFunctions%2F-1793262594
        fun getGradleVersion(project: Project): Version {
            val untrimmedGradleVersion: String = project.gradle.getGradleVersion()
            // Trim to handle candidate gradle versions (example 7.6-rc-4). This means we treat all
            // candidate versions of gradle as the same as their base version
            // (i.e., "7.6"="7.6-rc-4").
            return Version.fromString(untrimmedGradleVersion.substringBefore('-'))
        }

        // https://docs.gradle.org/current/kotlin-dsl/gradle/org.gradle.api/-java-version/index.html#-1790786897%2FFunctions%2F-1793262594
        fun getJavaVersion(): JavaVersion {
            return JavaVersion.current()
        }

        // This approach is taken from AGP's own version checking plugin:
        // https://android.googlesource.com/platform/tools/base/+/1839aa23b8dc562005e2f0f0cc8e8b4c5caa37d0/build-system/gradle-core/src/main/java/com/android/build/gradle/internal/utils/agpVersionChecker.kt#58.
        fun getAGPVersion(project: Project): Version? {
            val agpPluginName: String = "com.android.base"
            val agpVersionFieldName: String = "ANDROID_GRADLE_PLUGIN_VERSION"
            var agpVersion: Version?
            try {
                agpVersion =
                    Version.fromString(
                        project.plugins.getPlugin(agpPluginName)::class.java.classLoader.loadClass(
                            com.android.Version::class.java.name
                        ).fields.find { it.name == agpVersionFieldName }!!
                            .get(null) as String
                    )
            } catch (ignored: ClassNotFoundException) {
                // Use deprecated Version class as it exists in older AGP (com.android.Version) does
                // not exist in those versions.
                @Suppress("deprecation")
                agpVersion =
                    Version.fromString(
                        project.plugins.getPlugin(agpPluginName)::class.java.classLoader.loadClass(
                            com.android.builder.model.Version::class.java.name
                        ).fields.find { it.name == agpVersionFieldName }!!
                            .get(null) as String
                    )
            }
            return agpVersion
        }

        fun getKGPVersion(project: Project): Version? {
            val kotlinVersionProperty: String = "kotlin_version"
            val firstKotlinVersionFieldName: String = "pluginVersion"
            val secondKotlinVersionFieldName: String = "kotlinPluginVersion"
            // This property corresponds to application of the Kotlin Gradle plugin in the
            // top-level build.gradle file.
            if (project.hasProperty(kotlinVersionProperty)) {
                return Version.fromString(project.properties.get(kotlinVersionProperty) as String)
            }
            val kotlinPlugin =
                project.getPlugins()
                    .findPlugin(KotlinAndroidPluginWrapper::class.java)
            val versionfield =
                kotlinPlugin?.javaClass?.kotlin?.members?.first {
                    it.name == firstKotlinVersionFieldName || it.name == secondKotlinVersionFieldName
                }
            val versionString = versionfield?.call(kotlinPlugin)
            if (versionString == null) {
                return null
            } else {
                return Version.fromString(versionString as String)
            }
        }

        private fun getErrorMessage(
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

        private fun getWarnMessage(
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

        fun checkGradleVersion(
            version: Version,
            project: Project
        ) {
            if (version < errorGradleVersion) {
                val errorMessage: String =
                    getErrorMessage(
                        GRADLE_NAME,
                        version.toString(),
                        errorGradleVersion.toString(),
                        getPotentialGradleFix(project.getRootDir().getPath())
                    )
                project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true)
                throw DependencyValidationException(errorMessage)
            } else if (version < warnGradleVersion) {
                val warnMessage: String =
                    getWarnMessage(
                        GRADLE_NAME,
                        version.toString(),
                        warnGradleVersion.toString(),
                        getPotentialGradleFix(project.getRootDir().getPath())
                    )
                project.logger.error(warnMessage)
            }
        }

        fun checkJavaVersion(
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

        fun checkAGPVersion(
            version: Version,
            project: Project
        ) {
            if (version < errorAGPVersion) {
                val errorMessage: String =
                    getErrorMessage(
                        AGP_NAME,
                        version.toString(),
                        errorAGPVersion.toString(),
                        getPotentialAGPFix(project.getRootDir().getPath())
                    )
                project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true)
                throw DependencyValidationException(errorMessage)
            } else if (version < warnAGPVersion) {
                val warnMessage: String =
                    getWarnMessage(
                        AGP_NAME,
                        version.toString(),
                        warnAGPVersion.toString(),
                        getPotentialAGPFix(project.getRootDir().getPath())
                    )
                project.logger.error(warnMessage)
            }
        }

        fun checkKGPVersion(
            version: Version,
            project: Project
        ) {
            if (version < errorKGPVersion) {
                val errorMessage: String =
                    getErrorMessage(
                        KGP_NAME,
                        version.toString(),
                        errorKGPVersion.toString(),
                        getPotentialKGPFix(project.getRootDir().getPath())
                    )
                project.extra.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true)
                throw DependencyValidationException(errorMessage)
            } else if (version < warnKGPVersion) {
                val warnMessage: String =
                    getWarnMessage(
                        KGP_NAME,
                        version.toString(),
                        warnKGPVersion.toString(),
                        getPotentialKGPFix(project.getRootDir().getPath())
                    )
                project.logger.error(warnMessage)
            }
        }
    }
}

// Helper class to parse the versions that are provided as plain strings (Gradle, Kotlin) and
// perform easy comparisons. All versions will have a major, minor, and patch value. These values
// default to 0 when they are not provided or are otherwise unparseable.
// For example the version strings "8.2", "8.2.2hfd", and "8.2.0" would parse to the same version.
class Version(val major: Int, val minor: Int, val patch: Int) : Comparable<Version> {
    companion object {
        fun fromString(version: String): Version {
            val asList: List<String> = version.split(".")
            val convertedToNumbers: List<Int> = asList.map { it.toIntOrNull() ?: 0 }
            return Version(
                major = convertedToNumbers.getOrElse(0, { 0 }),
                minor = convertedToNumbers.getOrElse(1, { 0 }),
                patch = convertedToNumbers.getOrElse(2, { 0 })
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
        return major.toString() + "." + minor.toString() + "." + patch.toString()
    }
}

// Custom error for when the dependency_version_checker.kts script finds a dependency out of
// the defined support range.
class DependencyValidationException(message: String? = null, cause: Throwable? = null) : Exception(message, cause) {
    constructor(cause: Throwable) : this(null, cause)
}
