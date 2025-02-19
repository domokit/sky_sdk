package com.flutter.gradle

import com.android.build.api.AndroidPluginVersion
import com.android.build.api.variant.AndroidComponentsExtension
import com.flutter.gradle.DependencyVersionChecker.AGP_NAME
import com.flutter.gradle.DependencyVersionChecker.GRADLE_NAME
import com.flutter.gradle.DependencyVersionChecker.JAVA_NAME
import com.flutter.gradle.DependencyVersionChecker.KGP_NAME
import com.flutter.gradle.DependencyVersionChecker.OUT_OF_SUPPORT_RANGE_PROPERTY
import com.flutter.gradle.DependencyVersionChecker.POTENTIAL_JAVA_FIX
import com.flutter.gradle.DependencyVersionChecker.errorAGPVersion
import com.flutter.gradle.DependencyVersionChecker.errorGradleVersion
import com.flutter.gradle.DependencyVersionChecker.errorKGPVersion
import com.flutter.gradle.DependencyVersionChecker.getErrorMessage
import com.flutter.gradle.DependencyVersionChecker.getPotentialAGPFix
import com.flutter.gradle.DependencyVersionChecker.getPotentialGradleFix
import com.flutter.gradle.DependencyVersionChecker.getPotentialKGPFix
import com.flutter.gradle.DependencyVersionChecker.getWarnMessage
import com.flutter.gradle.DependencyVersionChecker.warnAGPVersion
import com.flutter.gradle.DependencyVersionChecker.warnGradleVersion
import com.flutter.gradle.DependencyVersionChecker.warnJavaVersion
import com.flutter.gradle.DependencyVersionChecker.warnKGPVersion
import io.mockk.every
import io.mockk.mockk
import io.mockk.mockkStatic
import io.mockk.verify
import org.gradle.api.JavaVersion
import org.gradle.api.Project
import org.gradle.api.logging.Logger
import org.gradle.api.plugins.ExtraPropertiesExtension
import org.gradle.internal.extensions.core.extra
import kotlin.test.Test
import kotlin.test.assertFailsWith

const val FAKE_PROJECT_ROOT_DIR = "/fake/root/dir"

// The following values will need to be modified when the corresponding "warn$DepName" versions
// are updated in DependencyVersionChecker.kt
const val SUPPORTED_GRADLE_VERSION: String = "7.4.2"
val SUPPORTED_JAVA_VERSION: JavaVersion = JavaVersion.VERSION_11
val SUPPORTED_AGP_VERSION: AndroidPluginVersion = AndroidPluginVersion(7, 3, 1)
const val SUPPORTED_KGP_VERSION: String = "1.8.10"

class DependencyVersionCheckerTest {
    @Test
    fun `AGP version in error range results in DependencyValidationException`() {
        val exampleErrorAgpVersion = AndroidPluginVersion(4, 2, 0)
        val mockProject = MockProjectFactory.createMockProjectWithSpecifiedDependencyVersions(agpVersion = exampleErrorAgpVersion)

        val mockExtraPropertiesExtension = mockProject.extra
        every { mockExtraPropertiesExtension.set(any(), any()) } returns Unit

        val dependencyValidationException =
            assertFailsWith<DependencyValidationException> { DependencyVersionChecker.checkDependencyVersions(mockProject) }
        assert(
            dependencyValidationException.message ==
                getErrorMessage(
                    AGP_NAME,
                    exampleErrorAgpVersion.toString(),
                    errorAGPVersion.toString(),
                    getPotentialAGPFix(FAKE_PROJECT_ROOT_DIR)
                )
        )
        verify { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true) }
    }

    @Test
    fun `AGP version in warn range results in warning logs`() {
        val exampleWarnAgpVersion = AndroidPluginVersion(7, 1, 0)
        val mockProject = MockProjectFactory.createMockProjectWithSpecifiedDependencyVersions(agpVersion = exampleWarnAgpVersion)

        val mockExtraPropertiesExtension = mockProject.extra
        every { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, false) } returns Unit
        val mockLogger = mockProject.logger
        every { mockLogger.error(any()) } returns Unit

        DependencyVersionChecker.checkDependencyVersions(mockProject)
        verify {
            mockLogger.error(
                getWarnMessage(
                    AGP_NAME,
                    exampleWarnAgpVersion.toString(),
                    warnAGPVersion.toString(),
                    getPotentialAGPFix(FAKE_PROJECT_ROOT_DIR)
                )
            )
        }
        verify(exactly = 0) { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true) }
    }

    @Test
    fun `KGP version in error range results in DependencyValidationException`() {
        val exampleErrorKgpVersion = "1.6.0"
        val mockProject = MockProjectFactory.createMockProjectWithSpecifiedDependencyVersions(kgpVersion = exampleErrorKgpVersion)

        val mockExtraPropertiesExtension = mockProject.extra
        every { mockExtraPropertiesExtension.set(any(), any()) } returns Unit

        val dependencyValidationException =
            assertFailsWith<DependencyValidationException> { DependencyVersionChecker.checkDependencyVersions(mockProject) }

        println(dependencyValidationException.message)
        assert(
            dependencyValidationException.message ==
                getErrorMessage(
                    KGP_NAME,
                    exampleErrorKgpVersion,
                    errorKGPVersion.toString(),
                    getPotentialKGPFix(FAKE_PROJECT_ROOT_DIR)
                )
        )
        verify { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true) }
    }

    @Test
    fun `KGP version in warn range results in warning logs`() {
        val exampleWarnKgpVersion = "1.8.0"
        val mockProject = MockProjectFactory.createMockProjectWithSpecifiedDependencyVersions(kgpVersion = exampleWarnKgpVersion)

        val mockExtraPropertiesExtension = mockProject.extra
        every { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, false) } returns Unit
        val mockLogger = mockProject.logger
        every { mockLogger.error(any()) } returns Unit

        DependencyVersionChecker.checkDependencyVersions(mockProject)
        verify {
            mockLogger.error(
                getWarnMessage(
                    KGP_NAME,
                    exampleWarnKgpVersion,
                    warnKGPVersion.toString(),
                    getPotentialKGPFix(FAKE_PROJECT_ROOT_DIR)
                )
            )
        }
        verify(exactly = 0) { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true) }
    }

    // No test for Java version in error range, as the lowest supported Java version is also the
    // lowest possible.

    @Test
    fun `Java version in warn range results in warning logs`() {
        val exampleWarnJavaVersion = JavaVersion.VERSION_1_8
        val mockProject = MockProjectFactory.createMockProjectWithSpecifiedDependencyVersions(javaVersion = exampleWarnJavaVersion)

        val mockExtraPropertiesExtension = mockProject.extra
        every { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, false) } returns Unit
        val mockLogger = mockProject.logger
        every { mockLogger.error(any()) } returns Unit

        DependencyVersionChecker.checkDependencyVersions(mockProject)
        verify {
            mockLogger.error(
                getWarnMessage(
                    JAVA_NAME,
                    exampleWarnJavaVersion.toString(),
                    warnJavaVersion.toString(),
                    POTENTIAL_JAVA_FIX
                )
            )
        }
        verify(exactly = 0) { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true) }
    }

    @Test
    fun `Gradle version in error range results in DependencyValidationException`() {
        val exampleErrorGradleVersion = "7.0.0"
        val mockProject = MockProjectFactory.createMockProjectWithSpecifiedDependencyVersions(gradleVersion = exampleErrorGradleVersion)

        val mockExtraPropertiesExtension = mockProject.extra
        every { mockExtraPropertiesExtension.set(any(), any()) } returns Unit

        val dependencyValidationException =
            assertFailsWith<DependencyValidationException> { DependencyVersionChecker.checkDependencyVersions(mockProject) }

        assert(
            dependencyValidationException.message ==
                getErrorMessage(
                    GRADLE_NAME,
                    exampleErrorGradleVersion,
                    errorGradleVersion.toString(),
                    getPotentialGradleFix(FAKE_PROJECT_ROOT_DIR)
                )
        )
        verify { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true) }
    }

    @Test
    fun `Gradle version in warn range results in warning logs`() {
        val exampleWarnGradleVersion = "7.4.0"
        val mockProject = MockProjectFactory.createMockProjectWithSpecifiedDependencyVersions(gradleVersion = exampleWarnGradleVersion)

        val mockExtraPropertiesExtension = mockProject.extra
        every { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, false) } returns Unit
        val mockLogger = mockProject.logger
        every { mockLogger.error(any()) } returns Unit

        DependencyVersionChecker.checkDependencyVersions(mockProject)
        verify {
            mockLogger.error(
                getWarnMessage(
                    GRADLE_NAME,
                    exampleWarnGradleVersion,
                    warnGradleVersion.toString(),
                    getPotentialGradleFix(FAKE_PROJECT_ROOT_DIR)
                )
            )
        }
        verify(exactly = 0) { mockExtraPropertiesExtension.set(OUT_OF_SUPPORT_RANGE_PROPERTY, true) }
    }
}

// There isn't a way to create a real org.gradle.api.Project object for testing unfortunately, so
// these tests rely heavily on mocking.
//
// TODO(gmackall): We should consider adding functional tests built on top of a testing framework
//  perhaps like
//  https://github.com/autonomousapps/dependency-analysis-gradle-plugin/tree/main/testkit
//  as a way to fill this gap in testing (combined with moving functionality to individual tasks
//  that can be tested independently).
object MockProjectFactory {
    fun createMockProjectWithSpecifiedDependencyVersions(
        javaVersion: JavaVersion = SUPPORTED_JAVA_VERSION,
        gradleVersion: String = SUPPORTED_GRADLE_VERSION,
        agpVersion: AndroidPluginVersion = SUPPORTED_AGP_VERSION,
        kgpVersion: String = SUPPORTED_KGP_VERSION
    ): Project {
        // Java
        mockkStatic(JavaVersion::class)
        every { JavaVersion.current() } returns javaVersion

        // Gradle
        val mockProject = mockk<Project>()
        every { mockProject.gradle.gradleVersion } returns gradleVersion

        // AGP
        val mockAndroidComponentsExtension = mockk<AndroidComponentsExtension<*, *, *>>()
        every { mockProject.extensions.findByType(AndroidComponentsExtension::class.java) } returns mockAndroidComponentsExtension
        every { mockAndroidComponentsExtension.pluginVersion } returns agpVersion

        // KGP
        every { mockProject.hasProperty(eq("kotlin_version")) } returns true
        every { mockProject.properties["kotlin_version"] } returns kgpVersion

        // Logger
        val mockLogger = mockk<Logger>()
        every { mockProject.logger } returns mockLogger

        // Extra properties extension
        val mockExtraPropertiesExtension = mockk<ExtraPropertiesExtension>()
        every { mockProject.extra } returns mockExtraPropertiesExtension

        // Project path
        every { mockProject.rootDir.path } returns FAKE_PROJECT_ROOT_DIR

        return mockProject
    }
}
