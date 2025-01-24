package com.flutter.gradle
import com.android.build.api.dsl.ApplicationDefaultConfig
import com.android.build.api.dsl.ApplicationExtension
import com.flutter.gradle.BaseApplicationNameHandler.GRADLE_BASE_APPLICATION_NAME_PROPERTY
import org.gradle.api.Project
import org.gradle.api.plugins.ExtensionContainer
import org.junit.jupiter.api.Assertions.assertEquals
import org.mockito.Mockito
import kotlin.test.Test

class BaseApplicationNameHandlerTest {
    @Test
    fun `setBaseName respects Flutter tool property`() {
        val baseApplicationNamePassedByFlutterTool = "toolSetBaseApplicationName"

        // Set up mocks.
        val mockProject: Project = Mockito.mock(Project::class.java)
        val mockAndroidComponentsExtension: ApplicationExtension = Mockito.mock(ApplicationExtension::class.java)
        val mockExtensionContainer: ExtensionContainer = Mockito.mock(ExtensionContainer::class.java)
        val mockDefaultConfig = Mockito.mock(ApplicationDefaultConfig::class.java)
        val mockManifestPlaceholders = HashMap<String, Any>()

        Mockito.`when`(mockProject.hasProperty(GRADLE_BASE_APPLICATION_NAME_PROPERTY)).thenReturn(true)
        Mockito.`when`(mockProject.property(GRADLE_BASE_APPLICATION_NAME_PROPERTY)).thenReturn(baseApplicationNamePassedByFlutterTool)

        Mockito.`when`(mockProject.extensions).thenReturn(mockExtensionContainer)
        Mockito.`when`(mockExtensionContainer.findByType(ApplicationExtension::class.java)).thenReturn(mockAndroidComponentsExtension)
        Mockito.`when`(mockAndroidComponentsExtension.defaultConfig).thenReturn(mockDefaultConfig)
        Mockito.`when`(mockDefaultConfig.manifestPlaceholders).thenReturn(mockManifestPlaceholders)

        // Call the base name handler.
        BaseApplicationNameHandler.setBaseName(mockProject)

        // Make sure we set the value passed by the tool.
        assertEquals(mockManifestPlaceholders["applicationName"], baseApplicationNamePassedByFlutterTool)
    }

    @Test
    fun `setBaseName defaults to correct value`() {
        // Set up mocks.
        val mockProject: Project = Mockito.mock(Project::class.java)
        val mockAndroidComponentsExtension: ApplicationExtension = Mockito.mock(ApplicationExtension::class.java)
        val mockExtensionContainer: ExtensionContainer = Mockito.mock(ExtensionContainer::class.java)
        val mockDefaultConfig = Mockito.mock(ApplicationDefaultConfig::class.java)
        val mockManifestPlaceholders = HashMap<String, Any>()

        Mockito.`when`(mockProject.hasProperty(GRADLE_BASE_APPLICATION_NAME_PROPERTY)).thenReturn(false)

        Mockito.`when`(mockProject.extensions).thenReturn(mockExtensionContainer)
        Mockito.`when`(mockExtensionContainer.findByType(ApplicationExtension::class.java)).thenReturn(mockAndroidComponentsExtension)
        Mockito.`when`(mockAndroidComponentsExtension.defaultConfig).thenReturn(mockDefaultConfig)
        Mockito.`when`(mockDefaultConfig.manifestPlaceholders).thenReturn(mockManifestPlaceholders)

        // Call the base name handler.
        BaseApplicationNameHandler.setBaseName(mockProject)

        // Make sure we default to the correct value.
        assertEquals(mockManifestPlaceholders["applicationName"], BaseApplicationNameHandler.DEFAULT_BASE_APPLICATION_NAME)
    }
}
