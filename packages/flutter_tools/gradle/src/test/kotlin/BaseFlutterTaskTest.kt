package com.flutter.gradle
import FlutterTask
import com.android.build.api.dsl.ApplicationDefaultConfig
import com.android.build.api.dsl.ApplicationExtension
import com.flutter.gradle.BaseApplicationNameHandler.GRADLE_BASE_APPLICATION_NAME_PROPERTY
import org.gradle.api.GradleException
import org.gradle.api.Project
import org.gradle.api.plugins.ExtensionContainer
import org.gradle.internal.impldep.org.jsoup.Connection.Base
import org.junit.jupiter.api.Assertions.assertEquals
import org.mockito.Mockito
import kotlin.test.Test
import kotlin.test.assertFailsWith

class BaseFlutterTaskTest {
    class BaseFlutterTaskForTest : BaseFlutterTask(){
    }
    @Test
    fun `buildBundle throws a GradleException when sourceDir is null`() {
        // Using BaseFlutterTask to call buildBundle
        // Set up mocks.
        val baseFlutterTask: BaseFlutterTaskForTest = BaseFlutterTaskForTest()


//        Mockito.`when`(baseFlutterTask.sourceDir).thenReturn(null)
//        Mockito.`when`(baseFlutterTask.buildBundle()).thenReturn(GradleException())

        // Make sure the exception was thrown.
        assertFailsWith<GradleException> {
            baseFlutterTask.buildBundle()
        }

        // Using FlutterTask to call buildBundle
//        val flutterTask: FlutterTask = FlutterTask()
//        // Set up mocks.
//        Mockito.`when`(flutterTask.sourceDir).thenReturn(null)
//
//        // Make sure the exception was thrown.
//        assertFailsWith<GradleException> {
//            baseFlutterTask.buildBundle()
//        }
    }
}