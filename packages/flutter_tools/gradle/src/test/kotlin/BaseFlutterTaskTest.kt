package com.flutter.gradle

import io.mockk.every
import io.mockk.mockk
import org.gradle.api.GradleException
import java.io.File
import kotlin.test.Test
import kotlin.test.assertFailsWith

class BaseFlutterTaskTest {
    @Test
    fun `buildBundle throws a GradleException when sourceDir is null`() {
        val baseFlutterTask = mockk<BaseFlutterTask>()
        every { baseFlutterTask.sourceDir } returns null

        val helper = BaseFlutterTaskHelper(baseFlutterTask)

        val gradleException =
            assertFailsWith<GradleException> { helper.checkPreConditions() }
        assert(
            gradleException.message ==
                    helper.gradleErrorMessage
        )
    }

    @Test
    fun `buildBundle throws a GradleException when sourceDir is not a directory`() {
        val baseFlutterTask = mockk<BaseFlutterTask>()
        val file = mockk<File>()
        every { baseFlutterTask.sourceDir } returns file
        every { baseFlutterTask.sourceDir!!.isDirectory } returns false

        val helper = BaseFlutterTaskHelper(baseFlutterTask)

        val gradleException =
            assertFailsWith<GradleException> { helper.checkPreConditions() }
        assert(
            gradleException.message ==
                    helper.gradleErrorMessage
        )
    }
}
