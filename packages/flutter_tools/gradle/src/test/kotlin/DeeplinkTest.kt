package com.flutter.gradle

import org.gradle.internal.impldep.org.junit.Assert.assertThrows
import kotlin.test.Test
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class DeeplinkTest {
    @Test
    fun `equals should return true for equal objects`() {
        val deeplink1 = Deeplink("scheme1", "host1", "path1", null)
        val deeplink2 = Deeplink("scheme1", "host1", "path1", null)

        assertTrue { deeplink1 == deeplink2 }
    }

    @Test
    fun `equals should return false for unequal objects`() {
        val deeplink1 = Deeplink("scheme1", "host1", "path1", null)
        val deeplink2 = Deeplink("scheme2", "host2", "path2", null)

        assertFalse { deeplink1 == deeplink2 }
    }

    @Test
    fun `equals should return false for other of different type`() {
        val deeplink1 = Deeplink("scheme1", "host1", "path1", null)
        val notADeeplink = 5

        assertFalse { deeplink1.equals(notADeeplink) }
    }

    @Suppress("UnusedEquals")
    @Test
    fun `equals should throw NullPointerException for null other`() {
        val deeplink1 = Deeplink("scheme1", "host1", "path1", null)
        val deeplink2 = null

        assertThrows(NullPointerException::class.java, { deeplink1.equals(deeplink2) })
    }
}
