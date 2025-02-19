package com.flutter.gradle

import kotlin.test.Test
import kotlin.test.assertEquals

class VersionUtilsTest {
    @Test
    fun handles_documenation_examples() {
        versionComparison("2.8.0", "2.8", expected = "2.8.0")
        versionComparison("8.7-rc-2", "8.7.2", expected = "8.7.2")
    }

    @Test
    fun expanded_examples() {
        versionComparison("1.2", "1.2.0", expected = "1.2.0")
        versionComparison("1.0", "1", expected = "1.0")
        versionComparison("1.2.0-alpha", "1.2", expected = "1.2")
        versionComparison("1.2.3", "1.2.3", expected = "1.2.3")
        versionComparison("1.2.3-beta", "1.2.3", expected = "1.2.3")
        versionComparison("1.2.3", "1.2.3.4", expected = "1.2.3.4")
        versionComparison("rc-2", "rc-1", expected = "rc-2")
        versionComparison("8.7-rc-1", "8.7", expected = "8.7")
        versionComparison("8.7-rc-1", "8.7.2", expected = "8.7.2")
        versionComparison("8.7.2", "8.7.1", expected = "8.7.2")
        versionComparison("7.0.2", "8.7.1", expected = "8.7.1")
        versionComparison("8.1", "7.5", expected = "8.1")
    }

    fun versionComparison(
        version1: String,
        version2: String,
        expected: String
    ) {
        assertEquals(expected, VersionUtils.mostRecentSemanticVersion(version1, version2))
    }
}
