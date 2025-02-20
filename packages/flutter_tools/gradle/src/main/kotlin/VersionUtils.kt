// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package com.flutter.gradle

import kotlin.math.max

object VersionUtils {
    /**
     * Compares semantic versions ignoring labels.
     *
     * If the versions are equal (ignoring labels), returns one of the two strings arbitrarily. If
     * minor or patch are omitted (non-conformant to semantic versioning), they are considered zero.
     * If the provided versions in both are equal, the longest version string is returned. For
     * example, "2.8.0" vs "2.8" will always consider "2.8.0" to be the most recent version. For
     * another example, "8.7-rc-2" vs "8.7.2" will always consider "8.7.2" to be the most recent
     * version.
     */
    @JvmStatic
    fun mostRecentSemanticVersion(
        version1: String,
        version2: String
    ): String {
        val v1Parts = version1.split(".", "-")
        val v2Parts = version2.split(".", "-")
        val maxSize = max(v1Parts.size, v2Parts.size)

        for (i in 0..maxSize - 1) {
            val v1Part: String = v1Parts.getOrNull(i) ?: "0"
            val v2Part: String = v2Parts.getOrNull(i) ?: "0"

            val v1Num: Int? = v1Part.toIntOrNull()
            val v2Num: Int? = v2Part.toIntOrNull()
            when {
                v1Num != null && v2Num != null -> { // Both are numbers
                    if (v1Num != v2Num) {
                        return if (v1Num > v2Num) version1 else version2
                    }
                }
                v1Num != null && v2Num == null ->
                    return version1 // v1 is a number, v2 is not, so v1 is newer.
                v1Num == null && v2Num != null ->
                    return version2 // v1 is not a number, v2 is, so v2 is newer.
                v1Num == null && v2Num == null -> { // Both are not numbers (pre-release identifiers)
                    if (v1Part != v2Part) {
                        return if (comparePreReleaseIdentifiers(v1Part, v2Part)) version1 else version2
                    }
                }
            }
        }

        // If versions are equal, return the longest version string
        return if (version1.length >= version2.length) version1 else version2
    }

    /** Compares only non digits and returns true if v1Part is than v2Part. */
    private fun comparePreReleaseIdentifiers(
        v1Part: String,
        v2Part: String
    ): Boolean {
        val digits = Regex("\\d")
        val v1PreRelease = v1Part.replace(digits, "")
        val v2PreRelease = v2Part.replace(digits, "")
        return v1PreRelease < v2PreRelease
    }
}
