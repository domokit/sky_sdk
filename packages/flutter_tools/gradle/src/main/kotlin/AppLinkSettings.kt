// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package com.flutter.gradle

import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put
import kotlinx.serialization.json.putJsonArray

data class AppLinkSettings(
    val applicationId: String
) {
    var deeplinkingFlagEnabled = false
    val deeplinks = mutableSetOf<Deeplink>()

    fun toJson(): JsonObject {
        return buildJsonObject {
            put("applicationId", applicationId)
            put("deeplinkingFlagEnabled", deeplinkingFlagEnabled)
            putJsonArray("deeplinks") {
                for (deeplink:Deeplink in deeplinks) {
                    add(deeplink.toJson())
                }
            }

        }
    }
}