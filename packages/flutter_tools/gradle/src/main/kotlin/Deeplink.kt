// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package com.flutter.gradle

// TODO(gmackall): Identify which of these can be val instead of var.
class Deeplink(var scheme: String?, var host: String?, var path: String?, var intentFilterCheck: IntentFilterCheck?) {
    // TODO(gmackall): This behavior was kept identical to the original Groovy behavior as part of
    // the Groovy->Kotlin conversion, but should be changed once the conversion is complete.
    override fun equals(other: Any?): Boolean {
        if (other == null) {
            throw NullPointerException()
        }
        if (other.javaClass != javaClass) {
            return false
        }
        val otherAsDeeplink = other as Deeplink
        return scheme == otherAsDeeplink.scheme &&
            host == otherAsDeeplink.host &&
            path == otherAsDeeplink.path
    }

    override fun hashCode(): Int {
        return scheme.hashCode() + host.hashCode() + path.hashCode()
    }
}
