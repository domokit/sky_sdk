// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@file:Suppress("PackageName")

package com.example.android_engine_test.extensions

import android.view.View
import android.view.ViewGroup
import androidx.core.view.children

// / Finds a native view using serialized data provided by a driver script.
sealed class NativeSelector {
    // / Finds a native view by a {@code contentDescription} attribute.
    data class ByContentDescription(
        val contentDescription: String
    ) : NativeSelector() {
        override fun find(root: View): View? {
            val found = ArrayList<View>()
            findRecursive(root, found)
            if (found.isEmpty()) {
                return null
            }
            if (found.size > 1) {
                throw IllegalStateException("Expected a single matching view, got ${found.size}")
            }
            return found[0]
        }

        private fun findRecursive(
            parent: View,
            found: ArrayList<View>
        ) {
            if (contentDescription == parent.contentDescription) {
                found.add(parent)
            }
            if (parent is ViewGroup) {
                for (child in parent.children) {
                    findRecursive(child, found)
                }
            }
        }
    }

    // / Finds a native view by a {@code id} attribute.
    data class ByViewId(
        val id: Int
    ) : NativeSelector() {
        override fun find(root: View): View? = root.findViewById(id)
    }

    // / Given a root view, returns the only view that matches this selector.
    // /
    // / If no view is found, returns null.
    abstract fun find(root: View): View?
}
