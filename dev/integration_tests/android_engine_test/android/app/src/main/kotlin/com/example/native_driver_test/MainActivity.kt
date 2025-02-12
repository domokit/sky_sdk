// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@file:Suppress("PackageName")

package com.example.android_engine_test

import android.os.Bundle
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.example.android_engine_test.extensions.NativeDriverSupportPlugin
import com.example.android_engine_test.fixtures.BlueOrangeGradientPlatformViewFactory
import com.example.android_engine_test.fixtures.BlueOrangeGradientSurfaceViewPlatformViewFactory
import com.example.android_engine_test.fixtures.BoxPlatformViewFactory
import com.example.android_engine_test.fixtures.ChangingColorButtonPlatformViewFactory
import com.example.android_engine_test.fixtures.OtherFaceTexturePlugin
import com.example.android_engine_test.fixtures.SmileyFaceTexturePlugin
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine

class MainActivity : FlutterActivity() {
    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        // Intentionally do not use GeneratedPluginRegistrant.

        flutterEngine
            .plugins
            .apply {
                add(SmileyFaceTexturePlugin())
                add(OtherFaceTexturePlugin())
                add(NativeDriverSupportPlugin())
            }

        flutterEngine
            .platformViewsController
            .registry
            .apply {
                registerViewFactory("blue_orange_gradient_platform_view", BlueOrangeGradientPlatformViewFactory())
                registerViewFactory("blue_orange_gradient_surface_view_platform_view", BlueOrangeGradientSurfaceViewPlatformViewFactory())
                registerViewFactory("changing_color_button_platform_view", ChangingColorButtonPlatformViewFactory())
                registerViewFactory("box_platform_view", BoxPlatformViewFactory())
            }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // https://developer.android.com/training/system-ui
        val windowInsetsController = WindowCompat.getInsetsController(window, window.decorView)
        windowInsetsController.systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        windowInsetsController.hide(WindowInsetsCompat.Type.systemBars())
        actionBar?.hide()
    }
}
