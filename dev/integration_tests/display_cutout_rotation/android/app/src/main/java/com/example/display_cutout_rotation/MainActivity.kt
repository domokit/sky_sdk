// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@file:Suppress("PackageName")

package com.example.display_cutout_rotation

import android.os.Build
import android.os.Bundle
import android.view.WindowManager
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import io.flutter.embedding.android.FlutterActivity

class MainActivity : FlutterActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // https://developer.android.com/training/system-ui
        // Set app into fullscreen mode without insets from system bars.
        // Matches api 35 default behavior and is required by test which assumes no other inset
        // except for a cutout.
        val windowInsetsController = WindowCompat.getInsetsController(window, window.decorView)
        windowInsetsController.systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        windowInsetsController.hide(WindowInsetsCompat.Type.systemBars())

        // The default behavior on SDK level 34 and below is for display cutouts to be consumed
        // before the insets would reach the engine. In order to receive the display cutouts in the
        // engine, the test app must request that it be allowed to draw its content behind cutouts.
        // See
        // https://developer.android.com/reference/android/view/WindowManager.LayoutParams#layoutInDisplayCutoutMode
        // LAYOUT_IN_DISPLAY_CUTOUT_MODE_ALWAYS was added in api 30 so we need to check api level
        // before setting the value. Not setting this value will prevent flutter from drawing in
        // cutout areas which our test is explicitly requires.
        if (Build.VERSION.SDK_INT >= 30) {
            window.attributes.layoutInDisplayCutoutMode =
                WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_ALWAYS
        }
    }
}
