// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@file:Suppress("PackageName")

package com.example.android_engine_test.fixtures

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.view.View
import android.view.ViewGroup
import io.flutter.plugin.platform.PlatformView
import io.flutter.plugin.platform.PlatformViewFactory

class BoxPlatformViewFactory : PlatformViewFactory(null) {
    override fun create(
        context: Context,
        viewId: Int,
        args: Any?
    ): PlatformView = BoxPlatformView(context)
}

private class BoxPlatformView(
    context: Context
) : View(context),
    PlatformView {
    val paint = Paint()

    init {
        layoutParams =
            ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )
    }

    override fun getView(): View = this

    override fun dispose() {}

    override fun onDraw(canvas: Canvas) {
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), paint)
        super.onDraw(canvas)
    }

    override fun onSizeChanged(
        w: Int,
        h: Int,
        oldw: Int,
        oldh: Int
    ) {
        paint.color = Color.rgb(0x41, 0x69, 0xE1)
        super.onSizeChanged(w, h, oldw, oldh)
    }
}
