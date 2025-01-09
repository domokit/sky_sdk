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

class ChangingColorButtonPlatformViewFactory : PlatformViewFactory(null) {
    override fun create(
        context: Context,
        viewId: Int,
        args: Any?
    ): PlatformView = ChangingColorButtonPlatformView(context)
}

private class ChangingColorButtonPlatformView(
    context: Context
) : View(context),
    PlatformView {
    private val paintRed =
        Paint().apply {
            color = Color.RED
            style = Paint.Style.FILL
        }
    private val paintBlue =
        Paint().apply {
            color = Color.BLUE
            style = Paint.Style.FILL
        }

    private var isBlue = false

    init {
        layoutParams =
            ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )

        contentDescription = "Change color"

        setOnClickListener {
            run {
                isBlue = !isBlue
                invalidate() // Force a redraw
            }
        }
    }

    override fun getView(): View = this

    override fun dispose() {}

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), if (isBlue) paintBlue else paintRed)
    }
}
