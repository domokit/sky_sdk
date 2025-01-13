// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@file:Suppress("PackageName")

package com.example.android_engine_test.fixtures

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LinearGradient
import android.graphics.Paint
import android.graphics.Shader
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.view.ViewGroup
import io.flutter.plugin.platform.PlatformView
import io.flutter.plugin.platform.PlatformViewFactory

class BlueOrangeGradientSurfaceViewPlatformViewFactory : PlatformViewFactory(null) {
    override fun create(
        context: Context,
        viewId: Int,
        args: Any?
    ): PlatformView = GradientSurfaceViewPlatformView(context)
}

private class GradientSurfaceViewPlatformView(
    context: Context
) : SurfaceView(context),
    PlatformView,
    SurfaceHolder.Callback {
    val paint = Paint()

    init {
        layoutParams =
            ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )
        holder.addCallback(this)
    }

    override fun getView(): View = this

    override fun dispose() {}

    override fun surfaceCreated(holder: SurfaceHolder) {
        val canvas = holder.lockCanvas()
        if (canvas != null) {
            drawGradient(canvas)
            holder.unlockCanvasAndPost(canvas)
        }
    }

    override fun surfaceChanged(
        holder: SurfaceHolder,
        format: Int,
        width: Int,
        height: Int
    ) {
        val canvas = holder.lockCanvas()
        if (canvas != null) {
            drawGradient(canvas)
            holder.unlockCanvasAndPost(canvas)
        }
    }

    override fun surfaceDestroyed(holder: SurfaceHolder) {}

    private fun drawGradient(canvas: Canvas) {
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), paint)
    }

    override fun onSizeChanged(
        w: Int,
        h: Int,
        oldw: Int,
        oldh: Int
    ) {
        paint.shader =
            LinearGradient(
                0f,
                0f,
                w.toFloat(),
                h.toFloat(),
                intArrayOf(
                    Color.rgb(0x41, 0x69, 0xE1),
                    Color.rgb(0xFF, 0xA5, 0x00)
                ),
                null,
                Shader.TileMode.CLAMP
            )
        super.onSizeChanged(w, h, oldw, oldh)
    }
}
