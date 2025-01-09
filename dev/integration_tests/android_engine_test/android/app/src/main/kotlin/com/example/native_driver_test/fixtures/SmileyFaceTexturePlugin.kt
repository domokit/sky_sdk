// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@file:Suppress("PackageName")

package com.example.android_engine_test.fixtures

import android.graphics.Color
import android.graphics.Paint
import android.os.Build
import android.view.Surface
import io.flutter.Log
import io.flutter.embedding.engine.plugins.FlutterPlugin
import io.flutter.embedding.engine.plugins.FlutterPlugin.FlutterPluginBinding
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import io.flutter.plugin.common.MethodChannel.MethodCallHandler
import io.flutter.view.TextureRegistry.SurfaceProducer

class SmileyFaceTexturePlugin :
    FlutterPlugin,
    MethodCallHandler,
    SurfaceProducer.Callback {
    private val tag = "SmileyFaceTexturePlugin"
    private lateinit var channel: MethodChannel
    private lateinit var binding: FlutterPluginBinding
    private lateinit var producer: SurfaceProducer

    private var surface: Surface? = null

    override fun onAttachedToEngine(binding: FlutterPluginBinding) {
        this.binding = binding
        channel = MethodChannel(binding.binaryMessenger, "smiley_face_texture")
        channel.setMethodCallHandler(this)
        producer = binding.textureRegistry.createSurfaceProducer()
        producer.setCallback(this)
    }

    override fun onDetachedFromEngine(binding: FlutterPluginBinding) {
        channel.setMethodCallHandler(null)
        producer.release()
    }

    override fun onMethodCall(
        call: MethodCall,
        result: MethodChannel.Result
    ) {
        if (call.method == "initTexture") {
            val height = call.argument<Int>("height") ?: 1
            val width = call.argument<Int>("width") ?: 1
            producer.setSize(width, height)
            result.success(updateTexture())
        } else {
            result.notImplemented()
        }
    }

    private fun updateTexture(): Long {
        var surface = this.surface
        if (surface == null) {
            surface = producer.surface
            this.surface = surface
        }
        drawOnSurface(producer.surface)
        return producer.id()
    }

    private fun drawOnSurface(surface: Surface) {
        val canvas =
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                surface.lockHardwareCanvas()
            } else {
                surface.lockCanvas(null)
            }

        // Yellow background
        canvas.drawRGB(255, 230, 15)

        val paint = Paint()
        paint.style = Paint.Style.FILL

        // Black eyes
        paint.color = Color.BLACK
        canvas.drawRect(200f, 200f, 250f, 250f, paint) // Left eye
        canvas.drawRect(400f, 200f, 450f, 250f, paint) // Right eye

        // Black mouth
        paint.color = Color.BLACK
        canvas.drawRect(250f, 400f, 400f, 420f, paint) // Simple mouth

        surface.unlockCanvasAndPost(canvas)
    }

    override fun onSurfaceCreated() {
        Log.i(tag, "onSurfaceCreated()")
        updateTexture()
    }

    override fun onSurfaceDestroyed() {
        Log.i(tag, "onSurfaceDestroyed()")
        surface?.release()
        surface = null
    }
}
