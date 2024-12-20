// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/apk_utils.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

Future<void> main() async {
  await task(() async {
    try {
      await runPluginProjectTest((FlutterPluginProject pluginProject) async {
        section('check main plugin file exists');
        final File pluginMainKotlinFile = File(
          path.join(
            pluginProject.rootPath,
            'android',
            'src',
            'main',
            'kotlin',
            path.join('com', 'example', 'aaa', 'AaaPlugin.kt'),
          ),
        );

        if (!pluginMainKotlinFile.existsSync()) {
          throw TaskResult.failure(
            "Expected ${pluginMainKotlinFile.path} to exist, but it doesn't",
          );
        }

        section('add java 8 feature');
        pluginMainKotlinFile.writeAsStringSync(r'''
package com.example.aaa

import android.util.Log
import androidx.annotation.NonNull

import io.flutter.embedding.engine.plugins.FlutterPlugin
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import io.flutter.plugin.common.MethodChannel.MethodCallHandler
import io.flutter.plugin.common.MethodChannel.Result

import java.util.HashMap

/** AaaPlugin */
class AaaPlugin: FlutterPlugin, MethodCallHandler {
  init {
    val map: HashMap<String, String> = HashMap<String, String>()
    // getOrDefault is a JAVA8 feature.
    Log.d("AaaPlugin", map.getOrDefault("foo", "baz"))
  }
  /// The MethodChannel that will the communication between Flutter and native Android
  ///
  /// This local reference serves to register the plugin with the Flutter Engine and unregister it
  /// when the Flutter Engine is detached from the Activity
  private lateinit var channel : MethodChannel

  override fun onAttachedToEngine(@NonNull flutterPluginBinding: FlutterPlugin.FlutterPluginBinding) {
    channel = MethodChannel(flutterPluginBinding.binaryMessenger, "aaa")
    channel.setMethodCallHandler(this)
  }

  override fun onMethodCall(@NonNull call: MethodCall, @NonNull result: Result) {
    if (call.method == "getPlatformVersion") {
      result.success("Android ${android.os.Build.VERSION.RELEASE}")
    } else {
      result.notImplemented()
    }
  }

  override fun onDetachedFromEngine(@NonNull binding: FlutterPlugin.FlutterPluginBinding) {
    channel.setMethodCallHandler(null)
  }
}
''');

        section('Compiles');
        await inDirectory(pluginProject.exampleAndroidPath, () {
          return flutter(
            'build',
            options: <String>['apk', '--debug', '--target-platform=android-arm'],
          );
        });
      });
      return TaskResult.success(null);
    } on TaskResult catch (taskResult) {
      return taskResult;
    } catch (e) {
      return TaskResult.failure(e.toString());
    }
  });
}
