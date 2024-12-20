// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.text;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.activity.ActivityAware;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.embedding.engine.systemchannels.ProcessTextChannel;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.PluginRegistry.ActivityResultListener;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ProcessTextPlugin
    implements FlutterPlugin,
        ActivityAware,
        ActivityResultListener,
        ProcessTextChannel.ProcessTextMethodHandler {
  private static final String TAG = "ProcessTextPlugin";

  @NonNull private final ProcessTextChannel processTextChannel;
  @NonNull private final PackageManager packageManager;
  @Nullable private ActivityPluginBinding activityBinding;
  private Map<String, ResolveInfo> resolveInfosById;

  @NonNull
  private Map<Integer, MethodChannel.Result> requestsByCode =
      new HashMap<Integer, MethodChannel.Result>();

  public ProcessTextPlugin(@NonNull ProcessTextChannel processTextChannel) {
    this.processTextChannel = processTextChannel;
    this.packageManager = processTextChannel.packageManager;

    processTextChannel.setMethodHandler(this);
  }

  @Override
  public Map<String, String> queryTextActions() {
    if (resolveInfosById == null) {
      cacheResolveInfos();
    }
    Map<String, String> result = new HashMap<String, String>();
    for (String id : resolveInfosById.keySet()) {
      final ResolveInfo info = resolveInfosById.get(id);
      result.put(id, info.loadLabel(packageManager).toString());
    }
    return result;
  }

  @Override
  public void processTextAction(
      @NonNull String id,
      @NonNull String text,
      @NonNull boolean readOnly,
      @NonNull MethodChannel.Result result) {
    if (activityBinding == null) {
      result.error("error", "Plugin not bound to an Activity", null);
      return;
    }

    if (Build.VERSION.SDK_INT < API_LEVELS.API_23) {
      result.error("error", "Android version not supported", null);
      return;
    }

    if (resolveInfosById == null) {
      result.error("error", "Can not process text actions before calling queryTextActions", null);
      return;
    }

    final ResolveInfo info = resolveInfosById.get(id);
    if (info == null) {
      result.error("error", "Text processing activity not found", null);
      return;
    }

    Integer requestCode = result.hashCode();
    requestsByCode.put(requestCode, result);

    Intent intent = new Intent();
    intent.setClassName(info.activityInfo.packageName, info.activityInfo.name);
    intent.setAction(Intent.ACTION_PROCESS_TEXT);
    intent.setType("text/plain");
    intent.putExtra(Intent.EXTRA_PROCESS_TEXT, text);
    intent.putExtra(Intent.EXTRA_PROCESS_TEXT_READONLY, readOnly);

    // Start the text processing activity. When the activity completes, the onActivityResult
    // callback
    // is called.
    activityBinding.getActivity().startActivityForResult(intent, requestCode);
  }

  private void cacheResolveInfos() {
    resolveInfosById = new HashMap<String, ResolveInfo>();

    if (Build.VERSION.SDK_INT < API_LEVELS.API_23) {
      return;
    }

    Intent intent = new Intent().setAction(Intent.ACTION_PROCESS_TEXT).setType("text/plain");

    List<ResolveInfo> infos;
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
      infos = packageManager.queryIntentActivities(intent, PackageManager.ResolveInfoFlags.of(0));
    } else {
      infos = packageManager.queryIntentActivities(intent, 0);
    }

    for (ResolveInfo info : infos) {
      final String id = info.activityInfo.name;
      final String label = info.loadLabel(packageManager).toString();
      resolveInfosById.put(id, info);
    }
  }

  /**
   * Executed when a text processing activity terminates.
   *
   * <p>When an activity returns a value, the request is completed successfully and returns the
   * processed text.
   *
   * <p>When an activity does not return a value. the request is completed successfully and returns
   * null.
   */
  @TargetApi(API_LEVELS.API_23)
  @RequiresApi(API_LEVELS.API_23)
  public boolean onActivityResult(int requestCode, int resultCode, @Nullable Intent intent) {
    // Return early if the result is not related to a request sent by this plugin.
    if (!requestsByCode.containsKey(requestCode)) {
      return false;
    }

    String result = null;
    if (resultCode == Activity.RESULT_OK) {
      result = intent.getStringExtra(Intent.EXTRA_PROCESS_TEXT);
    }
    requestsByCode.remove(requestCode).success(result);
    return true;
  }

  /**
   * Unregisters this {@code ProcessTextPlugin} as the {@code
   * ProcessTextChannel.ProcessTextMethodHandler}, for the {@link
   * io.flutter.embedding.engine.systemchannels.ProcessTextChannel}.
   *
   * <p>Do not invoke any methods on a {@code ProcessTextPlugin} after invoking this method.
   */
  public void destroy() {
    processTextChannel.setMethodHandler(null);
  }

  // FlutterPlugin interface implementation.

  public void onAttachedToEngine(@NonNull FlutterPluginBinding binding) {
    // Nothing to do because this plugin is instantiated by the engine.
  }

  public void onDetachedFromEngine(@NonNull FlutterPluginBinding binding) {
    // Nothing to do because this plugin is instantiated by the engine.
  }

  // ActivityAware interface implementation.
  //
  // Store the binding and manage the activity result listener.

  public void onAttachedToActivity(@NonNull ActivityPluginBinding binding) {
    this.activityBinding = binding;
    this.activityBinding.addActivityResultListener(this);
  };

  public void onDetachedFromActivityForConfigChanges() {
    this.activityBinding.removeActivityResultListener(this);
    this.activityBinding = null;
  }

  public void onReattachedToActivityForConfigChanges(@NonNull ActivityPluginBinding binding) {
    this.activityBinding = binding;
    this.activityBinding.addActivityResultListener(this);
  }

  public void onDetachedFromActivity() {
    this.activityBinding.removeActivityResultListener(this);
    this.activityBinding = null;
  }
}
