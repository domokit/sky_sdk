// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.util;

import static io.flutter.Build.API_LEVELS;

import android.content.Context;
import android.os.Build;
import androidx.annotation.NonNull;
import java.io.File;

public final class PathUtils {
  @NonNull
  public static String getFilesDir(@NonNull Context applicationContext) {
    File filesDir = applicationContext.getFilesDir();
    if (filesDir == null) {
      filesDir = new File(getDataDirPath(applicationContext), "files");
    }
    return filesDir.getPath();
  }

  @NonNull
  public static String getDataDirectory(@NonNull Context applicationContext) {
    final String name = "flutter";
    File flutterDir = applicationContext.getDir(name, Context.MODE_PRIVATE);
    if (flutterDir == null) {
      flutterDir = new File(getDataDirPath(applicationContext), "app_" + name);
    }
    return flutterDir.getPath();
  }

  @NonNull
  public static String getCacheDirectory(@NonNull Context applicationContext) {
    File cacheDir;
    cacheDir = applicationContext.getCodeCacheDir();
    if (cacheDir == null) {
      cacheDir = applicationContext.getCacheDir();
    }
    if (cacheDir == null) {
      // This can happen if the disk is full. This code path is used to set up dart:io's
      // `Directory.systemTemp`. It's unknown if the application will ever try to
      // use that or not, so do not throw here. In this case, this directory does
      // not exist because the disk is full, and the application will later get an
      // exception when it tries to actually write.
      cacheDir = new File(getDataDirPath(applicationContext), "cache");
    }
    return cacheDir.getPath();
  }

  private static String getDataDirPath(Context applicationContext) {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_24) {
      return applicationContext.getDataDir().getPath();
    } else {
      return applicationContext.getApplicationInfo().dataDir;
    }
  }
}
