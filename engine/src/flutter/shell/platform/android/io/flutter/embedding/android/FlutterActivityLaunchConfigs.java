// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import android.os.Bundle;

/** Collection of Flutter launch configuration options. */
// This class is public so that Flutter app developers can reference
// BackgroundMode
@SuppressWarnings("WeakerAccess")
public class FlutterActivityLaunchConfigs {
  // Meta-data arguments, processed from manifest XML.
  /* package */ static final String DART_ENTRYPOINT_META_DATA_KEY = "io.flutter.Entrypoint";
  /* package */ static final String DART_ENTRYPOINT_URI_META_DATA_KEY = "io.flutter.EntrypointUri";
  /* package */ static final String INITIAL_ROUTE_META_DATA_KEY = "io.flutter.InitialRoute";
  /* package */ static final String NORMAL_THEME_META_DATA_KEY =
      "io.flutter.embedding.android.NormalTheme";
  /* package */ static final String HANDLE_DEEPLINKING_META_DATA_KEY =
      "flutter_deeplinking_enabled";
  // Intent extra arguments.
  /* package */ static final String EXTRA_DART_ENTRYPOINT = "dart_entrypoint";
  /* package */ static final String EXTRA_INITIAL_ROUTE = "route";
  /* package */ static final String EXTRA_BACKGROUND_MODE = "background_mode";
  /* package */ static final String EXTRA_CACHED_ENGINE_ID = "cached_engine_id";
  /* package */ static final String EXTRA_DART_ENTRYPOINT_ARGS = "dart_entrypoint_args";
  /* package */ static final String EXTRA_CACHED_ENGINE_GROUP_ID = "cached_engine_group_id";
  /* package */ static final String EXTRA_DESTROY_ENGINE_WITH_ACTIVITY =
      "destroy_engine_with_activity";
  /* package */ static final String EXTRA_ENABLE_STATE_RESTORATION = "enable_state_restoration";

  // Default configuration.
  /* package */ static final String DEFAULT_DART_ENTRYPOINT = "main";
  /* package */ static final String DEFAULT_INITIAL_ROUTE = "/";
  /* package */ static final String DEFAULT_BACKGROUND_MODE = BackgroundMode.opaque.name();

  /** The mode of the background of a Flutter {@code Activity}, either opaque or transparent. */
  public enum BackgroundMode {
    /** Indicates a FlutterActivity with an opaque background. This is the default. */
    opaque,
    /** Indicates a FlutterActivity with a transparent background. */
    transparent
  }

  /**
   * Whether to handle the deeplinking.
   *
   * <p>The default implementation looks {@code <meta-data>} called {@link
   * FlutterActivityLaunchConfigs#HANDLE_DEEPLINKING_META_DATA_KEY} within the Android manifest
   * definition for this {@code FlutterActivity}.
   *
   * <p>Defaults to {@code true}.
   */
  public static boolean deepLinkEnabled(Bundle metaData) {
    // Check if metadata is not null and contains the HANDLE_DEEPLINKING_META_DATA_KEY.
    if (metaData != null && metaData.containsKey(HANDLE_DEEPLINKING_META_DATA_KEY)) {
      return metaData.getBoolean(HANDLE_DEEPLINKING_META_DATA_KEY);
    } else {
      // Return true if the deep linking flag is not found in metadata.
      return true;
    }
  }

  private FlutterActivityLaunchConfigs() {}
}
