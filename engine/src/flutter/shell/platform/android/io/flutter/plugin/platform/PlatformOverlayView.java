// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import android.content.Context;
import android.util.AttributeSet;
import android.view.MotionEvent;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.embedding.android.FlutterImageView;

/** A host view for Flutter content displayed over a platform view. */
public class PlatformOverlayView extends FlutterImageView {
  @Nullable private AccessibilityEventsDelegate accessibilityDelegate;

  public PlatformOverlayView(
      @NonNull Context context,
      int width,
      int height,
      @NonNull AccessibilityEventsDelegate accessibilityDelegate) {
    super(context, width, height, FlutterImageView.SurfaceKind.overlay);
    this.accessibilityDelegate = accessibilityDelegate;
  }

  public PlatformOverlayView(@NonNull Context context) {
    this(context, 1, 1, null);
  }

  public PlatformOverlayView(@NonNull Context context, @NonNull AttributeSet attrs) {
    this(context, 1, 1, null);
  }

  @Override
  public boolean onHoverEvent(@NonNull MotionEvent event) {
    // This view doesn't have any accessibility information of its own, but anything drawn in
    // this view is visible above the platform view it is overlaying, so should respond to
    // accessibility exploration events. Forward those events to the accessibility delegate in
    // a special mode that will stop as soon as it reaches a platform view, so that it will not
    // find widgets that behind the platform view. If no such widget is found, treat the event
    // as unhandled so that it can fall through to the platform view.
    if (accessibilityDelegate != null
        && accessibilityDelegate.onAccessibilityHoverEvent(event, true)) {
      return true;
    }
    return super.onHoverEvent(event);
  }
}
