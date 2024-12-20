// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import android.view.MotionEvent;
import android.view.View;
import android.view.accessibility.AccessibilityEvent;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.view.AccessibilityBridge;

/**
 * Delegates accessibility events to the currently attached accessibility bridge if one is attached.
 */
class AccessibilityEventsDelegate {
  private AccessibilityBridge accessibilityBridge;

  /**
   * Delegates handling of {@link android.view.ViewParent#requestSendAccessibilityEvent} to the
   * accessibility bridge.
   *
   * <p>This is a no-op if there is no accessibility delegate set.
   *
   * <p>This is used by embedded platform views to propagate accessibility events from their view
   * hierarchy to the accessibility bridge.
   *
   * <p>As the embedded view doesn't have to be the only View in the embedded hierarchy (it can have
   * child views) and the event might have been originated from any view in this hierarchy, this
   * method gets both a reference to the embedded platform view, and a reference to the view from
   * its hierarchy that sent the event.
   *
   * @param embeddedView the embedded platform view for which the event is delegated
   * @param eventOrigin the view in the embedded view's hierarchy that sent the event.
   * @return True if the event was sent.
   */
  public boolean requestSendAccessibilityEvent(
      @NonNull View embeddedView, @NonNull View eventOrigin, @NonNull AccessibilityEvent event) {
    if (accessibilityBridge == null) {
      return false;
    }
    return accessibilityBridge.externalViewRequestSendAccessibilityEvent(
        embeddedView, eventOrigin, event);
  }

  public boolean onAccessibilityHoverEvent(MotionEvent event, boolean ignorePlatformViews) {
    if (accessibilityBridge == null) {
      return false;
    }
    return accessibilityBridge.onAccessibilityHoverEvent(event, ignorePlatformViews);
  }

  /*
   * This setter should only be used directly in PlatformViewsController when attached/detached to an accessibility
   * bridge.
   */
  void setAccessibilityBridge(@Nullable AccessibilityBridge accessibilityBridge) {
    this.accessibilityBridge = accessibilityBridge;
  }
}
