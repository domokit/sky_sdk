// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package com.yourcompany.platforminteraction;

import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.lang.StringBuilder;

import android.graphics.Rect;
import android.os.Build;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.view.WindowManager;
import android.content.ClipboardManager;
import android.content.ClipData;
import android.content.Context;
import androidx.annotation.NonNull;

import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.android.FlutterView;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.MethodChannel.MethodCallHandler;
import io.flutter.plugins.GeneratedPluginRegistrant;

import android.view.accessibility.AccessibilityManager;
import android.view.accessibility.AccessibilityNodeProvider;
import android.view.accessibility.AccessibilityNodeInfo;

public class MainActivity extends FlutterActivity {
  @Override
  public void configureFlutterEngine(@NonNull FlutterEngine flutterEngine) {
      GeneratedPluginRegistrant.registerWith(flutterEngine);
      new MethodChannel(flutterEngine.getDartExecutor().getBinaryMessenger(), "semantics")
              .setMethodCallHandler(new SemanticsTesterMethodHandler());
  }

  class SemanticsTesterMethodHandler implements MethodCallHandler {
    Float mScreenDensity = 1.0f;

    @Override
    public void onMethodCall(MethodCall methodCall, MethodChannel.Result result) {
        FlutterView flutterView = findViewById(MainActivity.this.flutterViewId);
        AccessibilityNodeProvider provider = flutterView.getAccessibilityNodeProvider();
        DisplayMetrics displayMetrics = new DisplayMetrics();
        WindowManager wm = (WindowManager) getApplicationContext().getSystemService(Context.WINDOW_SERVICE);
        wm.getDefaultDisplay().getMetrics(displayMetrics);
        mScreenDensity = displayMetrics.density;
        if (methodCall.method.equals("getSemanticsNode")) {
            Map<String, Object> data = methodCall.arguments();
            @SuppressWarnings("unchecked")
            Integer id = (Integer) data.get("id");
            if (id == null) {
                result.error("No ID provided", "", null);
                return;
            }
            if (provider == null) {
                result.error("Semantics not enabled", "", null);
                return;
            }
            AccessibilityNodeInfo node = provider.createAccessibilityNodeInfo(id);
            result.success(convertSemantics(node, id));
            return;
        }
        if (methodCall.method.equals("setClipboard")) {
            Map<String, Object> data = methodCall.arguments();
            @SuppressWarnings("unchecked")
            String message = (String) data.get("message");
            ClipboardManager clipboard = (ClipboardManager) getSystemService(Context.CLIPBOARD_SERVICE);
            ClipData clip = ClipData.newPlainText("message", message);
            clipboard.setPrimaryClip(clip);
            result.success(null);
            return;
        }
        result.notImplemented();
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> convertSemantics(AccessibilityNodeInfo node, int id) {
        if (node == null)
            return null;
        Map<String, Object> result = new HashMap<>();
        Map<String, Object> flags = new HashMap<>();
        Map<String, Object> rect = new HashMap<>();
        result.put("id", id);
        result.put("text", node.getText());
        result.put("contentDescription", node.getContentDescription());
        flags.put("isChecked", node.isChecked());
        flags.put("isCheckable", node.isCheckable());
        // This is not a typo.
        // See: https://developer.android.com/reference/android/view/accessibility/AccessibilityNodeInfo#isDismissable()
        flags.put("isDismissible", node.isDismissable());
        flags.put("isEditable", node.isEditable());
        flags.put("isEnabled", node.isEnabled());
        flags.put("isFocusable", node.isFocusable());
        flags.put("isFocused", node.isFocused());
        // heading flag is only available on Android Pie or newer
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
            flags.put("isHeading", node.isHeading());
        }
        flags.put("isPassword", node.isPassword());
        flags.put("isLongClickable", node.isLongClickable());
        result.put("flags", flags);
        Rect nodeRect = new Rect();
        node.getBoundsInScreen(nodeRect);
        rect.put("left", nodeRect.left / mScreenDensity);
        rect.put("top", nodeRect.top/ mScreenDensity);
        rect.put("right", nodeRect.right / mScreenDensity);
        rect.put("bottom", nodeRect.bottom/ mScreenDensity);
        rect.put("width", nodeRect.width());
        rect.put("height", nodeRect.height());
        result.put("rect", rect);
        result.put("className", node.getClassName());
        result.put("contentDescription", node.getContentDescription());
        result.put("liveRegion", node.getLiveRegion());
        List<AccessibilityNodeInfo.AccessibilityAction> actionList = node.getActionList();
        if (actionList.size() > 0) {
            ArrayList<Integer> actions = new ArrayList<>();
            for (AccessibilityNodeInfo.AccessibilityAction action : actionList) {
                if (kIdByAction.containsKey(action)) {
                    actions.add(kIdByAction.get(action).intValue());
                }
            }
            result.put("actions", actions);
        }
        return result;
    }
  }

  // These indices need to be in sync with android_semantics_testing/lib/src/constants.dart
  static final int kFocusIndex = 1 << 0;
  static final int kClearFocusIndex = 1 << 1;
  static final int kSelectIndex = 1 << 2;
  static final int kClearSelectionIndex = 1 << 3;
  static final int kClickIndex = 1 << 4;
  static final int kLongClickIndex = 1 << 5;
  static final int kAccessibilityFocusIndex = 1 << 6;
  static final int kClearAccessibilityFocusIndex = 1 << 7;
  static final int kNextAtMovementGranularityIndex = 1 << 8;
  static final int kPreviousAtMovementGranularityIndex = 1 << 9;
  static final int kNextHtmlElementIndex = 1 << 10;
  static final int kPreviousHtmlElementIndex = 1 << 11;
  static final int kScrollForwardIndex = 1 << 12;
  static final int kScrollBackwardIndex = 1 << 13;
  static final int kCutIndex = 1 << 14;
  static final int kCopyIndex = 1 << 15;
  static final int kPasteIndex = 1 << 16;
  static final int kSetSelectionIndex = 1 << 17;
  static final int kExpandIndex = 1 << 18;
  static final int kCollapseIndex = 1 << 19;
  static final int kSetText = 1 << 21;

  static final Map<AccessibilityNodeInfo.AccessibilityAction, Integer> kIdByAction = new HashMap<AccessibilityNodeInfo.AccessibilityAction, Integer>() {{
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_FOCUS, kFocusIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_CLEAR_FOCUS, kClearFocusIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_SELECT, kSelectIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_CLEAR_SELECTION, kClearSelectionIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_CLICK, kClickIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_LONG_CLICK, kLongClickIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_ACCESSIBILITY_FOCUS, kAccessibilityFocusIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_CLEAR_ACCESSIBILITY_FOCUS, kClearAccessibilityFocusIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_NEXT_AT_MOVEMENT_GRANULARITY, kNextAtMovementGranularityIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_PREVIOUS_AT_MOVEMENT_GRANULARITY, kPreviousAtMovementGranularityIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_NEXT_HTML_ELEMENT, kNextHtmlElementIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_PREVIOUS_HTML_ELEMENT, kPreviousHtmlElementIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_SCROLL_FORWARD, kScrollForwardIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_SCROLL_BACKWARD, kScrollBackwardIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_CUT, kCutIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_COPY, kCopyIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_PASTE, kPasteIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_SET_SELECTION, kSetSelectionIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_EXPAND, kExpandIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_COLLAPSE, kCollapseIndex);
    put(AccessibilityNodeInfo.AccessibilityAction.ACTION_SET_TEXT, kSetText);
  }};
}
