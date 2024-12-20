// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import static io.flutter.Build.API_LEVELS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import android.annotation.TargetApi;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.BasicMessageChannel;
import java.util.HashMap;
import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

@Config(
    manifest = Config.NONE,
    shadows = {})
@RunWith(RobolectricTestRunner.class)
@TargetApi(API_LEVELS.API_24)
public class AccessibilityChannelTest {
  @Test
  public void repliesWhenNoAccessibilityHandler() throws JSONException {
    AccessibilityChannel accessibilityChannel =
        new AccessibilityChannel(mock(DartExecutor.class), mock(FlutterJNI.class));
    JSONObject arguments = new JSONObject();
    arguments.put("type", "announce");
    BasicMessageChannel.Reply reply = mock(BasicMessageChannel.Reply.class);
    accessibilityChannel.parsingMessageHandler.onMessage(arguments, reply);
    verify(reply).reply(null);
  }

  @Test
  public void handleFocus() throws JSONException {
    AccessibilityChannel accessibilityChannel =
        new AccessibilityChannel(mock(DartExecutor.class), mock(FlutterJNI.class));
    HashMap<String, Object> arguments = new HashMap<>();
    arguments.put("type", "focus");
    arguments.put("nodeId", 123);
    AccessibilityChannel.AccessibilityMessageHandler handler =
        mock(AccessibilityChannel.AccessibilityMessageHandler.class);
    accessibilityChannel.setAccessibilityMessageHandler(handler);
    BasicMessageChannel.Reply reply = mock(BasicMessageChannel.Reply.class);
    accessibilityChannel.parsingMessageHandler.onMessage(arguments, reply);
    verify(handler).onFocus(123);
  }
}
