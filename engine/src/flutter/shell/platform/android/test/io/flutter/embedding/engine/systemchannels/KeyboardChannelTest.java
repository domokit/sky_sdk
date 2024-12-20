// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static org.mockito.Mockito.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.systemchannels.KeyboardChannel;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.StandardMethodCodec;
import java.nio.ByteBuffer;
import java.util.HashMap;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class KeyboardChannelTest {

  private static BinaryMessenger.BinaryReply sendToBinaryMessageHandler(
      BinaryMessenger.BinaryMessageHandler binaryMessageHandler, String method, Object args) {
    MethodCall methodCall = new MethodCall(method, args);
    ByteBuffer encodedMethodCall = StandardMethodCodec.INSTANCE.encodeMethodCall(methodCall);
    BinaryMessenger.BinaryReply reply = mock(BinaryMessenger.BinaryReply.class);
    binaryMessageHandler.onMessage((ByteBuffer) encodedMethodCall.flip(), reply);
    return reply;
  }

  @SuppressWarnings("deprecation")
  // setMessageHandler is deprecated.
  @Test
  public void respondsToGetKeyboardStateChannelMessage() {
    ArgumentCaptor<BinaryMessenger.BinaryMessageHandler> binaryMessageHandlerCaptor =
        ArgumentCaptor.forClass(BinaryMessenger.BinaryMessageHandler.class);
    DartExecutor mockBinaryMessenger = mock(DartExecutor.class);
    KeyboardChannel.KeyboardMethodHandler mockHandler =
        mock(KeyboardChannel.KeyboardMethodHandler.class);
    KeyboardChannel keyboardChannel = new KeyboardChannel(mockBinaryMessenger);

    verify(mockBinaryMessenger, times(1))
        .setMessageHandler(any(String.class), binaryMessageHandlerCaptor.capture());

    BinaryMessenger.BinaryMessageHandler binaryMessageHandler =
        binaryMessageHandlerCaptor.getValue();

    keyboardChannel.setKeyboardMethodHandler(mockHandler);
    sendToBinaryMessageHandler(binaryMessageHandler, "getKeyboardState", null);

    verify(mockHandler, times(1)).getKeyboardState();
  }

  @Test
  public void repliesWhenNoKeyboardChannelHandler() {
    // Regression test for https://github.com/flutter/flutter/issues/122441#issuecomment-1582052616.

    KeyboardChannel keyboardChannel = new KeyboardChannel(mock(DartExecutor.class));
    MethodCall methodCall = new MethodCall("getKeyboardState", null);
    MethodChannel.Result result = mock(MethodChannel.Result.class);
    keyboardChannel.parsingMethodHandler.onMethodCall(methodCall, result);

    verify(result).success(new HashMap());
  }
}
