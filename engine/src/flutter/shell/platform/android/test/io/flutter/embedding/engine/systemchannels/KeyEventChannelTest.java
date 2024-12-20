// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import android.annotation.TargetApi;
import android.view.KeyEvent;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.JSONMessageCodec;
import io.flutter.util.FakeKeyEvent;
import java.nio.ByteBuffer;
import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_35)
public class KeyEventChannelTest {

  KeyEvent keyEvent;
  @Mock BinaryMessenger fakeMessenger;
  boolean[] handled;
  KeyEventChannel keyEventChannel;

  private void sendReply(boolean handled, BinaryMessenger.BinaryReply messengerReply)
      throws JSONException {
    JSONObject reply = new JSONObject();
    reply.put("handled", true);
    ByteBuffer binaryReply = JSONMessageCodec.INSTANCE.encodeMessage(reply);
    assertNotNull(binaryReply);
    binaryReply.rewind();
    messengerReply.reply(binaryReply);
  }

  @Before
  public void setUp() {
    MockitoAnnotations.openMocks(this);
    keyEvent = new FakeKeyEvent(KeyEvent.ACTION_DOWN, 65);
    handled = new boolean[] {false};
    keyEventChannel = new KeyEventChannel(fakeMessenger);
  }

  @Test
  public void keyDownEventIsSentToFramework() throws JSONException {
    KeyEventChannel.FlutterKeyEvent flutterKeyEvent =
        new KeyEventChannel.FlutterKeyEvent(keyEvent, null);
    keyEventChannel.sendFlutterKeyEvent(
        flutterKeyEvent,
        false,
        (isHandled) -> {
          handled[0] = isHandled;
        });

    ArgumentCaptor<ByteBuffer> byteBufferArgumentCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    ArgumentCaptor<BinaryMessenger.BinaryReply> replyArgumentCaptor =
        ArgumentCaptor.forClass(BinaryMessenger.BinaryReply.class);
    verify(fakeMessenger, times(1))
        .send(any(), byteBufferArgumentCaptor.capture(), replyArgumentCaptor.capture());
    ByteBuffer capturedMessage = byteBufferArgumentCaptor.getValue();
    capturedMessage.rewind();
    JSONObject message = (JSONObject) JSONMessageCodec.INSTANCE.decodeMessage(capturedMessage);
    assertNotNull(message);
    assertEquals("keydown", message.get("type"));

    // Simulate a reply, and see that it is handled.
    sendReply(true, replyArgumentCaptor.getValue());
    assertTrue(handled[0]);
  }

  @Test
  public void keyUpEventIsSentToFramework() throws JSONException {
    keyEvent = new FakeKeyEvent(KeyEvent.ACTION_UP, 65);
    KeyEventChannel.FlutterKeyEvent flutterKeyEvent =
        new KeyEventChannel.FlutterKeyEvent(keyEvent, null);
    keyEventChannel.sendFlutterKeyEvent(
        flutterKeyEvent,
        false,
        (isHandled) -> {
          handled[0] = isHandled;
        });

    ArgumentCaptor<ByteBuffer> byteBufferArgumentCaptor = ArgumentCaptor.forClass(ByteBuffer.class);
    ArgumentCaptor<BinaryMessenger.BinaryReply> replyArgumentCaptor =
        ArgumentCaptor.forClass(BinaryMessenger.BinaryReply.class);
    verify(fakeMessenger, times(1))
        .send(any(), byteBufferArgumentCaptor.capture(), replyArgumentCaptor.capture());
    ByteBuffer capturedMessage = byteBufferArgumentCaptor.getValue();
    capturedMessage.rewind();
    JSONObject message = (JSONObject) JSONMessageCodec.INSTANCE.decodeMessage(capturedMessage);
    assertNotNull(message);
    assertEquals("keydown", message.get("type"));

    // Simulate a reply, and see that it is handled.
    sendReply(true, replyArgumentCaptor.getValue());
    assertTrue(handled[0]);
  }
}
