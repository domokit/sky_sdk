// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static io.flutter.Build.API_LEVELS;
import static junit.framework.TestCase.assertEquals;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.doAnswer;

import android.annotation.TargetApi;
import android.view.KeyEvent;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.systemchannels.KeyEventChannel;
import io.flutter.embedding.engine.systemchannels.KeyEventChannel.EventResponseHandler;
import io.flutter.embedding.engine.systemchannels.KeyEventChannel.FlutterKeyEvent;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_28)
public class KeyChannelResponderTest {

  @Mock KeyEventChannel keyEventChannel;
  KeyChannelResponder channelResponder;

  @Before
  public void setUp() {
    MockitoAnnotations.openMocks(this);
    channelResponder = new KeyChannelResponder(keyEventChannel);
  }

  @Test
  public void primaryResponderTest() {
    final int[] completionCallbackInvocationCounter = {0};

    doAnswer(
            invocation -> {
              ((EventResponseHandler) invocation.getArgument(2)).onFrameworkResponse(true);
              return null;
            })
        .when(keyEventChannel)
        .sendFlutterKeyEvent(
            any(FlutterKeyEvent.class), any(boolean.class), any(EventResponseHandler.class));

    final KeyEvent keyEvent = new KeyEvent(KeyEvent.ACTION_DOWN, 65);
    channelResponder.handleEvent(
        keyEvent,
        (canHandleEvent) -> {
          completionCallbackInvocationCounter[0]++;
        });
    assertEquals(completionCallbackInvocationCounter[0], 1);
  }
}
