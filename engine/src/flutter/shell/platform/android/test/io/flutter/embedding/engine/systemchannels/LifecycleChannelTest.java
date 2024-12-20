// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.plugin.common.BasicMessageChannel;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class LifecycleChannelTest {
  LifecycleChannel lifecycleChannel;
  BasicMessageChannel<String> mockChannel;

  @Before
  public void setUp() {
    mockChannel = mock(BasicMessageChannel.class);
    lifecycleChannel = new LifecycleChannel(mockChannel);
  }

  @Test
  public void lifecycleChannel_handlesResumed() {
    lifecycleChannel.appIsResumed();
    ArgumentCaptor<String> stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(1)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.resumed", stringArgumentCaptor.getValue());

    lifecycleChannel.noWindowsAreFocused();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(2)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.inactive", stringArgumentCaptor.getValue());

    lifecycleChannel.aWindowIsFocused();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(3)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.resumed", stringArgumentCaptor.getValue());

    // Stays inactive, so no event is sent.
    lifecycleChannel.appIsInactive();
    verify(mockChannel, times(4)).send(any(String.class));

    // Stays inactive, so no event is sent.
    lifecycleChannel.appIsResumed();
    verify(mockChannel, times(5)).send(any(String.class));

    lifecycleChannel.aWindowIsFocused();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(5)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.resumed", stringArgumentCaptor.getValue());
  }

  @Test
  public void lifecycleChannel_handlesInactive() {
    lifecycleChannel.appIsInactive();
    ArgumentCaptor<String> stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(1)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.inactive", stringArgumentCaptor.getValue());

    // Stays inactive, so no event is sent.
    lifecycleChannel.aWindowIsFocused();
    verify(mockChannel, times(1)).send(any(String.class));

    // Stays inactive, so no event is sent.
    lifecycleChannel.noWindowsAreFocused();
    verify(mockChannel, times(1)).send(any(String.class));

    lifecycleChannel.appIsResumed();
    lifecycleChannel.aWindowIsFocused();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(2)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.resumed", stringArgumentCaptor.getValue());
  }

  @Test
  public void lifecycleChannel_handlesPaused() {
    // Stays inactive, so no event is sent.
    lifecycleChannel.appIsPaused();
    ArgumentCaptor<String> stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(1)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.paused", stringArgumentCaptor.getValue());

    // Stays paused, so no event is sent.
    lifecycleChannel.aWindowIsFocused();
    verify(mockChannel, times(1)).send(any(String.class));

    lifecycleChannel.noWindowsAreFocused();
    verify(mockChannel, times(1)).send(any(String.class));

    lifecycleChannel.appIsResumed();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(2)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.inactive", stringArgumentCaptor.getValue());

    lifecycleChannel.aWindowIsFocused();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(3)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.resumed", stringArgumentCaptor.getValue());
  }

  @Test
  public void lifecycleChannel_handlesDetached() {
    // Stays inactive, so no event is sent.
    lifecycleChannel.appIsDetached();
    ArgumentCaptor<String> stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(1)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.detached", stringArgumentCaptor.getValue());

    // Stays paused, so no event is sent.
    lifecycleChannel.aWindowIsFocused();
    verify(mockChannel, times(1)).send(any(String.class));

    lifecycleChannel.noWindowsAreFocused();
    verify(mockChannel, times(1)).send(any(String.class));

    lifecycleChannel.appIsResumed();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(2)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.inactive", stringArgumentCaptor.getValue());

    lifecycleChannel.aWindowIsFocused();
    stringArgumentCaptor = ArgumentCaptor.forClass(String.class);
    verify(mockChannel, times(3)).send(stringArgumentCaptor.capture());
    assertEquals("AppLifecycleState.resumed", stringArgumentCaptor.getValue());
  }
}
