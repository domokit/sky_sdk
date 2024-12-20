// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import static io.flutter.Build.API_LEVELS;
import static junit.framework.TestCase.assertEquals;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.isNull;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import android.annotation.TargetApi;
import android.util.DisplayMetrics;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.BasicMessageChannel;
import java.nio.ByteBuffer;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class SettingsChannelTest {
  @Test
  @TargetApi(API_LEVELS.API_33)
  @Config(sdk = API_LEVELS.API_33)
  @SuppressWarnings("deprecation")
  // DartExecutor.send is deprecated.
  public void setDisplayMetricsDoesNothingOnAPILevel33() {
    final DartExecutor executor = mock(DartExecutor.class);
    executor.onAttachedToJNI();
    final SettingsChannel settingsChannel = new SettingsChannel(executor);

    final ArgumentCaptor<ByteBuffer> messageCaptor = ArgumentCaptor.forClass(ByteBuffer.class);

    settingsChannel.startMessage().setDisplayMetrics(mock(DisplayMetrics.class)).send();

    verify(executor).send(eq("flutter/settings"), messageCaptor.capture(), isNull());
  }

  @Test
  public void configurationQueueWorks() {
    final SettingsChannel.ConfigurationQueue queue = new SettingsChannel.ConfigurationQueue();
    final int baseId = Integer.MIN_VALUE;

    queue.enqueueConfiguration(
        new SettingsChannel.ConfigurationQueue.SentConfiguration(mock(DisplayMetrics.class)));
    queue.enqueueConfiguration(
        new SettingsChannel.ConfigurationQueue.SentConfiguration(mock(DisplayMetrics.class)));
    assertEquals(baseId + 0, queue.getConfiguration(baseId + 0).generationNumber);
    assertEquals(baseId + 1, queue.getConfiguration(baseId + 1).generationNumber);
    assertEquals(baseId + 1, queue.getConfiguration(baseId + 1).generationNumber);

    queue.enqueueConfiguration(
        new SettingsChannel.ConfigurationQueue.SentConfiguration(mock(DisplayMetrics.class)));
    queue.enqueueConfiguration(
        new SettingsChannel.ConfigurationQueue.SentConfiguration(mock(DisplayMetrics.class)));
    assertEquals(baseId + 3, queue.getConfiguration(baseId + 3).generationNumber);
    // Can get the same configuration more than once.
    assertEquals(baseId + 3, queue.getConfiguration(baseId + 3).generationNumber);

    final BasicMessageChannel.Reply replyFor4 =
        queue.enqueueConfiguration(
            new SettingsChannel.ConfigurationQueue.SentConfiguration(mock(DisplayMetrics.class)));
    final BasicMessageChannel.Reply replyFor5 =
        queue.enqueueConfiguration(
            new SettingsChannel.ConfigurationQueue.SentConfiguration(mock(DisplayMetrics.class)));
    replyFor4.reply(null);
    replyFor5.reply(null);
    assertEquals(baseId + 5, queue.getConfiguration(baseId + 5).generationNumber);
    assertEquals(baseId + 5, queue.getConfiguration(baseId + 5).generationNumber);
  }

  // TODO(LongCatIsLooong): add tests for API 34 code path.
  // https://github.com/flutter/flutter/issues/128825
}
