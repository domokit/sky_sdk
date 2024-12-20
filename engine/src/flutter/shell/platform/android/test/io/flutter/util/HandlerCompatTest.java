// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.util;

import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.assertTrue;

import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

@RunWith(RobolectricTestRunner.class)
public class HandlerCompatTest {
  @Test
  @Config(sdk = API_LEVELS.API_28)
  public void createAsync_createsAnAsyncHandler() {
    Handler handler = Handler.createAsync(Looper.getMainLooper());

    Message message = Message.obtain();
    handler.sendMessageAtTime(message, 0);

    assertTrue(message.isAsynchronous());
  }
}
