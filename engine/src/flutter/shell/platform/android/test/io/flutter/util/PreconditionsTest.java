// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class PreconditionsTest {
  @Test
  public void checkNotNull_notNull() {
    // Should always return its input.
    assertEquals("non-null", Preconditions.checkNotNull("non-null"));
    assertEquals(42, (int) Preconditions.checkNotNull(42));
    Object classParam = new Object();
    assertEquals(classParam, Preconditions.checkNotNull(classParam));
  }

  @Test
  public void checkNotNull_Null() {
    assertThrows(
        NullPointerException.class,
        () -> {
          Preconditions.checkNotNull(null);
        });
  }
}
