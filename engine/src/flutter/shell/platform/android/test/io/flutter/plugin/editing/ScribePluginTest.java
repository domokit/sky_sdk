// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.editing;

import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.TargetApi;
import android.content.Context;
import android.os.Build;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.systemchannels.ScribeChannel;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@RunWith(AndroidJUnit4.class)
public class ScribePluginTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  ScribePlugin scribePlugin;
  InputMethodManager mockImm;
  View testView;

  @Before
  public void setUp() {
    ScribeChannel mockScribeChannel = mock(ScribeChannel.class);
    testView = new View(ctx);
    mockImm = mock(InputMethodManager.class);
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_34) {
      when(mockImm.isStylusHandwritingAvailable()).thenReturn(true);
    }
    scribePlugin = new ScribePlugin(testView, mockImm, mockScribeChannel);
  }

  @Config(minSdk = API_LEVELS.API_34)
  @TargetApi(API_LEVELS.API_34)
  @Test
  public void scribePluginIsFeatureAvailable() {
    assertEquals(scribePlugin.isFeatureAvailable(), true);

    verify(mockImm).isStylusHandwritingAvailable();
  }

  @Config(minSdk = API_LEVELS.API_34)
  @TargetApi(API_LEVELS.API_34)
  @Test
  public void scribePluginIsStylusHandwritingAvailable() {
    assertEquals(scribePlugin.isStylusHandwritingAvailable(), true);

    verify(mockImm).isStylusHandwritingAvailable();
  }

  @Config(minSdk = API_LEVELS.API_34)
  @TargetApi(API_LEVELS.API_34)
  @Test
  public void scribePluginStartStylusHandwriting() {
    scribePlugin.startStylusHandwriting();

    verify(mockImm).startStylusHandwriting(testView);
  }

  @Config(sdk = API_LEVELS.API_32)
  @TargetApi(API_LEVELS.API_32)
  @Test
  public void scribePluginStartStylusHandwritingWhenAPILevelUnsupported() {
    assertNotNull(scribePlugin);

    assertThrows(
        NoSuchMethodError.class,
        () -> {
          scribePlugin.startStylusHandwriting();
        });
  }

  @Config(sdk = API_LEVELS.API_33)
  @TargetApi(API_LEVELS.API_33)
  @Test
  public void scribePluginIsFeatureAvailableWhenAPILevelUnsupported() {
    assertEquals(scribePlugin.isFeatureAvailable(), false);
  }

  @Config(sdk = API_LEVELS.API_33)
  @TargetApi(API_LEVELS.API_33)
  @Test
  public void scribePluginIsStylusHandwritingAvailableWhenAPILevelUnsupported() {
    assertNotNull(scribePlugin);

    assertThrows(
        NoSuchMethodError.class,
        () -> {
          scribePlugin.isStylusHandwritingAvailable();
        });
  }
}
