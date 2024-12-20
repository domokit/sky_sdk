// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static org.robolectric.util.reflector.Reflector.reflector;

import android.content.res.Resources;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.graphics.drawable.Drawable;
import androidx.annotation.Nullable;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;
import org.robolectric.annotation.RealObject;
import org.robolectric.shadows.ShadowResources;
import org.robolectric.util.reflector.Direct;
import org.robolectric.util.reflector.ForType;

@SuppressWarnings("deprecation")
// getDrawableInt
@Implements(Resources.class)
public class SplashShadowResources extends ShadowResources {
  @RealObject private Resources resources;

  public static final int SPLASH_DRAWABLE_ID = 191919;
  public static final int THEMED_SPLASH_DRAWABLE_ID = 212121;

  @ForType(Resources.class)
  interface ResourcesReflector {
    @Direct
    Drawable getDrawable(int id, Resources.Theme theme);

    @Direct
    Drawable getDrawable(int id);
  }

  @Implementation
  protected Drawable getDrawable(int id) {
    if (id == SPLASH_DRAWABLE_ID) {
      return new ColorDrawable(Color.BLUE);
    }
    return reflector(Resources.class, resources).getDrawable(id);
  }

  @Implementation
  protected Drawable getDrawable(int id, @Nullable Resources.Theme theme) {
    if (id == THEMED_SPLASH_DRAWABLE_ID) {
      // We pretend the drawable contains theme references. It can't be parsed without the app
      // theme.
      if (theme == null) {
        throw new Resources.NotFoundException(
            "Cannot parse drawable due to missing theme references.");
      }
      return new ColorDrawable(Color.GRAY);
    }
    return reflector(Resources.class, resources).getDrawable(id, theme);
  }
}
