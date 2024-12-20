// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import android.content.Context;
import android.graphics.Color;
import android.view.View;
import android.widget.TextView;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.plugin.common.MessageCodec;
import io.flutter.plugin.common.StringCodec;
import io.flutter.plugin.platform.PlatformView;
import io.flutter.plugin.platform.PlatformViewFactory;
import java.nio.ByteBuffer;

public final class TextPlatformViewFactory extends PlatformViewFactory {
  TextPlatformViewFactory() {
    super(
        new MessageCodec<Object>() {
          @Nullable
          @Override
          public ByteBuffer encodeMessage(@Nullable Object o) {
            if (o instanceof String) {
              return StringCodec.INSTANCE.encodeMessage((String) o);
            }
            return null;
          }

          @Nullable
          @Override
          public Object decodeMessage(@Nullable ByteBuffer byteBuffer) {
            return StringCodec.INSTANCE.decodeMessage(byteBuffer);
          }
        });
  }

  @SuppressWarnings("unchecked")
  @Override
  @NonNull
  public PlatformView create(@NonNull Context context, int id, @Nullable Object args) {
    String params = (String) args;
    return new TextPlatformView(context, id, params);
  }

  private static class TextPlatformView implements PlatformView {
    final TextView textView;

    @SuppressWarnings("unchecked")
    TextPlatformView(@NonNull final Context context, int id, @Nullable String params) {
      textView = new TextView(context);
      textView.setTextSize(72);
      textView.setBackgroundColor(Color.WHITE);
      textView.setText(params);
    }

    @Override
    @NonNull
    public View getView() {
      return textView;
    }

    @Override
    public void dispose() {}
  }
}
