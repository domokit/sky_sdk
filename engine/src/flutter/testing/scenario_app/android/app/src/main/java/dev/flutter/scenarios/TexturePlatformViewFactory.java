// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.view.Choreographer;
import android.view.TextureView;
import android.view.View;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.Log;
import io.flutter.plugin.common.MessageCodec;
import io.flutter.plugin.common.StringCodec;
import io.flutter.plugin.platform.PlatformView;
import io.flutter.plugin.platform.PlatformViewFactory;
import java.nio.ByteBuffer;

@TargetApi(API_LEVELS.API_23)
public final class TexturePlatformViewFactory extends PlatformViewFactory {
  TexturePlatformViewFactory() {
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
    return new TexturePlatformView(context);
  }

  private static class TexturePlatformView implements PlatformView {
    static String TAG = "TexturePlatformView";

    final TextureView textureView;

    @SuppressWarnings("unchecked")
    TexturePlatformView(@NonNull final Context context) {
      textureView = new TextureView(context);
      textureView.setSurfaceTextureListener(
          new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
              Log.i(TAG, "onSurfaceTextureAvailable");
              final Canvas canvas = textureView.lockCanvas();
              canvas.drawColor(Color.WHITE);

              final Paint paint = new Paint();
              paint.setColor(Color.GREEN);
              canvas.drawCircle(canvas.getWidth() / 2, canvas.getHeight() / 2, 20, paint);
              textureView.unlockCanvasAndPost(canvas);
              Choreographer.getInstance()
                  .postFrameCallbackDelayed(
                      new Choreographer.FrameCallback() {
                        @Override
                        public void doFrame(long frameTimeNanos) {
                          textureView.invalidate();
                        }
                      },
                      500);
            }

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
              Log.i(TAG, "onSurfaceTextureDestroyed");
              return true;
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
              Log.i(TAG, "onSurfaceTextureSizeChanged");
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture surface) {
              Log.i(TAG, "onSurfaceTextureUpdated");
            }
          });
    }

    @Override
    @NonNull
    public View getView() {
      return textureView;
    }

    @Override
    public void dispose() {}
  }
}
