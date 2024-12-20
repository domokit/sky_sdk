// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.mouse;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.view.PointerIcon;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import io.flutter.embedding.engine.systemchannels.MouseCursorChannel;
import java.util.HashMap;

/** A mandatory plugin that handles mouse cursor requests. */
@TargetApi(API_LEVELS.API_24)
@RequiresApi(API_LEVELS.API_24)
public class MouseCursorPlugin {
  @NonNull private final MouseCursorViewDelegate mView;
  @NonNull private final MouseCursorChannel mouseCursorChannel;

  public MouseCursorPlugin(
      @NonNull MouseCursorViewDelegate view, @NonNull MouseCursorChannel mouseCursorChannel) {
    mView = view;

    this.mouseCursorChannel = mouseCursorChannel;
    mouseCursorChannel.setMethodHandler(
        new MouseCursorChannel.MouseCursorMethodHandler() {
          @Override
          public void activateSystemCursor(@NonNull String kind) {
            mView.setPointerIcon(resolveSystemCursor(kind));
          }
        });
  }

  /**
   * Return a pointer icon object for a system cursor.
   *
   * <p>This method guarantees to return a non-null object.
   */
  private PointerIcon resolveSystemCursor(@NonNull String kind) {
    if (MouseCursorPlugin.systemCursorConstants == null) {
      // Initialize the map when first used, because the map can grow big in the future (~70)
      // and most mobile devices will not use them.
      MouseCursorPlugin.systemCursorConstants =
          new HashMap<String, Integer>() {
            private static final long serialVersionUID = 1L;

            {
              put("alias", PointerIcon.TYPE_ALIAS);
              put("allScroll", PointerIcon.TYPE_ALL_SCROLL);
              put("basic", PointerIcon.TYPE_ARROW);
              put("cell", PointerIcon.TYPE_CELL);
              put("click", PointerIcon.TYPE_HAND);
              put("contextMenu", PointerIcon.TYPE_CONTEXT_MENU);
              put("copy", PointerIcon.TYPE_COPY);
              put("forbidden", PointerIcon.TYPE_NO_DROP);
              put("grab", PointerIcon.TYPE_GRAB);
              put("grabbing", PointerIcon.TYPE_GRABBING);
              put("help", PointerIcon.TYPE_HELP);
              put("move", PointerIcon.TYPE_ALL_SCROLL);
              put("none", PointerIcon.TYPE_NULL);
              put("noDrop", PointerIcon.TYPE_NO_DROP);
              put("precise", PointerIcon.TYPE_CROSSHAIR);
              put("text", PointerIcon.TYPE_TEXT);
              put("resizeColumn", PointerIcon.TYPE_HORIZONTAL_DOUBLE_ARROW);
              put("resizeDown", PointerIcon.TYPE_VERTICAL_DOUBLE_ARROW);
              put("resizeUpLeft", PointerIcon.TYPE_TOP_RIGHT_DIAGONAL_DOUBLE_ARROW);
              put("resizeDownRight", PointerIcon.TYPE_TOP_LEFT_DIAGONAL_DOUBLE_ARROW);
              put("resizeLeft", PointerIcon.TYPE_HORIZONTAL_DOUBLE_ARROW);
              put("resizeLeftRight", PointerIcon.TYPE_HORIZONTAL_DOUBLE_ARROW);
              put("resizeRight", PointerIcon.TYPE_HORIZONTAL_DOUBLE_ARROW);
              put("resizeRow", PointerIcon.TYPE_VERTICAL_DOUBLE_ARROW);
              put("resizeUp", PointerIcon.TYPE_VERTICAL_DOUBLE_ARROW);
              put("resizeUpDown", PointerIcon.TYPE_VERTICAL_DOUBLE_ARROW);
              put("resizeUpLeft", PointerIcon.TYPE_TOP_LEFT_DIAGONAL_DOUBLE_ARROW);
              put("resizeUpRight", PointerIcon.TYPE_TOP_RIGHT_DIAGONAL_DOUBLE_ARROW);
              put("resizeUpLeftDownRight", PointerIcon.TYPE_TOP_LEFT_DIAGONAL_DOUBLE_ARROW);
              put("resizeUpRightDownLeft", PointerIcon.TYPE_TOP_RIGHT_DIAGONAL_DOUBLE_ARROW);
              put("verticalText", PointerIcon.TYPE_VERTICAL_TEXT);
              put("wait", PointerIcon.TYPE_WAIT);
              put("zoomIn", PointerIcon.TYPE_ZOOM_IN);
              put("zoomOut", PointerIcon.TYPE_ZOOM_OUT);
            }
          };
    }

    final int cursorConstant =
        MouseCursorPlugin.systemCursorConstants.getOrDefault(kind, PointerIcon.TYPE_ARROW);
    return mView.getSystemPointerIcon(cursorConstant);
  }

  /**
   * Detaches the text input plugin from the platform views controller.
   *
   * <p>The MouseCursorPlugin instance should not be used after calling this.
   */
  public void destroy() {
    mouseCursorChannel.setMethodHandler(null);
  }

  /**
   * A map from Flutter's system cursor {@code kind} to Android's pointer icon constants.
   *
   * <p>It is null until the first time a system cursor is requested, at which time it is filled
   * with the entire mapping.
   */
  @NonNull private static HashMap<String, Integer> systemCursorConstants;

  /**
   * Delegate interface for requesting the system to display a pointer icon object.
   *
   * <p>Typically implemented by an {@link android.view.View}, such as a {@code FlutterView}.
   */
  public interface MouseCursorViewDelegate {
    /**
     * Gets a system pointer icon object for the given {@code type}.
     *
     * <p>If typeis not recognized, returns the default pointer icon.
     *
     * <p>This is typically implemented by calling {@link android.view.PointerIcon#getSystemIcon}
     * with the context associated with this view.
     */
    @NonNull
    public PointerIcon getSystemPointerIcon(int type);

    /**
     * Request the pointer to display the specified icon object.
     *
     * <p>If the delegate is implemented by a {@link android.view.View}, then this method is
     * automatically implemented by View.
     */
    public void setPointerIcon(@NonNull PointerIcon icon);
  }
}
