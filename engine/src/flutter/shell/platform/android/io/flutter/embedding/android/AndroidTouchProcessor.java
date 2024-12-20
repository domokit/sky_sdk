package io.flutter.embedding.android;

import static io.flutter.Build.API_LEVELS;

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Matrix;
import android.os.Build;
import android.util.TypedValue;
import android.view.InputDevice;
import android.view.MotionEvent;
import android.view.ViewConfiguration;
import androidx.annotation.IntDef;
import androidx.annotation.NonNull;
import androidx.annotation.VisibleForTesting;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;

/** Sends touch information from Android to Flutter in a format that Flutter understands. */
public class AndroidTouchProcessor {
  private static final String TAG = "AndroidTouchProcessor";
  // Must match the PointerChange enum in pointer.dart.
  @IntDef({
    PointerChange.CANCEL,
    PointerChange.ADD,
    PointerChange.REMOVE,
    PointerChange.HOVER,
    PointerChange.DOWN,
    PointerChange.MOVE,
    PointerChange.UP,
    PointerChange.PAN_ZOOM_START,
    PointerChange.PAN_ZOOM_UPDATE,
    PointerChange.PAN_ZOOM_END
  })
  public @interface PointerChange {
    int CANCEL = 0;
    int ADD = 1;
    int REMOVE = 2;
    int HOVER = 3;
    int DOWN = 4;
    int MOVE = 5;
    int UP = 6;
    int PAN_ZOOM_START = 7;
    int PAN_ZOOM_UPDATE = 8;
    int PAN_ZOOM_END = 9;
  }

  // Must match the PointerDeviceKind enum in pointer.dart.
  @IntDef({
    PointerDeviceKind.TOUCH,
    PointerDeviceKind.MOUSE,
    PointerDeviceKind.STYLUS,
    PointerDeviceKind.INVERTED_STYLUS,
    PointerDeviceKind.TRACKPAD,
    PointerDeviceKind.UNKNOWN
  })
  public @interface PointerDeviceKind {
    int TOUCH = 0;
    int MOUSE = 1;
    int STYLUS = 2;
    int INVERTED_STYLUS = 3;
    int TRACKPAD = 4;
    int UNKNOWN = 5;
  }

  // Must match the PointerSignalKind enum in pointer.dart.
  @IntDef({
    PointerSignalKind.NONE,
    PointerSignalKind.SCROLL,
    PointerSignalKind.SCROLL_INERTIA_CANCEL,
    PointerSignalKind.SCALE,
    PointerSignalKind.UNKNOWN
  })
  public @interface PointerSignalKind {
    int NONE = 0;
    int SCROLL = 1;
    int SCROLL_INERTIA_CANCEL = 2;
    int SCALE = 3;
    int UNKNOWN = 4;
  }

  // This value must match kPointerDataFieldCount in pointer_data.cc. (The
  // pointer_data.cc also lists other locations that must be kept consistent.)
  @VisibleForTesting static final int POINTER_DATA_FIELD_COUNT = 36;
  @VisibleForTesting static final int BYTES_PER_FIELD = 8;

  // Default if context is null, chosen to ensure reasonable speed scrolling.
  @VisibleForTesting static final int DEFAULT_VERTICAL_SCROLL_FACTOR = 48;
  @VisibleForTesting static final int DEFAULT_HORIZONTAL_SCROLL_FACTOR = 48;

  // This value must match the value in framework's platform_view.dart.
  // This flag indicates whether the original Android pointer events were batched together.
  private static final int POINTER_DATA_FLAG_BATCHED = 1;

  // The view ID for the only view in a single-view Flutter app.
  private static final int IMPLICIT_VIEW_ID = 0;

  @NonNull private final FlutterRenderer renderer;
  @NonNull private final MotionEventTracker motionEventTracker;

  private static final Matrix IDENTITY_TRANSFORM = new Matrix();

  private final boolean trackMotionEvents;

  private final Map<Integer, float[]> ongoingPans = new HashMap<>();

  // Only used on api 25 and below to avoid requerying display metrics.
  private int cachedVerticalScrollFactor;

  /**
   * Constructs an {@code AndroidTouchProcessor} that will send touch event data to the Flutter
   * execution context represented by the given {@link FlutterRenderer}.
   *
   * @param renderer The object that manages textures for rendering.
   * @param trackMotionEvents This is used to query motion events when platform views are rendered.
   */
  // TODO(mattcarroll): consider moving packet behavior to a FlutterInteractionSurface instead of
  // FlutterRenderer
  public AndroidTouchProcessor(@NonNull FlutterRenderer renderer, boolean trackMotionEvents) {
    this.renderer = renderer;
    this.motionEventTracker = MotionEventTracker.getInstance();
    this.trackMotionEvents = trackMotionEvents;
  }

  public boolean onTouchEvent(@NonNull MotionEvent event) {
    return onTouchEvent(event, IDENTITY_TRANSFORM);
  }

  /**
   * Sends the given {@link MotionEvent} data to Flutter in a format that Flutter understands.
   *
   * @param event The motion event from the view.
   * @param transformMatrix Applies to the view that originated the event. It's used to transform
   *     the gesture pointers into screen coordinates.
   * @return True if the event was handled.
   */
  public boolean onTouchEvent(@NonNull MotionEvent event, @NonNull Matrix transformMatrix) {
    int maskedAction = event.getActionMasked();
    int pointerChange = getPointerChangeForAction(event.getActionMasked());
    boolean updateForSinglePointer =
        maskedAction == MotionEvent.ACTION_DOWN || maskedAction == MotionEvent.ACTION_POINTER_DOWN;
    boolean updateForMultiplePointers =
        !updateForSinglePointer
            && (maskedAction == MotionEvent.ACTION_UP
                || maskedAction == MotionEvent.ACTION_POINTER_UP);

    int deviceType = getPointerDeviceTypeForToolType(event.getToolType(event.getActionIndex()));
    boolean shouldRemovePointer =
        updateForMultiplePointers && (deviceType == PointerDeviceKind.TOUCH);
    int originalPointerCount = event.getPointerCount();

    // The following packing code must match the struct in pointer_data.h.

    // Prepare a data packet of the appropriate size and order.
    // Allocate space for an additional pointer if this is an ACTION_UP or ACTION_POINTER_UP
    // event taken with device type touch, to handle the synthesized PointerChange.REMOVE event.
    int totalPointerCount = originalPointerCount + (shouldRemovePointer ? 1 : 0);
    ByteBuffer packet =
        ByteBuffer.allocateDirect(totalPointerCount * POINTER_DATA_FIELD_COUNT * BYTES_PER_FIELD);
    packet.order(ByteOrder.LITTLE_ENDIAN);

    if (updateForSinglePointer) {
      // ACTION_DOWN and ACTION_POINTER_DOWN always apply to a single pointer only.
      addPointerForIndex(event, event.getActionIndex(), pointerChange, 0, transformMatrix, packet);
    } else if (updateForMultiplePointers) {
      // ACTION_UP and ACTION_POINTER_UP may contain position updates for other pointers.
      // We are converting these updates to move events here in order to preserve this data.
      // We also mark these events with a flag in order to help the framework reassemble
      // the original Android event later, should it need to forward it to a PlatformView.
      for (int p = 0; p < originalPointerCount; p++) {
        if (p != event.getActionIndex() && event.getToolType(p) == MotionEvent.TOOL_TYPE_FINGER) {
          addPointerForIndex(
              event, p, PointerChange.MOVE, POINTER_DATA_FLAG_BATCHED, transformMatrix, packet);
        }
      }
      // It's important that we're sending the UP event last. This allows PlatformView
      // to correctly batch everything back into the original Android event if needed.
      addPointerForIndex(event, event.getActionIndex(), pointerChange, 0, transformMatrix, packet);

      if (shouldRemovePointer) {
        // Synthesizes remove events immediately after the UP event so that the touches
        // are divided into distinct segments, each beginning with a DOWN event and ending with an
        // UP event.
        // This approach makes sense since each segment can be considered a separate pointer,
        // and it prevents Flutter from generating hover events between these segments, which are
        // not applicable to touch screens, as hovering is not possible.
        // (Flutter will automatically synthesize an add event before the pointer makes its next
        // contact.)
        addPointerForIndex(
            event, event.getActionIndex(), PointerChange.REMOVE, 0, transformMatrix, packet);
      }
    } else {
      // ACTION_MOVE may not actually mean all pointers have moved
      // but it's the responsibility of a later part of the system to
      // ignore 0-deltas if desired.
      for (int p = 0; p < originalPointerCount; p++) {
        addPointerForIndex(event, p, pointerChange, 0, transformMatrix, packet);
      }
    }

    // Verify that the packet is the expected size.
    if (packet.position() % (POINTER_DATA_FIELD_COUNT * BYTES_PER_FIELD) != 0) {
      throw new AssertionError("Packet position is not on field boundary");
    }

    // Send the packet to Flutter.
    renderer.dispatchPointerDataPacket(packet, packet.position());

    return true;
  }

  /**
   * Sends the given generic {@link MotionEvent} data to Flutter in a format that Flutter
   * understands.
   *
   * <p>Generic motion events include joystick movement, mouse hover, track pad touches, scroll
   * wheel movements, etc.
   *
   * @param event The generic motion event being processed.
   * @param context For use by ViewConfiguration.get(context) to scale input.
   * @return True if the event was handled.
   */
  public boolean onGenericMotionEvent(@NonNull MotionEvent event, @NonNull Context context) {
    // Method isFromSource is only available in API 18+ (Jelly Bean MR2)
    // Mouse hover support is not implemented for API < 18.
    boolean isPointerEvent = event.isFromSource(InputDevice.SOURCE_CLASS_POINTER);
    boolean isMovementEvent =
        (event.getActionMasked() == MotionEvent.ACTION_HOVER_MOVE
            || event.getActionMasked() == MotionEvent.ACTION_SCROLL);
    if (isPointerEvent && isMovementEvent) {
      // Continue.
    } else {
      return false;
    }

    int pointerChange = getPointerChangeForAction(event.getActionMasked());
    ByteBuffer packet =
        ByteBuffer.allocateDirect(
            event.getPointerCount() * POINTER_DATA_FIELD_COUNT * BYTES_PER_FIELD);
    packet.order(ByteOrder.LITTLE_ENDIAN);

    // ACTION_HOVER_MOVE always applies to a single pointer only.
    addPointerForIndex(
        event, event.getActionIndex(), pointerChange, 0, IDENTITY_TRANSFORM, packet, context);
    if (packet.position() % (POINTER_DATA_FIELD_COUNT * BYTES_PER_FIELD) != 0) {
      throw new AssertionError("Packet position is not on field boundary.");
    }
    renderer.dispatchPointerDataPacket(packet, packet.position());
    return true;
  }

  /// Calls addPointerForIndex with null for context.
  ///
  /// Without context the scroll wheel will not mimick android's scroll speed.
  private void addPointerForIndex(
      MotionEvent event,
      int pointerIndex,
      int pointerChange,
      int pointerData,
      Matrix transformMatrix,
      ByteBuffer packet) {
    addPointerForIndex(
        event, pointerIndex, pointerChange, pointerData, transformMatrix, packet, null);
  }

  // TODO: consider creating a PointerPacket class instead of using a procedure that
  // mutates inputs. https://github.com/flutter/flutter/issues/132853
  private void addPointerForIndex(
      MotionEvent event,
      int pointerIndex,
      int pointerChange,
      int pointerData,
      Matrix transformMatrix,
      ByteBuffer packet,
      Context context) {
    if (pointerChange == -1) {
      return;
    }
    // TODO(dkwingsmt): Use the correct source view ID once Android supports
    // multiple views.
    // https://github.com/flutter/flutter/issues/134405
    final int viewId = IMPLICIT_VIEW_ID;
    final int pointerId = event.getPointerId(pointerIndex);

    int pointerKind = getPointerDeviceTypeForToolType(event.getToolType(pointerIndex));
    // We use this in lieu of using event.getRawX and event.getRawY as we wish to support
    // earlier versions than API level 29.
    float viewToScreenCoords[] = {event.getX(pointerIndex), event.getY(pointerIndex)};
    transformMatrix.mapPoints(viewToScreenCoords);
    long buttons;
    if (pointerKind == PointerDeviceKind.MOUSE) {
      buttons = event.getButtonState() & 0x1F;
      if (buttons == 0
          && event.getSource() == InputDevice.SOURCE_MOUSE
          && pointerChange == PointerChange.DOWN) {
        // Some implementations translate trackpad scrolling into a mouse down-move-up event
        // sequence with buttons: 0, such as ARC on a Chromebook. See #11420, a legacy
        // implementation that uses the same condition but converts differently.
        ongoingPans.put(pointerId, viewToScreenCoords);
      }
    } else if (pointerKind == PointerDeviceKind.STYLUS) {
      // Returns converted android button state into flutter framework normalized state
      // and updates ongoingPans for chromebook trackpad scrolling.
      // See
      // https://github.com/flutter/flutter/blob/master/packages/flutter/lib/src/gestures/events.dart
      // for target button constants.
      buttons = (event.getButtonState() >> 4) & 0xF;
    } else {
      buttons = 0;
    }

    int panZoomType = -1;
    boolean isTrackpadPan = ongoingPans.containsKey(pointerId);
    if (isTrackpadPan) {
      panZoomType = getPointerChangeForPanZoom(pointerChange);
      if (panZoomType == -1) {
        return;
      }
    }

    long motionEventId = 0;
    if (trackMotionEvents) {
      MotionEventTracker.MotionEventId trackedEvent = motionEventTracker.track(event);
      motionEventId = trackedEvent.getId();
    }

    int signalKind =
        event.getActionMasked() == MotionEvent.ACTION_SCROLL
            ? PointerSignalKind.SCROLL
            : PointerSignalKind.NONE;

    long timeStamp = event.getEventTime() * 1000; // Convert from milliseconds to microseconds.

    packet.putLong(motionEventId); // motionEventId
    packet.putLong(timeStamp); // time_stamp
    if (isTrackpadPan) {
      packet.putLong(panZoomType); // change
      packet.putLong(PointerDeviceKind.TRACKPAD); // kind
    } else {
      packet.putLong(pointerChange); // change
      packet.putLong(pointerKind); // kind
    }
    packet.putLong(signalKind); // signal_kind
    packet.putLong(pointerId); // device
    packet.putLong(0); // pointer_identifier, will be generated in pointer_data_packet_converter.cc.

    if (isTrackpadPan) {
      float[] panStart = ongoingPans.get(pointerId);
      packet.putDouble(panStart[0]); // physical_x
      packet.putDouble(panStart[1]); // physical_y
    } else {
      packet.putDouble(viewToScreenCoords[0]); // physical_x
      packet.putDouble(viewToScreenCoords[1]); // physical_y
    }

    packet.putDouble(
        0.0); // physical_delta_x, will be generated in pointer_data_packet_converter.cc.
    packet.putDouble(
        0.0); // physical_delta_y, will be generated in pointer_data_packet_converter.cc.

    packet.putLong(buttons); // buttons

    packet.putLong(0); // obscured

    packet.putLong(0); // synthesized

    packet.putDouble(event.getPressure(pointerIndex)); // pressure
    double pressureMin = 0.0;
    double pressureMax = 1.0;
    if (event.getDevice() != null) {
      InputDevice.MotionRange pressureRange =
          event.getDevice().getMotionRange(MotionEvent.AXIS_PRESSURE);
      if (pressureRange != null) {
        pressureMin = pressureRange.getMin();
        pressureMax = pressureRange.getMax();
      }
    }
    packet.putDouble(pressureMin); // pressure_min
    packet.putDouble(pressureMax); // pressure_max

    if (pointerKind == PointerDeviceKind.STYLUS) {
      packet.putDouble(event.getAxisValue(MotionEvent.AXIS_DISTANCE, pointerIndex)); // distance
      packet.putDouble(0.0); // distance_max
    } else {
      packet.putDouble(0.0); // distance
      packet.putDouble(0.0); // distance_max
    }

    packet.putDouble(event.getSize(pointerIndex)); // size

    packet.putDouble(event.getToolMajor(pointerIndex)); // radius_major
    packet.putDouble(event.getToolMinor(pointerIndex)); // radius_minor

    packet.putDouble(0.0); // radius_min
    packet.putDouble(0.0); // radius_max

    packet.putDouble(event.getAxisValue(MotionEvent.AXIS_ORIENTATION, pointerIndex)); // orientation

    if (pointerKind == PointerDeviceKind.STYLUS) {
      packet.putDouble(event.getAxisValue(MotionEvent.AXIS_TILT, pointerIndex)); // tilt
    } else {
      packet.putDouble(0.0); // tilt
    }

    packet.putLong(pointerData); // platformData

    // See android scrollview for inspiration.
    // https://cs.android.com/android/platform/superproject/main/+/main:frameworks/base/core/java/android/widget/ScrollView.java?q=function:onGenericMotionEvent%20filepath:widget%2FScrollView.java&ss=android%2Fplatform%2Fsuperproject%2Fmain
    if (signalKind == PointerSignalKind.SCROLL) {
      double horizontalScaleFactor = DEFAULT_HORIZONTAL_SCROLL_FACTOR;
      double verticalScaleFactor = DEFAULT_VERTICAL_SCROLL_FACTOR;
      if (context != null) {
        horizontalScaleFactor = getHorizontalScrollFactor(context);
        verticalScaleFactor = getVerticalScrollFactor(context);
      }
      // We flip the sign of the scroll value below because it aligns the pixel value with the
      // scroll direction in native android.
      final double horizontalScrollPixels =
          horizontalScaleFactor * -event.getAxisValue(MotionEvent.AXIS_HSCROLL, pointerIndex);
      final double verticalScrollPixels =
          verticalScaleFactor * -event.getAxisValue(MotionEvent.AXIS_VSCROLL, pointerIndex);
      packet.putDouble(horizontalScrollPixels); // scroll_delta_x
      packet.putDouble(verticalScrollPixels); // scroll_delta_y
    } else {
      packet.putDouble(0.0); // scroll_delta_x
      packet.putDouble(0.0); // scroll_delta_y
    }

    if (isTrackpadPan) {
      float[] panStart = ongoingPans.get(pointerId);
      packet.putDouble(viewToScreenCoords[0] - panStart[0]);
      packet.putDouble(viewToScreenCoords[1] - panStart[1]);
    } else {
      packet.putDouble(0.0); // pan_x
      packet.putDouble(0.0); // pan_y
    }
    packet.putDouble(0.0); // pan_delta_x
    packet.putDouble(0.0); // pan_delta_y
    packet.putDouble(1.0); // scale
    packet.putDouble(0.0); // rotation
    packet.putLong(viewId); // view_id

    if (isTrackpadPan && (panZoomType == PointerChange.PAN_ZOOM_END)) {
      ongoingPans.remove(pointerId);
    }
  }

  private float getHorizontalScrollFactor(@NonNull Context context) {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_26) {
      return ViewConfiguration.get(context).getScaledHorizontalScrollFactor();
    } else {
      // Vertical scroll factor is not a typo. This is what View.java does in android.
      return getVerticalScrollFactorPre26(context);
    }
  }

  private float getVerticalScrollFactor(@NonNull Context context) {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_26) {
      return getVerticalScrollFactorAbove26(context);
    } else {
      return getVerticalScrollFactorPre26(context);
    }
  }

  @TargetApi(API_LEVELS.API_26)
  private float getVerticalScrollFactorAbove26(@NonNull Context context) {
    return ViewConfiguration.get(context).getScaledVerticalScrollFactor();
  }

  // See
  // https://cs.android.com/android/platform/superproject/main/+/main:frameworks/base/core/java/android/view/View.java?q=function:getVerticalScrollFactor%20filepath:android%2Fview%2FView.java&ss=android%2Fplatform%2Fsuperproject%2Fmain
  private int getVerticalScrollFactorPre26(@NonNull Context context) {
    if (cachedVerticalScrollFactor == 0) {
      TypedValue outValue = new TypedValue();
      if (!context
          .getTheme()
          .resolveAttribute(android.R.attr.listPreferredItemHeight, outValue, true)) {
        return DEFAULT_VERTICAL_SCROLL_FACTOR;
      }
      cachedVerticalScrollFactor =
          (int) outValue.getDimension(context.getResources().getDisplayMetrics());
    }
    return cachedVerticalScrollFactor;
  }

  @PointerChange
  private int getPointerChangeForAction(int maskedAction) {
    // Primary pointer:
    if (maskedAction == MotionEvent.ACTION_DOWN) {
      return PointerChange.DOWN;
    }
    if (maskedAction == MotionEvent.ACTION_UP) {
      return PointerChange.UP;
    }
    // Secondary pointer:
    if (maskedAction == MotionEvent.ACTION_POINTER_DOWN) {
      return PointerChange.DOWN;
    }
    if (maskedAction == MotionEvent.ACTION_POINTER_UP) {
      return PointerChange.UP;
    }
    // All pointers:
    if (maskedAction == MotionEvent.ACTION_MOVE) {
      return PointerChange.MOVE;
    }
    if (maskedAction == MotionEvent.ACTION_HOVER_MOVE) {
      return PointerChange.HOVER;
    }
    if (maskedAction == MotionEvent.ACTION_CANCEL) {
      return PointerChange.CANCEL;
    }
    if (maskedAction == MotionEvent.ACTION_SCROLL) {
      return PointerChange.HOVER;
    }
    return -1;
  }

  @PointerChange
  private int getPointerChangeForPanZoom(int pointerChange) {
    if (pointerChange == PointerChange.DOWN) {
      return PointerChange.PAN_ZOOM_START;
    } else if (pointerChange == PointerChange.MOVE) {
      return PointerChange.PAN_ZOOM_UPDATE;
    } else if (pointerChange == PointerChange.UP || pointerChange == PointerChange.CANCEL) {
      return PointerChange.PAN_ZOOM_END;
    }
    return -1;
  }

  @PointerDeviceKind
  private int getPointerDeviceTypeForToolType(int toolType) {
    switch (toolType) {
      case MotionEvent.TOOL_TYPE_FINGER:
        return PointerDeviceKind.TOUCH;
      case MotionEvent.TOOL_TYPE_STYLUS:
        return PointerDeviceKind.STYLUS;
      case MotionEvent.TOOL_TYPE_MOUSE:
        return PointerDeviceKind.MOUSE;
      case MotionEvent.TOOL_TYPE_ERASER:
        return PointerDeviceKind.INVERTED_STYLUS;
      default:
        // MotionEvent.TOOL_TYPE_UNKNOWN will reach here.
        return PointerDeviceKind.UNKNOWN;
    }
  }
}
