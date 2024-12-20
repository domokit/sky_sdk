// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static io.flutter.Build.API_LEVELS;
import static junit.framework.TestCase.assertEquals;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.TargetApi;
import android.content.Context;
import android.view.InputDevice;
import android.view.MotionEvent;
import android.view.ViewConfiguration;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.mockito.InOrder;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_28)
public class AndroidTouchProcessorTest {
  @Mock FlutterRenderer mockRenderer;
  AndroidTouchProcessor touchProcessor;
  @Captor ArgumentCaptor<ByteBuffer> packetCaptor;
  @Captor ArgumentCaptor<Integer> packetSizeCaptor;
  // Used for mock events in SystemClock.uptimeMillis() time base.
  // 2 days in milliseconds
  final long eventTimeMilliseconds = 172800000;
  final float pressure = 0.8f;
  // https://github.com/flutter/flutter/blob/master/packages/flutter/lib/src/gestures/events.dart
  final int enginePrimaryStylusButton = 0x02;

  @Before
  public void setUp() {
    MockitoAnnotations.openMocks(this);
    touchProcessor = new AndroidTouchProcessor(mockRenderer, false);
  }

  private long readTimeStamp(ByteBuffer buffer) {
    return buffer.getLong(1 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private long readPointerChange(ByteBuffer buffer) {
    return readPointerChangeForPointer(buffer, 0);
  }

  private long readPointerChangeForPointer(ByteBuffer buffer, int pointerIndex) {
    return buffer.getLong(
        2 * AndroidTouchProcessor.BYTES_PER_FIELD
            + (AndroidTouchProcessor.BYTES_PER_FIELD
                * AndroidTouchProcessor.POINTER_DATA_FIELD_COUNT
                * pointerIndex));
  }

  private long readPointerDeviceKind(ByteBuffer buffer) {
    return readPointerDeviceKindForPointer(buffer, 0);
  }

  private long readPointerDeviceKindForPointer(ByteBuffer buffer, int pointerIndex) {
    return buffer.getLong(
        3 * AndroidTouchProcessor.BYTES_PER_FIELD
            + (AndroidTouchProcessor.BYTES_PER_FIELD
                * AndroidTouchProcessor.POINTER_DATA_FIELD_COUNT
                * pointerIndex));
  }

  private long readPointerSignalKind(ByteBuffer buffer) {
    return readPointerSignalKindForPointer(buffer, 0);
  }

  private long readPointerSignalKindForPointer(ByteBuffer buffer, int pointerIndex) {
    return buffer.getLong(
        4 * AndroidTouchProcessor.BYTES_PER_FIELD
            + (AndroidTouchProcessor.BYTES_PER_FIELD
                * AndroidTouchProcessor.POINTER_DATA_FIELD_COUNT
                * pointerIndex));
  }

  private long readDevice(ByteBuffer buffer) {
    return buffer.getLong(5 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPointerPhysicalX(ByteBuffer buffer) {
    return readPointerPhysicalXForPointer(buffer, 0);
  }

  private double readPointerPhysicalXForPointer(ByteBuffer buffer, int pointerIndex) {
    return buffer.getDouble(
        7 * AndroidTouchProcessor.BYTES_PER_FIELD
            + (AndroidTouchProcessor.BYTES_PER_FIELD
                * AndroidTouchProcessor.POINTER_DATA_FIELD_COUNT
                * pointerIndex));
  }

  private double readPointerPhysicalY(ByteBuffer buffer) {
    return readPointerPhysicalYForPointer(buffer, 0);
  }

  private double readPointerPhysicalYForPointer(ByteBuffer buffer, int pointerIndex) {
    return buffer.getDouble(
        8 * AndroidTouchProcessor.BYTES_PER_FIELD
            + (AndroidTouchProcessor.BYTES_PER_FIELD
                * AndroidTouchProcessor.POINTER_DATA_FIELD_COUNT
                * pointerIndex));
  }

  private long readButtons(ByteBuffer buffer) {
    return buffer.getLong(11 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readObscured(ByteBuffer buffer) {
    return buffer.getDouble(12 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readSynthesized(ByteBuffer buffer) {
    return buffer.getDouble(13 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPressure(ByteBuffer buffer) {
    return buffer.getDouble(14 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPressureMin(ByteBuffer buffer) {
    return buffer.getDouble(15 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPressureMax(ByteBuffer buffer) {
    return buffer.getDouble(16 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readDistance(ByteBuffer buffer) {
    return buffer.getDouble(17 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readDistanceMax(ByteBuffer buffer) {
    return buffer.getDouble(18 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readSize(ByteBuffer buffer) {
    return buffer.getDouble(19 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readRadiusMajor(ByteBuffer buffer) {
    return buffer.getDouble(20 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readRadiusMinor(ByteBuffer buffer) {
    return buffer.getDouble(21 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readRadiusMin(ByteBuffer buffer) {
    return buffer.getDouble(22 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readRadiusMax(ByteBuffer buffer) {
    return buffer.getDouble(23 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readStylusTilt(ByteBuffer buffer) {
    return buffer.getDouble(25 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readScrollDeltaX(ByteBuffer buffer) {
    return buffer.getDouble(27 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readScrollDeltaY(ByteBuffer buffer) {
    return buffer.getDouble(28 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPointerPanX(ByteBuffer buffer) {
    return buffer.getDouble(29 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPointerPanY(ByteBuffer buffer) {
    return buffer.getDouble(30 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPointerPanDeltaX(ByteBuffer buffer) {
    return buffer.getDouble(31 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readPointerPanDeltaY(ByteBuffer buffer) {
    return buffer.getDouble(32 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readScale(ByteBuffer buffer) {
    return buffer.getDouble(33 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private double readRotation(ByteBuffer buffer) {
    return buffer.getDouble(34 * AndroidTouchProcessor.BYTES_PER_FIELD);
  }

  private class MotionEventMocker {
    int pointerId;
    int source;
    int toolType;

    MotionEventMocker(int pointerId, int source, int toolType) {
      this.pointerId = pointerId;
      this.source = source;
      this.toolType = toolType;
    }

    MotionEvent mockEvent(int action, float x, float y, int buttonState) {
      return mockEvent(action, x, y, buttonState, x, y, x, y, x, x, y);
    }

    MotionEvent mockEvent(
        int action,
        float x,
        float y,
        int buttonState,
        float hScroll,
        float vScroll,
        float axisDistance,
        float axisTilt,
        float size,
        float toolMajor,
        float toolMinor) {
      MotionEvent event = mock(MotionEvent.class);
      when(event.getDevice()).thenReturn(null);
      when(event.getSource()).thenReturn(source);
      when(event.getEventTime()).thenReturn(eventTimeMilliseconds);
      when(event.getPointerCount()).thenReturn(1);
      when(event.getActionMasked()).thenReturn(action);
      final int actionIndex = 0;
      when(event.getActionIndex()).thenReturn(actionIndex);
      when(event.getButtonState()).thenReturn(buttonState);
      when(event.getPointerId(actionIndex)).thenReturn(pointerId);
      when(event.getX(actionIndex)).thenReturn(x);
      when(event.getY(actionIndex)).thenReturn(y);
      when(event.getToolType(actionIndex)).thenReturn(toolType);
      when(event.isFromSource(InputDevice.SOURCE_CLASS_POINTER)).thenReturn(true);
      when(event.getAxisValue(MotionEvent.AXIS_HSCROLL, pointerId)).thenReturn(hScroll);
      when(event.getAxisValue(MotionEvent.AXIS_VSCROLL, pointerId)).thenReturn(vScroll);
      when(event.getAxisValue(MotionEvent.AXIS_DISTANCE, pointerId)).thenReturn(axisDistance);
      when(event.getAxisValue(MotionEvent.AXIS_TILT, pointerId)).thenReturn(axisTilt);
      when(event.getPressure(actionIndex)).thenReturn(pressure);
      when(event.getSize(actionIndex)).thenReturn(size);
      when(event.getToolMajor(actionIndex)).thenReturn(toolMajor);
      when(event.getToolMinor(actionIndex)).thenReturn(toolMinor);
      return event;
    }
  }

  @Test
  public void normalTouch() {
    MotionEventMocker mocker =
        new MotionEventMocker(0, InputDevice.SOURCE_TOUCHSCREEN, MotionEvent.TOOL_TYPE_FINGER);
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_DOWN, 0.0f, 0.0f, 0));
    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.DOWN, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TOUCH, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_MOVE, 10.0f, 5.0f, 0));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.MOVE, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TOUCH, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(10.0, readPointerPhysicalX(packet));
    assertEquals(5.0, readPointerPhysicalY(packet));
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_UP, 10.0f, 5.0f, 0));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.UP, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TOUCH, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(10.0, readPointerPhysicalX(packet));
    assertEquals(5.0, readPointerPhysicalY(packet));
    assertEquals(
        AndroidTouchProcessor.PointerChange.REMOVE, readPointerChangeForPointer(packet, 1));
    assertEquals(
        AndroidTouchProcessor.PointerDeviceKind.TOUCH, readPointerDeviceKindForPointer(packet, 1));
    assertEquals(
        AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKindForPointer(packet, 1));
    assertEquals(10.0, readPointerPhysicalXForPointer(packet, 1));
    assertEquals(5.0, readPointerPhysicalYForPointer(packet, 1));
    // Expect two pointer changes - the original UP and the synthesized REMOVE.
    assertEquals(
        AndroidTouchProcessor.BYTES_PER_FIELD * AndroidTouchProcessor.POINTER_DATA_FIELD_COUNT * 2,
        packet.capacity());
    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void trackpadGesture() {
    MotionEventMocker mocker =
        new MotionEventMocker(1, InputDevice.SOURCE_MOUSE, MotionEvent.TOOL_TYPE_MOUSE);
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_DOWN, 0.0f, 0.0f, 0));
    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.PAN_ZOOM_START, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TRACKPAD, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_MOVE, 10.0f, 5.0f, 0));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.PAN_ZOOM_UPDATE, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TRACKPAD, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));
    assertEquals(10.0, readPointerPanX(packet));
    assertEquals(5.0, readPointerPanY(packet));
    // Always zero.
    assertEquals(0.0, readPointerPanDeltaX(packet));
    assertEquals(0.0, readPointerPanDeltaY(packet));
    assertEquals(0.0, readRotation(packet));
    // Always 1.
    assertEquals(1.0, readScale(packet));
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_UP, 10.0f, 5.0f, 0));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.PAN_ZOOM_END, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TRACKPAD, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));
    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void mouse() {
    MotionEventMocker mocker =
        new MotionEventMocker(2, InputDevice.SOURCE_MOUSE, MotionEvent.TOOL_TYPE_MOUSE);
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_DOWN, 0.0f, 0.0f, 1));
    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.DOWN, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.MOUSE, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_MOVE, 10.0f, 5.0f, 1));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.MOVE, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.MOUSE, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(10.0, readPointerPhysicalX(packet));
    assertEquals(5.0, readPointerPhysicalY(packet));
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_UP, 10.0f, 5.0f, 1));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.UP, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.MOUSE, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(10.0, readPointerPhysicalX(packet));
    assertEquals(5.0, readPointerPhysicalY(packet));
    // Ensure that we don't send a synthesized remove when using a mouse.
    assertEquals(
        AndroidTouchProcessor.BYTES_PER_FIELD * AndroidTouchProcessor.POINTER_DATA_FIELD_COUNT,
        packet.capacity());
    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void unexpectedMaskedAction() {
    // Regression test for https://github.com/flutter/flutter/issues/111068
    MotionEventMocker mocker =
        new MotionEventMocker(1, InputDevice.SOURCE_STYLUS, MotionEvent.TOOL_TYPE_STYLUS);
    // ACTION_BUTTON_PRESS is not handled by AndroidTouchProcessor, nothing should be dispatched.
    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_BUTTON_PRESS, 0.0f, 0.0f, 0));
    verify(mockRenderer, never()).dispatchPointerDataPacket(ByteBuffer.allocate(0), 0);
  }

  @Test
  @Config(minSdk = API_LEVELS.API_26)
  public void scrollWheelAbove26() {
    // Pointer id must be zero to match actionIndex in mocked event.
    final int pointerId = 0;
    MotionEventMocker mocker =
        new MotionEventMocker(
            pointerId, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);
    final float horizontalScrollValue = -1f;
    final float verticalScrollValue = .5f;
    final Context context = ApplicationProvider.getApplicationContext();
    final double horizontalScaleFactor =
        ViewConfiguration.get(context).getScaledHorizontalScrollFactor();
    final double verticalScaleFactor =
        ViewConfiguration.get(context).getScaledVerticalScrollFactor();
    // Zero verticalScaleFactor will cause this test to miss bugs.
    assertEquals("zero horizontal scale factor", true, horizontalScaleFactor != 0);
    assertEquals("zero vertical scale factor", true, verticalScaleFactor != 0);

    final MotionEvent event =
        mocker.mockEvent(
            MotionEvent.ACTION_SCROLL,
            0.0f,
            0.0f,
            1,
            horizontalScrollValue,
            verticalScrollValue,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f);
    boolean handled = touchProcessor.onGenericMotionEvent(event, context);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    assertEquals(-horizontalScrollValue * horizontalScaleFactor, readScrollDeltaX(packet));
    assertEquals(-verticalScrollValue * verticalScaleFactor, readScrollDeltaY(packet));
    verify(event).getAxisValue(MotionEvent.AXIS_HSCROLL, pointerId);
    verify(event).getAxisValue(MotionEvent.AXIS_VSCROLL, pointerId);

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  @Config(sdk = API_LEVELS.API_25)
  public void scrollWheelBelow26() {
    // Pointer id must be zero to match actionIndex in mocked event.
    final int pointerId = 0;
    MotionEventMocker mocker =
        new MotionEventMocker(
            pointerId, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);
    final float horizontalScrollValue = -1f;
    final float verticalScrollValue = .5f;
    final Context context = ApplicationProvider.getApplicationContext();

    final MotionEvent event =
        mocker.mockEvent(
            MotionEvent.ACTION_SCROLL,
            0.0f,
            0.0f,
            1,
            horizontalScrollValue,
            verticalScrollValue,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f);
    boolean handled = touchProcessor.onGenericMotionEvent(event, context);
    assertEquals(true, handled);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    // Magic number from roboletric's theme.
    final double magicScrollFactor = 64;
    assertEquals(-horizontalScrollValue * magicScrollFactor, readScrollDeltaX(packet));
    assertEquals(-verticalScrollValue * magicScrollFactor, readScrollDeltaY(packet));
    verify(event).getAxisValue(MotionEvent.AXIS_HSCROLL, pointerId);
    verify(event).getAxisValue(MotionEvent.AXIS_VSCROLL, pointerId);

    // Trigger default values.
    touchProcessor.onGenericMotionEvent(event, null);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();

    assertEquals(
        (double) -horizontalScrollValue * AndroidTouchProcessor.DEFAULT_HORIZONTAL_SCROLL_FACTOR,
        readScrollDeltaX(packet));
    assertEquals(
        (double) -verticalScrollValue * AndroidTouchProcessor.DEFAULT_VERTICAL_SCROLL_FACTOR,
        readScrollDeltaY(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void timeStamp() {
    final int pointerId = 0;
    MotionEventMocker mocker =
        new MotionEventMocker(
            pointerId, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);

    final MotionEvent event = mocker.mockEvent(MotionEvent.ACTION_SCROLL, 1f, 1f, 1);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    assertEquals(TimeUnit.MILLISECONDS.toMicros(eventTimeMilliseconds), readTimeStamp(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void device() {
    final int pointerId = 2;
    MotionEventMocker mocker =
        new MotionEventMocker(
            pointerId, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);

    final MotionEvent event = mocker.mockEvent(MotionEvent.ACTION_SCROLL, 1f, 1f, 1);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    assertEquals(pointerId, readDevice(packet));
    verify(event).getPointerId(0);

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void physicalXPhysicalY() {
    MotionEventMocker mocker =
        new MotionEventMocker(1, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);
    final float x = 10.0f;
    final float y = 20.0f;
    final MotionEvent event = mocker.mockEvent(MotionEvent.ACTION_DOWN, x, y, 0);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    assertEquals((double) x, readPointerPhysicalX(packet));
    assertEquals((double) y, readPointerPhysicalY(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void obscured() {
    MotionEventMocker mocker =
        new MotionEventMocker(1, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);
    final MotionEvent event = mocker.mockEvent(MotionEvent.ACTION_DOWN, 10.0f, 20.0f, 0);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    // Always zero.
    assertEquals(0.0, readObscured(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void synthesized() {
    MotionEventMocker mocker =
        new MotionEventMocker(1, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);
    final MotionEvent event = mocker.mockEvent(MotionEvent.ACTION_DOWN, 10.0f, 20.0f, 0);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    // Always zero.
    assertEquals(0.0, readSynthesized(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void pressure() {
    MotionEventMocker mocker =
        new MotionEventMocker(1, InputDevice.SOURCE_CLASS_POINTER, MotionEvent.TOOL_TYPE_MOUSE);
    final MotionEvent event = mocker.mockEvent(MotionEvent.ACTION_DOWN, 10.0f, 20.0f, 0);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    // Always zero.
    assertEquals((double) pressure, readPressure(packet));
    // Verify default range with null device.
    assertEquals(0.0, readPressureMin(packet));
    assertEquals(1.0, readPressureMax(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void stylusDistance() {
    MotionEventMocker mocker =
        new MotionEventMocker(0, InputDevice.SOURCE_STYLUS, MotionEvent.TOOL_TYPE_STYLUS);
    final float distance = 10.0f;
    final float tilt = 20.0f;
    final MotionEvent event =
        mocker.mockEvent(
            MotionEvent.ACTION_DOWN,
            0.0f,
            0.0f,
            MotionEvent.BUTTON_STYLUS_PRIMARY,
            0.0f,
            0.0f,
            distance,
            tilt,
            0.0f,
            0.0f,
            0.0f);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.STYLUS, readPointerDeviceKind(packet));
    assertEquals((double) distance, readDistance(packet));
    // Always zero.
    assertEquals(0.0, readDistanceMax(packet));
    assertEquals((double) tilt, readStylusTilt(packet));
    assertEquals(enginePrimaryStylusButton, readButtons(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void sizeAndRadius() {
    MotionEventMocker mocker =
        new MotionEventMocker(0, InputDevice.SOURCE_STYLUS, MotionEvent.TOOL_TYPE_STYLUS);
    final float size = 10.0f;
    final float radiusMajor = 20.0f;
    final float radiusMinor = 30.0f;
    final MotionEvent event =
        mocker.mockEvent(
            MotionEvent.ACTION_DOWN,
            0.0f,
            0.0f,
            0,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            size,
            radiusMajor,
            radiusMinor);
    boolean handled = touchProcessor.onTouchEvent(event);

    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();

    verify(event).getSize(0);
    verify(event).getToolMajor(0);
    verify(event).getToolMinor(0);

    assertEquals((double) size, readSize(packet));
    assertEquals((double) radiusMajor, readRadiusMajor(packet));
    assertEquals((double) radiusMinor, readRadiusMinor(packet));
    // Always zero.
    assertEquals(0.0, readRadiusMin(packet));
    assertEquals(0.0, readRadiusMax(packet));

    inOrder.verifyNoMoreInteractions();
  }

  @Test
  public void unexpectedPointerChange() {
    // Regression test for https://github.com/flutter/flutter/issues/129765
    MotionEventMocker mocker =
        new MotionEventMocker(0, InputDevice.SOURCE_MOUSE, MotionEvent.TOOL_TYPE_MOUSE);

    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_DOWN, 0.0f, 0.0f, 0));
    InOrder inOrder = inOrder(mockRenderer);
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    ByteBuffer packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.PAN_ZOOM_START, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TRACKPAD, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));

    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_MOVE, 10.0f, 5.0f, 0));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.PAN_ZOOM_UPDATE, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TRACKPAD, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));
    assertEquals(10.0, readPointerPanX(packet));
    assertEquals(5.0, readPointerPanY(packet));

    touchProcessor.onGenericMotionEvent(
        mocker.mockEvent(MotionEvent.ACTION_SCROLL, 0.0f, 0.0f, 0),
        ApplicationProvider.getApplicationContext());
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    packet.rewind();
    while (packet.hasRemaining()) {
      assertEquals(0, packet.get());
    }

    touchProcessor.onTouchEvent(mocker.mockEvent(MotionEvent.ACTION_UP, 10.0f, 5.0f, 0));
    inOrder
        .verify(mockRenderer)
        .dispatchPointerDataPacket(packetCaptor.capture(), packetSizeCaptor.capture());
    packet = packetCaptor.getValue();
    assertEquals(AndroidTouchProcessor.PointerChange.PAN_ZOOM_END, readPointerChange(packet));
    assertEquals(AndroidTouchProcessor.PointerDeviceKind.TRACKPAD, readPointerDeviceKind(packet));
    assertEquals(AndroidTouchProcessor.PointerSignalKind.NONE, readPointerSignalKind(packet));
    assertEquals(0.0, readPointerPhysicalX(packet));
    assertEquals(0.0, readPointerPhysicalY(packet));
    inOrder.verifyNoMoreInteractions();
  }
}
