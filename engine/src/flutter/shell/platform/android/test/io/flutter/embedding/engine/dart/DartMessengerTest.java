// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.dart;

import static android.os.Looper.getMainLooper;
import static junit.framework.TestCase.assertEquals;
import static junit.framework.TestCase.assertNotNull;
import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertArrayEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.robolectric.Shadows.shadowOf;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartMessenger.DartMessengerTaskQueue;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.BinaryMessenger.BinaryMessageHandler;
import java.nio.ByteBuffer;
import java.util.LinkedList;
import java.util.Random;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class DartMessengerTest {
  SynchronousTaskQueue synchronousTaskQueue = new SynchronousTaskQueue();

  private static class ReportingUncaughtExceptionHandler
      implements Thread.UncaughtExceptionHandler {
    public Throwable latestException;

    @Override
    public void uncaughtException(Thread t, Throwable e) {
      latestException = e;
    }
  }

  private static class SynchronousTaskQueue implements DartMessengerTaskQueue {
    public void dispatch(Runnable runnable) {
      runnable.run();
    }
  }

  @Test
  public void itHandlesErrors() {
    // Setup test.
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final Thread currentThread = Thread.currentThread();
    final Thread.UncaughtExceptionHandler savedHandler =
        currentThread.getUncaughtExceptionHandler();
    final ReportingUncaughtExceptionHandler reportingHandler =
        new ReportingUncaughtExceptionHandler();
    currentThread.setUncaughtExceptionHandler(reportingHandler);

    // Create object under test.
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    final BinaryMessageHandler throwingHandler = mock(BinaryMessageHandler.class);
    Mockito.doThrow(AssertionError.class)
        .when(throwingHandler)
        .onMessage(any(ByteBuffer.class), any(DartMessenger.Reply.class));
    BinaryMessenger.TaskQueue taskQueue = messenger.makeBackgroundTaskQueue();
    messenger.setMessageHandler("test", throwingHandler, taskQueue);
    messenger.handleMessageFromDart("test", ByteBuffer.allocate(0), 0, 0);
    assertNotNull(reportingHandler.latestException);
    assertTrue(reportingHandler.latestException instanceof AssertionError);
    currentThread.setUncaughtExceptionHandler(savedHandler);
  }

  @Test
  public void givesDirectByteBuffer() {
    // Setup test.
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    final String channel = "foobar";
    final boolean[] wasDirect = {false};
    final BinaryMessenger.BinaryMessageHandler handler =
        (message, reply) -> {
          wasDirect[0] = message.isDirect();
        };
    BinaryMessenger.TaskQueue taskQueue = messenger.makeBackgroundTaskQueue();
    messenger.setMessageHandler(channel, handler, taskQueue);
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    message.rewind();
    message.putChar('a');
    message.putChar('b');
    message.putChar('c');
    message.putChar('d');
    messenger.handleMessageFromDart(channel, message, /*replyId=*/ 123, 0);
    assertTrue(wasDirect[0]);
  }

  @Test
  public void directByteBufferLimitZeroAfterUsage() {
    // Setup test.
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    final String channel = "foobar";
    final ByteBuffer[] byteBuffers = {null};
    final int bufferSize = 4 * 2;
    final BinaryMessenger.BinaryMessageHandler handler =
        (message, reply) -> {
          byteBuffers[0] = message;
          assertEquals(bufferSize, byteBuffers[0].limit());
        };
    BinaryMessenger.TaskQueue taskQueue = messenger.makeBackgroundTaskQueue();
    messenger.setMessageHandler(channel, handler, taskQueue);
    final ByteBuffer message = ByteBuffer.allocateDirect(bufferSize);
    message.rewind();
    message.putChar('a');
    message.putChar('b');
    message.putChar('c');
    message.putChar('d');
    messenger.handleMessageFromDart(channel, message, /*replyId=*/ 123, 0);
    assertNotNull(byteBuffers[0]);
    assertTrue(byteBuffers[0].isDirect());
    assertEquals(0, byteBuffers[0].limit());
  }

  @Test
  public void directByteBufferLimitZeroAfterReply() {
    // Setup test.
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger = new DartMessenger(fakeFlutterJni);
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    final String channel = "foobar";
    message.rewind();
    message.putChar('a');
    message.putChar('b');
    message.putChar('c');
    message.putChar('d');
    final ByteBuffer[] byteBuffers = {null};
    BinaryMessenger.BinaryReply callback =
        (reply) -> {
          assertTrue(reply.isDirect());
          byteBuffers[0] = reply;
        };
    messenger.send(channel, null, callback);
    messenger.handlePlatformMessageResponse(1, message);
    assertEquals(0, byteBuffers[0].limit());
  }

  @Test
  public void replyIdIncrementsOnNullReply() {
    /// Setup test.
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger = new DartMessenger(fakeFlutterJni);
    final String channel = "foobar";
    messenger.send(channel, null, null);
    verify(fakeFlutterJni, times(1)).dispatchEmptyPlatformMessage(eq("foobar"), eq(1));
    messenger.send(channel, null, null);
    verify(fakeFlutterJni, times(1)).dispatchEmptyPlatformMessage(eq("foobar"), eq(2));
  }

  @Test
  public void cleansUpMessageData() throws InterruptedException {
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    BinaryMessenger.TaskQueue taskQueue = messenger.makeBackgroundTaskQueue();
    String channel = "foobar";
    BinaryMessenger.BinaryMessageHandler handler =
        (ByteBuffer message, BinaryMessenger.BinaryReply reply) -> {
          reply.reply(null);
        };
    messenger.setMessageHandler(channel, handler, taskQueue);
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    final int replyId = 1;
    final long messageData = 1234;
    messenger.handleMessageFromDart(channel, message, replyId, messageData);
    verify(fakeFlutterJni).cleanupMessageData(eq(messageData));
  }

  @Test
  public void cleansUpMessageDataOnError() throws InterruptedException {
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    BinaryMessenger.TaskQueue taskQueue = messenger.makeBackgroundTaskQueue();
    String channel = "foobar";
    BinaryMessenger.BinaryMessageHandler handler =
        (ByteBuffer message, BinaryMessenger.BinaryReply reply) -> {
          throw new RuntimeException("hello");
        };
    messenger.setMessageHandler(channel, handler, taskQueue);
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    final int replyId = 1;
    final long messageData = 1234;

    messenger.handleMessageFromDart(channel, message, replyId, messageData);
    verify(fakeFlutterJni).cleanupMessageData(eq(messageData));
  }

  @Test
  public void emptyResponseWhenHandlerIsNotSet() throws InterruptedException {
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    final String channel = "foobar";
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    final int replyId = 1;
    final long messageData = 1234;

    messenger.handleMessageFromDart(channel, message, replyId, messageData);
    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni).invokePlatformMessageEmptyResponseCallback(replyId);
  }

  @Test
  public void buffersResponseWhenHandlerIsNotSet() throws InterruptedException {
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    final BinaryMessenger.TaskQueue taskQueue = messenger.makeBackgroundTaskQueue();
    final String channel = "foobar";
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    final int replyId = 1;
    final long messageData = 1234;

    messenger.enableBufferingIncomingMessages();
    messenger.handleMessageFromDart(channel, message, replyId, messageData);

    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni, never()).invokePlatformMessageEmptyResponseCallback(eq(replyId));

    final BinaryMessenger.BinaryMessageHandler handler =
        (ByteBuffer msg, BinaryMessenger.BinaryReply reply) -> {
          reply.reply(ByteBuffer.wrap("done".getBytes()));
        };
    messenger.setMessageHandler(channel, handler, taskQueue);

    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni, never()).invokePlatformMessageEmptyResponseCallback(eq(replyId));

    final ArgumentCaptor<ByteBuffer> response = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(fakeFlutterJni)
        .invokePlatformMessageResponseCallback(anyInt(), response.capture(), anyInt());
    assertArrayEquals("done".getBytes(), response.getValue().array());
  }

  @Test
  public void disableBufferingTriggersEmptyResponseForPendingMessages()
      throws InterruptedException {
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    final String channel = "foobar";
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    final int replyId = 1;
    final long messageData = 1234;

    messenger.enableBufferingIncomingMessages();
    messenger.handleMessageFromDart(channel, message, replyId, messageData);
    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni, never()).invokePlatformMessageEmptyResponseCallback(replyId);

    messenger.disableBufferingIncomingMessages();
    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni).invokePlatformMessageEmptyResponseCallback(replyId);
  }

  @Test
  public void emptyResponseWhenHandlerIsUnregistered() throws InterruptedException {
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger =
        new DartMessenger(fakeFlutterJni, (options) -> synchronousTaskQueue);
    final BinaryMessenger.TaskQueue taskQueue = messenger.makeBackgroundTaskQueue();
    final String channel = "foobar";
    final ByteBuffer message = ByteBuffer.allocateDirect(4 * 2);
    final int replyId = 1;
    final long messageData = 1234;

    messenger.enableBufferingIncomingMessages();
    messenger.handleMessageFromDart(channel, message, replyId, messageData);

    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni, never()).invokePlatformMessageEmptyResponseCallback(eq(replyId));

    final BinaryMessenger.BinaryMessageHandler handler =
        (ByteBuffer msg, BinaryMessenger.BinaryReply reply) -> {
          reply.reply(ByteBuffer.wrap("done".getBytes()));
        };
    messenger.setMessageHandler(channel, handler, taskQueue);

    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni, never()).invokePlatformMessageEmptyResponseCallback(eq(replyId));

    final ArgumentCaptor<ByteBuffer> response = ArgumentCaptor.forClass(ByteBuffer.class);
    verify(fakeFlutterJni)
        .invokePlatformMessageResponseCallback(anyInt(), response.capture(), anyInt());
    assertArrayEquals("done".getBytes(), response.getValue().array());

    messenger.disableBufferingIncomingMessages();
    messenger.setMessageHandler(channel, null, null); // Unregister handler.

    messenger.handleMessageFromDart(channel, message, replyId, messageData);
    shadowOf(getMainLooper()).idle();
    verify(fakeFlutterJni).invokePlatformMessageEmptyResponseCallback(replyId);
  }

  @Test
  public void testSerialTaskQueue() throws InterruptedException {
    final FlutterJNI fakeFlutterJni = mock(FlutterJNI.class);
    final DartMessenger messenger = new DartMessenger(fakeFlutterJni);
    final ExecutorService taskQueuePool = Executors.newFixedThreadPool(4);
    final DartMessengerTaskQueue taskQueue = new DartMessenger.SerialTaskQueue(taskQueuePool);
    final int count = 5000;
    final LinkedList<Integer> ints = new LinkedList<>();
    Random rand = new Random();
    for (int i = 0; i < count; ++i) {
      final int value = i;
      taskQueue.dispatch(
          () -> {
            try {
              Thread.sleep(rand.nextInt(10));
            } catch (InterruptedException ex) {
              System.out.println(ex.toString());
            }
            ints.add(value);
          });
      taskQueuePool.execute(
          () -> {
            // Add some extra noise to make sure we aren't always handling on the same thread.
            try {
              Thread.sleep(rand.nextInt(10));
            } catch (InterruptedException ex) {
              System.out.println(ex.toString());
            }
          });
    }
    CountDownLatch latch = new CountDownLatch(1);
    taskQueue.dispatch(
        () -> {
          latch.countDown();
        });
    latch.await();
    assertEquals(count, ints.size());
    for (int i = 0; i < count - 1; ++i) {
      assertEquals((int) ints.get(i), (int) (ints.get(i + 1)) - 1);
    }
  }
}
