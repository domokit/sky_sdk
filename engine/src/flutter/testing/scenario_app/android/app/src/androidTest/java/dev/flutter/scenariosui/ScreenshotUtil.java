// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenariosui;

import androidx.annotation.NonNull;
import dev.flutter.scenarios.TestableFlutterActivity;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * Allows to capture screenshots, and transfers the screenshots to the host where they can be
 * further proccessed. On a LUCI environment, the screenshots are sent to Skia Gold.
 */
public class ScreenshotUtil {
  private static final int PORT = 3000;

  private static Connection conn;
  private static Executor executor;

  private static class Connection {
    final Socket clientSocket;
    final OutputStream out;
    final InputStream in;

    Connection(Socket socket) throws IOException {
      clientSocket = socket;
      out = socket.getOutputStream();
      in = socket.getInputStream();
    }

    synchronized void writeFile(String name) throws IOException {
      final ByteBuffer buffer = ByteBuffer.allocate(name.length() + 12);
      // See ScreenshotBlobTransformer#bind in screenshot_transformer.dart for consumer side.
      buffer.putInt(name.length());
      buffer.putInt(0);
      buffer.putInt(0);
      buffer.put(name.getBytes());
      final byte[] bytes = buffer.array();
      out.write(bytes, 0, bytes.length);
      out.flush();

      // Wait on run_android_tests.dart to write a single byte into the socket
      // as a signal that adb screencapture has completed.
      in.read();
    }

    synchronized void close() throws IOException {
      clientSocket.close();
    }
  }

  /** Starts the connection with the host. */
  public static synchronized void onCreate() {
    if (executor == null) {
      executor = Executors.newSingleThreadExecutor();
    }
    if (conn == null) {
      executor.execute(
          () -> {
            try {
              final Socket socket = new Socket(InetAddress.getLoopbackAddress(), PORT);
              conn = new Connection(socket);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          });
    }
  }

  /** Closes the connection with the host. */
  public static synchronized void finish() {
    if (executor != null && conn != null) {
      executor.execute(
          () -> {
            try {
              conn.close();
              conn = null;
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          });
    }
  }

  /**
   * Sends the file to the host.
   *
   * @param filename The file name.
   * @param fileContent The file content.
   */
  public static synchronized void writeFile(
      @NonNull String filename, @NonNull CountDownLatch latch) {
    if (executor != null && conn != null) {
      executor.execute(
          () -> {
            try {
              conn.writeFile(filename);
            } catch (IOException e) {
              throw new RuntimeException(e);
            } finally {
              latch.countDown();
            }
          });
    }
  }

  /**
   * Captures a screenshot of the activity, and sends the screenshot bytes to the host where it is
   * further processed.
   *
   * <p>The activity must be already launched.
   *
   * @param activity The target activity.
   * @param fileName The name of the file.
   */
  public static void capture(@NonNull TestableFlutterActivity activity, @NonNull String captureName)
      throws Exception {
    CountDownLatch latch = new CountDownLatch(1);
    activity.waitUntilFlutterRendered();
    ScreenshotUtil.writeFile(captureName, latch);
    latch.await();
  }
}
