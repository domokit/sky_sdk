package io.flutter.plugin.common;

import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.argThat;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import android.content.res.AssetManager;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartExecutor;
import java.nio.ByteBuffer;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentMatcher;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class MethodChannelTest {
  @Test
  public void resizeChannelBufferMessageIsWellformed() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = new DartExecutor(mockFlutterJNI, mock(AssetManager.class));
    String channel = "flutter/test";
    MethodChannel rawChannel = new MethodChannel(dartExecutor, channel);

    int newSize = 3;
    rawChannel.resizeChannelBuffer(newSize);

    // Created from the following Dart code:
    //   MethodCall methodCall = const MethodCall('resize', ['flutter/test', 3]);
    //   const StandardMethodCodec().encodeMethodCall(methodCall).buffer.asUint8List();
    final byte[] expected = {
      7, 6, 114, 101, 115, 105, 122, 101, 12, 2, 7, 12, 102, 108, 117, 116, 116, 101, 114, 47, 116,
      101, 115, 116, 3, 3, 0, 0, 0
    };

    // Verify that the correct message was sent to FlutterJNI.
    ArgumentMatcher<ByteBuffer> packetMatcher =
        new ByteBufferContentMatcher(ByteBuffer.wrap(expected));
    verify(mockFlutterJNI, times(1))
        .dispatchPlatformMessage(
            eq(BasicMessageChannel.CHANNEL_BUFFERS_CHANNEL),
            argThat(packetMatcher),
            anyInt(),
            anyInt());
  }

  @Test
  public void overflowChannelBufferMessageIsWellformed() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = new DartExecutor(mockFlutterJNI, mock(AssetManager.class));
    String channel = "flutter/test";
    MethodChannel rawChannel = new MethodChannel(dartExecutor, channel);

    rawChannel.setWarnsOnChannelOverflow(false);

    // Created from the following Dart code:
    //   MethodCall methodCall = const MethodCall('overflow', ['flutter/test', true]);
    //   const StandardMethodCodec().encodeMethodCall(methodCall).buffer.asUint8List();
    final byte[] expected = {
      7, 8, 111, 118, 101, 114, 102, 108, 111, 119, 12, 2, 7, 12, 102, 108, 117, 116, 116, 101, 114,
      47, 116, 101, 115, 116, 1
    };

    // Verify that the correct message was sent to FlutterJNI.
    ArgumentMatcher<ByteBuffer> packetMatcher =
        new ByteBufferContentMatcher(ByteBuffer.wrap(expected));
    verify(mockFlutterJNI, times(1))
        .dispatchPlatformMessage(
            eq(BasicMessageChannel.CHANNEL_BUFFERS_CHANNEL),
            argThat(packetMatcher),
            anyInt(),
            anyInt());
  }
}

// Custom ByteBuffer matcher which calls rewind on both buffers before calling equals.
// ByteBuffer.equals might return true when comparing byte buffers with different content if
// both have no remaining elements.
class ByteBufferContentMatcher implements ArgumentMatcher<ByteBuffer> {
  private ByteBuffer expected;

  public ByteBufferContentMatcher(ByteBuffer expected) {
    this.expected = expected;
  }

  @Override
  public boolean matches(ByteBuffer received) {
    expected.rewind();
    received.rewind();
    return received.equals(expected);
  }
}
