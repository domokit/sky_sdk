package io.flutter.plugin.common;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import android.text.SpannableString;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class StandardMessageCodecTest {
  // Data types defined as per StandardMessageCodec.Java
  // XXX Please consider exposing these so that tests can access them
  private static final byte NULL = 0;
  private static final byte TRUE = 1;
  private static final byte FALSE = 2;
  private static final byte INT = 3;
  private static final byte LONG = 4;
  private static final byte BIGINT = 5;
  private static final byte DOUBLE = 6;
  private static final byte STRING = 7;
  private static final byte BYTE_ARRAY = 8;
  private static final byte INT_ARRAY = 9;
  private static final byte LONG_ARRAY = 10;
  private static final byte DOUBLE_ARRAY = 11;
  private static final byte LIST = 12;
  private static final byte MAP = 13;
  private static final byte FLOAT_ARRAY = 14;

  @Test
  public void itEncodesNullLiterals() {
    // Setup message codec
    StandardMessageCodec codec = new StandardMessageCodec();

    // Attempt to encode message with a null literal inside a list
    // A list with a null is used instead of just a null literal because if
    // only null is encoded, then no message is returned; null is returned instead
    ArrayList<Object> messageContent = new ArrayList();
    messageContent.add(null);
    ByteBuffer message = codec.encodeMessage(messageContent);
    message.flip();
    ByteBuffer expected = ByteBuffer.allocateDirect(3);
    expected.put(new byte[] {LIST, 1, NULL});
    expected.flip();
    assertEquals(expected, message);
  }

  @Test
  public void itEncodesNullObjects() {
    // An example class that equals null
    class ExampleNullObject {
      @Override
      public boolean equals(Object other) {
        return other == null || other == this;
      }

      @Override
      public int hashCode() {
        return 0;
      }
    }

    // Setup message codec
    StandardMessageCodec codec = new StandardMessageCodec();

    // Same as itEncodesNullLiterals but with objects that equal null instead
    ArrayList<Object> messageContent = new ArrayList();
    messageContent.add(new ExampleNullObject());
    ByteBuffer message = codec.encodeMessage(messageContent);
    message.flip();
    ByteBuffer expected = ByteBuffer.allocateDirect(3);
    expected.put(new byte[] {LIST, 1, NULL});
    expected.flip();
    assertEquals(expected, message);
  }

  @Test
  @SuppressWarnings("deprecation")
  public void itEncodesBooleans() {
    // Setup message codec
    StandardMessageCodec codec = new StandardMessageCodec();

    ArrayList<Object> messageContent = new ArrayList();
    // Test handling of Boolean objects other than the static TRUE and FALSE constants.
    messageContent.add(new Boolean(true));
    messageContent.add(new Boolean(false));
    ByteBuffer message = codec.encodeMessage(messageContent);
    message.flip();
    ByteBuffer expected = ByteBuffer.allocateDirect(4);
    expected.put(new byte[] {LIST, 2, TRUE, FALSE});
    expected.flip();
    assertEquals(expected, message);
  }

  @Test
  public void itEncodesFloatArrays() {
    StandardMessageCodec codec = new StandardMessageCodec();

    float[] expectedValues = new float[] {1.0f, 2.2f, 5.3f};

    ByteBuffer message = codec.encodeMessage(expectedValues);
    message.flip();

    float[] values = (float[]) codec.decodeMessage(message);
    assertArrayEquals(expectedValues, values, 0.01f);
  }

  @Test
  public void itEncodesCharSequences() {
    StandardMessageCodec codec = new StandardMessageCodec();

    CharSequence cs = new SpannableString("hello world");

    ByteBuffer message = codec.encodeMessage(cs);
    message.flip();

    String value = (String) codec.decodeMessage(message);
    assertEquals(value, "hello world");
  }

  private static class NotEncodable {
    @Override
    public String toString() {
      return "not encodable";
    }
  }

  @Test
  public void errorHasType() {
    StandardMessageCodec codec = new StandardMessageCodec();
    NotEncodable notEncodable = new NotEncodable();
    Exception exception =
        assertThrows(
            IllegalArgumentException.class,
            () -> {
              codec.encodeMessage(notEncodable);
            });
    assertTrue(exception.getMessage().contains("NotEncodable"));
  }
}
