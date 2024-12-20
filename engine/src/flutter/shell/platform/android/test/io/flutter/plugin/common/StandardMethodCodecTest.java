package io.flutter.plugin.common;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class StandardMethodCodecTest {

  @Test
  public void encodeMethodTest() {
    final Map<String, String> args = new HashMap<>();
    args.put("testArg", "testValue");
    MethodCall call = new MethodCall("testMethod", args);
    final ByteBuffer buffer = StandardMethodCodec.INSTANCE.encodeMethodCall(call);
    assertNotNull(buffer);
    buffer.flip();
    final MethodCall result = StandardMethodCodec.INSTANCE.decodeMethodCall(buffer);
    assertEquals(call.method, result.method);
    assertEquals(call.arguments, result.arguments);
  }

  @Test
  public void encodeSuccessEnvelopeTest() {
    final Map<String, Integer> success = new HashMap<>();
    success.put("result", 1);
    final ByteBuffer buffer = StandardMethodCodec.INSTANCE.encodeSuccessEnvelope(success);
    assertNotNull(buffer);
    buffer.flip();
    final Object result = StandardMethodCodec.INSTANCE.decodeEnvelope(buffer);
    assertEquals(success, result);
  }

  @Test
  public void encodeSuccessEnvelopeUnsupportedObjectTest() {
    final StandardMethodCodecTest joke = new StandardMethodCodecTest();
    try {
      final ByteBuffer buffer = StandardMethodCodec.INSTANCE.encodeSuccessEnvelope(joke);
      fail("Should have failed to convert unsupported type.");
    } catch (IllegalArgumentException e) {
      // pass.
    }
  }

  @Test
  public void encodeErrorEnvelopeWithNullDetailsTest() {
    final ByteBuffer buffer =
        StandardMethodCodec.INSTANCE.encodeErrorEnvelope("code", "error", null);
    assertNotNull(buffer);
    buffer.flip();
    try {
      StandardMethodCodec.INSTANCE.decodeEnvelope(buffer);
      fail("Should have thrown a FlutterException since this is an error envelope.");
    } catch (FlutterException result) {
      assertEquals("code", result.code);
      assertEquals("error", result.getMessage());
      assertNull(result.details);
    }
  }

  @Test
  public void encodeErrorEnvelopeWithThrowableTest() {
    final Exception e = new IllegalArgumentException("foo");
    final ByteBuffer buffer =
        StandardMethodCodec.INSTANCE.encodeErrorEnvelope("code", e.getMessage(), e);
    assertNotNull(buffer);
    buffer.flip();
    try {
      StandardMethodCodec.INSTANCE.decodeEnvelope(buffer);
      fail("Should have thrown a FlutterException since this is an error envelope.");
    } catch (FlutterException result) {
      assertEquals("code", result.code);
      assertEquals("foo", result.getMessage());
      // Must contain part of a stack.
      String stack = (String) result.details;
      assertTrue(
          stack.contains(
              "at io.flutter.plugin.common.StandardMethodCodecTest.encodeErrorEnvelopeWithThrowableTest(StandardMethodCodecTest.java:"));
    }
  }

  @Test
  public void encodeErrorEnvelopeWithStacktraceTest() {
    final Exception e = new IllegalArgumentException("foo");
    final ByteBuffer buffer =
        StandardMethodCodec.INSTANCE.encodeErrorEnvelopeWithStacktrace(
            "code", e.getMessage(), e, "error stacktrace");
    assertNotNull(buffer);
    buffer.flip();
    buffer.order(ByteOrder.nativeOrder());
    final byte flag = buffer.get();
    final Object code = StandardMessageCodec.INSTANCE.readValue(buffer);
    final Object message = StandardMessageCodec.INSTANCE.readValue(buffer);
    final Object details = StandardMessageCodec.INSTANCE.readValue(buffer);
    final Object stacktrace = StandardMessageCodec.INSTANCE.readValue(buffer);
    assertEquals("code", (String) code);
    assertEquals("foo", (String) message);
    String stack = (String) details;
    assertTrue(
        stack.contains(
            "at io.flutter.plugin.common.StandardMethodCodecTest.encodeErrorEnvelopeWithStacktraceTest(StandardMethodCodecTest.java:"));
    assertEquals("error stacktrace", (String) stacktrace);
  }
}
