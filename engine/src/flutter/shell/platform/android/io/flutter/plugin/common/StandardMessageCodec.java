// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.BuildConfig;
import io.flutter.Log;
import java.io.ByteArrayOutputStream;
import java.math.BigInteger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

/**
 * MessageCodec using the Flutter standard binary encoding.
 *
 * <p>This codec is guaranteed to be compatible with the corresponding <a
 * href="https://api.flutter.dev/flutter/services/StandardMessageCodec-class.html">StandardMessageCodec</a>
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * <p>Supported messages are acyclic values of these forms:
 *
 * <ul>
 *   <li>null
 *   <li>Booleans
 *   <li>Bytes, Shorts, Integers, Longs
 *   <li>BigIntegers (see below)
 *   <li>Floats, Doubles
 *   <li>Strings
 *   <li>byte[], int[], long[], float[], double[]
 *   <li>Lists of supported values
 *   <li>Maps with supported keys and values
 * </ul>
 *
 * <p>On the Dart side, these values are represented as follows:
 *
 * <ul>
 *   <li>null: null
 *   <li>Boolean: bool
 *   <li>Byte, Short, Integer, Long: int
 *   <li>Float, Double: double
 *   <li>String: String
 *   <li>byte[]: Uint8List
 *   <li>int[]: Int32List
 *   <li>long[]: Int64List
 *   <li>float[]: Float32List
 *   <li>double[]: Float64List
 *   <li>List: List
 *   <li>Map: Map
 * </ul>
 *
 * <p>BigIntegers are represented in Dart as strings with the hexadecimal representation of the
 * integer's value.
 *
 * <p>To extend the codec, overwrite the writeValue and readValueOfType methods.
 */
public class StandardMessageCodec implements MessageCodec<Object> {
  private static final String TAG = "StandardMessageCodec#";
  public static final StandardMessageCodec INSTANCE = new StandardMessageCodec();

  @Override
  @Nullable
  public ByteBuffer encodeMessage(@Nullable Object message) {
    if (message == null) {
      return null;
    }
    final ExposedByteArrayOutputStream stream = new ExposedByteArrayOutputStream();
    writeValue(stream, message);
    final ByteBuffer buffer = ByteBuffer.allocateDirect(stream.size());
    buffer.put(stream.buffer(), 0, stream.size());
    return buffer;
  }

  @Override
  @Nullable
  public Object decodeMessage(@Nullable ByteBuffer message) {
    if (message == null) {
      return null;
    }
    message.order(ByteOrder.nativeOrder());
    final Object value = readValue(message);
    if (message.hasRemaining()) {
      throw new IllegalArgumentException("Message corrupted");
    }
    return value;
  }

  private static final boolean LITTLE_ENDIAN = ByteOrder.nativeOrder() == ByteOrder.LITTLE_ENDIAN;
  private static final Charset UTF8 = Charset.forName("UTF8");
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

  /**
   * Writes an int representing a size to the specified stream. Uses an expanding code of 1 to 5
   * bytes to optimize for small values.
   */
  protected static final void writeSize(@NonNull ByteArrayOutputStream stream, int value) {
    if (BuildConfig.DEBUG && 0 > value) {
      Log.e(TAG, "Attempted to write a negative size.");
    }
    if (value < 254) {
      stream.write(value);
    } else if (value <= 0xffff) {
      stream.write(254);
      writeChar(stream, value);
    } else {
      stream.write(255);
      writeInt(stream, value);
    }
  }

  /** Writes the least significant two bytes of the specified int to the specified stream. */
  protected static final void writeChar(@NonNull ByteArrayOutputStream stream, int value) {
    if (LITTLE_ENDIAN) {
      stream.write(value);
      stream.write(value >>> 8);
    } else {
      stream.write(value >>> 8);
      stream.write(value);
    }
  }

  /** Writes the specified int as 4 bytes to the specified stream. */
  protected static final void writeInt(@NonNull ByteArrayOutputStream stream, int value) {
    if (LITTLE_ENDIAN) {
      stream.write(value);
      stream.write(value >>> 8);
      stream.write(value >>> 16);
      stream.write(value >>> 24);
    } else {
      stream.write(value >>> 24);
      stream.write(value >>> 16);
      stream.write(value >>> 8);
      stream.write(value);
    }
  }

  /** Writes the specified long as 8 bytes to the specified stream. */
  protected static final void writeLong(@NonNull ByteArrayOutputStream stream, long value) {
    if (LITTLE_ENDIAN) {
      stream.write((byte) value);
      stream.write((byte) (value >>> 8));
      stream.write((byte) (value >>> 16));
      stream.write((byte) (value >>> 24));
      stream.write((byte) (value >>> 32));
      stream.write((byte) (value >>> 40));
      stream.write((byte) (value >>> 48));
      stream.write((byte) (value >>> 56));
    } else {
      stream.write((byte) (value >>> 56));
      stream.write((byte) (value >>> 48));
      stream.write((byte) (value >>> 40));
      stream.write((byte) (value >>> 32));
      stream.write((byte) (value >>> 24));
      stream.write((byte) (value >>> 16));
      stream.write((byte) (value >>> 8));
      stream.write((byte) value);
    }
  }

  /** Writes the specified double as 4 bytes to the specified stream */
  protected static final void writeFloat(@NonNull ByteArrayOutputStream stream, float value) {
    writeInt(stream, Float.floatToIntBits(value));
  }

  /** Writes the specified double as 8 bytes to the specified stream. */
  protected static final void writeDouble(@NonNull ByteArrayOutputStream stream, double value) {
    writeLong(stream, Double.doubleToLongBits(value));
  }

  /** Writes the length and then the actual bytes of the specified array to the specified stream. */
  protected static final void writeBytes(
      @NonNull ByteArrayOutputStream stream, @NonNull byte[] bytes) {
    writeSize(stream, bytes.length);
    stream.write(bytes, 0, bytes.length);
  }

  /**
   * Writes a number of padding bytes to the specified stream to ensure that the next value is
   * aligned to a whole multiple of the specified alignment. An example usage with alignment = 8 is
   * to ensure doubles are word-aligned in the stream.
   */
  protected static final void writeAlignment(@NonNull ByteArrayOutputStream stream, int alignment) {
    final int mod = stream.size() % alignment;
    if (mod != 0) {
      for (int i = 0; i < alignment - mod; i++) {
        stream.write(0);
      }
    }
  }

  /**
   * Writes a type discriminator byte and then a byte serialization of the specified value to the
   * specified stream.
   *
   * <p>Subclasses can extend the codec by overriding this method, calling super for values that the
   * extension does not handle.
   */
  protected void writeValue(@NonNull ByteArrayOutputStream stream, @Nullable Object value) {
    if (value == null || value.equals(null)) {
      stream.write(NULL);
    } else if (value instanceof Boolean) {
      stream.write(((Boolean) value).booleanValue() ? TRUE : FALSE);
    } else if (value instanceof Number) {
      if (value instanceof Integer || value instanceof Short || value instanceof Byte) {
        stream.write(INT);
        writeInt(stream, ((Number) value).intValue());
      } else if (value instanceof Long) {
        stream.write(LONG);
        writeLong(stream, (long) value);
      } else if (value instanceof Float || value instanceof Double) {
        stream.write(DOUBLE);
        writeAlignment(stream, 8);
        writeDouble(stream, ((Number) value).doubleValue());
      } else if (value instanceof BigInteger) {
        stream.write(BIGINT);
        writeBytes(stream, ((BigInteger) value).toString(16).getBytes(UTF8));
      } else {
        throw new IllegalArgumentException("Unsupported Number type: " + value.getClass());
      }
    } else if (value instanceof CharSequence) {
      stream.write(STRING);
      writeBytes(stream, value.toString().getBytes(UTF8));
    } else if (value instanceof byte[]) {
      stream.write(BYTE_ARRAY);
      writeBytes(stream, (byte[]) value);
    } else if (value instanceof int[]) {
      stream.write(INT_ARRAY);
      final int[] array = (int[]) value;
      writeSize(stream, array.length);
      writeAlignment(stream, 4);
      for (final int n : array) {
        writeInt(stream, n);
      }
    } else if (value instanceof long[]) {
      stream.write(LONG_ARRAY);
      final long[] array = (long[]) value;
      writeSize(stream, array.length);
      writeAlignment(stream, 8);
      for (final long n : array) {
        writeLong(stream, n);
      }
    } else if (value instanceof double[]) {
      stream.write(DOUBLE_ARRAY);
      final double[] array = (double[]) value;
      writeSize(stream, array.length);
      writeAlignment(stream, 8);
      for (final double d : array) {
        writeDouble(stream, d);
      }
    } else if (value instanceof List) {
      stream.write(LIST);
      final List<?> list = (List) value;
      writeSize(stream, list.size());
      for (final Object o : list) {
        writeValue(stream, o);
      }
    } else if (value instanceof Map) {
      stream.write(MAP);
      final Map<?, ?> map = (Map) value;
      writeSize(stream, map.size());
      for (final Entry<?, ?> entry : map.entrySet()) {
        writeValue(stream, entry.getKey());
        writeValue(stream, entry.getValue());
      }
    } else if (value instanceof float[]) {
      stream.write(FLOAT_ARRAY);
      final float[] array = (float[]) value;
      writeSize(stream, array.length);
      writeAlignment(stream, 4);
      for (final float f : array) {
        writeFloat(stream, f);
      }
    } else {
      throw new IllegalArgumentException(
          "Unsupported value: '" + value + "' of type '" + value.getClass() + "'");
    }
  }

  /** Reads an int representing a size as written by writeSize. */
  protected static final int readSize(@NonNull ByteBuffer buffer) {
    if (!buffer.hasRemaining()) {
      throw new IllegalArgumentException("Message corrupted");
    }
    final int value = buffer.get() & 0xff;
    if (value < 254) {
      return value;
    } else if (value == 254) {
      return buffer.getChar();
    } else {
      return buffer.getInt();
    }
  }

  /** Reads a byte array as written by writeBytes. */
  @NonNull
  protected static final byte[] readBytes(@NonNull ByteBuffer buffer) {
    final int length = readSize(buffer);
    final byte[] bytes = new byte[length];
    buffer.get(bytes);
    return bytes;
  }

  /** Reads alignment padding bytes as written by writeAlignment. */
  protected static final void readAlignment(@NonNull ByteBuffer buffer, int alignment) {
    final int mod = buffer.position() % alignment;
    if (mod != 0) {
      buffer.position(buffer.position() + alignment - mod);
    }
  }

  /** Reads a value as written by writeValue. */
  @Nullable
  protected final Object readValue(@NonNull ByteBuffer buffer) {
    if (!buffer.hasRemaining()) {
      throw new IllegalArgumentException("Message corrupted");
    }
    final byte type = buffer.get();
    return readValueOfType(type, buffer);
  }

  /**
   * Reads a value of the specified type.
   *
   * <p>Subclasses may extend the codec by overriding this method, calling super for types that the
   * extension does not handle.
   */
  @Nullable
  protected Object readValueOfType(byte type, @NonNull ByteBuffer buffer) {
    final Object result;
    switch (type) {
      case NULL:
        result = null;
        break;
      case TRUE:
        result = true;
        break;
      case FALSE:
        result = false;
        break;
      case INT:
        result = buffer.getInt();
        break;
      case LONG:
        result = buffer.getLong();
        break;
      case BIGINT:
        {
          final byte[] hex = readBytes(buffer);
          result = new BigInteger(new String(hex, UTF8), 16);
          break;
        }
      case DOUBLE:
        readAlignment(buffer, 8);
        result = buffer.getDouble();
        break;
      case STRING:
        {
          final byte[] bytes = readBytes(buffer);
          result = new String(bytes, UTF8);
          break;
        }
      case BYTE_ARRAY:
        {
          result = readBytes(buffer);
          break;
        }
      case INT_ARRAY:
        {
          final int length = readSize(buffer);
          final int[] array = new int[length];
          readAlignment(buffer, 4);
          buffer.asIntBuffer().get(array);
          result = array;
          buffer.position(buffer.position() + 4 * length);
          break;
        }
      case LONG_ARRAY:
        {
          final int length = readSize(buffer);
          final long[] array = new long[length];
          readAlignment(buffer, 8);
          buffer.asLongBuffer().get(array);
          result = array;
          buffer.position(buffer.position() + 8 * length);
          break;
        }
      case DOUBLE_ARRAY:
        {
          final int length = readSize(buffer);
          final double[] array = new double[length];
          readAlignment(buffer, 8);
          buffer.asDoubleBuffer().get(array);
          result = array;
          buffer.position(buffer.position() + 8 * length);
          break;
        }
      case LIST:
        {
          final int size = readSize(buffer);
          final List<Object> list = new ArrayList<>(size);
          for (int i = 0; i < size; i++) {
            list.add(readValue(buffer));
          }
          result = list;
          break;
        }
      case MAP:
        {
          final int size = readSize(buffer);
          final Map<Object, Object> map = new HashMap<>();
          for (int i = 0; i < size; i++) {
            map.put(readValue(buffer), readValue(buffer));
          }
          result = map;
          break;
        }
      case FLOAT_ARRAY:
        {
          final int length = readSize(buffer);
          final float[] array = new float[length];
          readAlignment(buffer, 4);
          buffer.asFloatBuffer().get(array);
          result = array;
          buffer.position(buffer.position() + 4 * length);
          break;
        }
      default:
        throw new IllegalArgumentException("Message corrupted");
    }
    return result;
  }

  static final class ExposedByteArrayOutputStream extends ByteArrayOutputStream {
    byte[] buffer() {
      return buf;
    }
  }
}
