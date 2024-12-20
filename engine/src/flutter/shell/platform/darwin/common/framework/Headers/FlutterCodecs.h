// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERCODECS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERCODECS_H_

#import <Foundation/Foundation.h>

#import "FlutterMacros.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * A message encoding/decoding mechanism.
 */
FLUTTER_DARWIN_EXPORT
@protocol FlutterMessageCodec
/**
 * Returns a shared instance of this `FlutterMessageCodec`.
 */
+ (instancetype)sharedInstance;

/**
 * Encodes the specified message into binary.
 *
 * @param message The message.
 * @return The binary encoding, or `nil`, if `message` was `nil`.
 */
- (NSData* _Nullable)encode:(id _Nullable)message;

/**
 * Decodes the specified message from binary.
 *
 * @param message The message.
 * @return The decoded message, or `nil`, if `message` was `nil`.
 */
- (id _Nullable)decode:(NSData* _Nullable)message;
@end

/**
 * A `FlutterMessageCodec` using unencoded binary messages, represented as
 * `NSData` instances.
 *
 * This codec is guaranteed to be compatible with the corresponding
 * [BinaryCodec](https://api.flutter.dev/flutter/services/BinaryCodec-class.html)
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * On the Dart side, messages are represented using `ByteData`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterBinaryCodec : NSObject <FlutterMessageCodec>
@end

/**
 * A `FlutterMessageCodec` using UTF-8 encoded `NSString` messages.
 *
 * This codec is guaranteed to be compatible with the corresponding
 * [StringCodec](https://api.flutter.dev/flutter/services/StringCodec-class.html)
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterStringCodec : NSObject <FlutterMessageCodec>
@end

/**
 * A `FlutterMessageCodec` using UTF-8 encoded JSON messages.
 *
 * This codec is guaranteed to be compatible with the corresponding
 * [JSONMessageCodec](https://api.flutter.dev/flutter/services/JSONMessageCodec-class.html)
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * Supports values accepted by `NSJSONSerialization` plus top-level
 * `nil`, `NSNumber`, and `NSString`.
 *
 * On the Dart side, JSON messages are handled by the JSON facilities of the
 * [`dart:convert`](https://api.dartlang.org/stable/dart-convert/JSON-constant.html)
 * package.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterJSONMessageCodec : NSObject <FlutterMessageCodec>
@end

/**
 * A writer of the Flutter standard binary encoding.
 *
 * See `FlutterStandardMessageCodec` for details on the encoding.
 *
 * The encoding is extensible via subclasses overriding `writeValue`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterStandardWriter : NSObject
/**
 * Create a `FlutterStandardWriter` who will write to \p data.
 */
- (instancetype)initWithData:(NSMutableData*)data;
/** Write a 8-bit byte. */
- (void)writeByte:(UInt8)value;
/** Write an array of \p bytes of size \p length. */
- (void)writeBytes:(const void*)bytes length:(NSUInteger)length;
/** Write an array of bytes contained in \p data. */
- (void)writeData:(NSData*)data;
/** Write 32-bit unsigned integer that represents a \p size of a collection. */
- (void)writeSize:(UInt32)size;
/** Write zero padding until data is aligned with \p alignment. */
- (void)writeAlignment:(UInt8)alignment;
/** Write a string with UTF-8 encoding. */
- (void)writeUTF8:(NSString*)value;
/** Introspects into an object and writes its representation.
 *
 * Supported Data Types:
 *  - NSNull
 *  - NSNumber
 *  - NSString (as UTF-8)
 *  - FlutterStandardTypedData
 *  - NSArray of supported types
 *  - NSDictionary of supporte types
 *
 * NSAsserts on failure.
 */
- (void)writeValue:(id)value;
@end

/**
 * A reader of the Flutter standard binary encoding.
 *
 * See `FlutterStandardMessageCodec` for details on the encoding.
 *
 * The encoding is extensible via subclasses overriding `readValueOfType`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterStandardReader : NSObject
/**
 * Create a new `FlutterStandardReader` who reads from \p data.
 */
- (instancetype)initWithData:(NSData*)data;
/** Returns YES when the reader hasn't reached the end of its data. */
- (BOOL)hasMore;
/** Reads a byte value and increments the position. */
- (UInt8)readByte;
/** Reads a sequence of byte values of \p length and increments the position. */
- (void)readBytes:(void*)destination length:(NSUInteger)length;
/** Reads a sequence of byte values of \p length and increments the position. */
- (NSData*)readData:(NSUInteger)length;
/** Reads a 32-bit unsigned integer representing a collection size and increments the position.*/
- (UInt32)readSize;
/** Advances the read position until it is aligned with \p alignment. */
- (void)readAlignment:(UInt8)alignment;
/** Read a null terminated string encoded with UTF-8/ */
- (NSString*)readUTF8;
/**
 * Reads a byte for `FlutterStandardField` the decodes a value matching that type.
 *
 * See also: -[FlutterStandardWriter writeValue]
 */
- (nullable id)readValue;
/**
 * Decodes a value matching the \p type specified.
 *
 * See also:
 *   - `FlutterStandardField`
 *   - `-[FlutterStandardWriter writeValue]`
 */
- (nullable id)readValueOfType:(UInt8)type;
@end

/**
 * A factory of compatible reader/writer instances using the Flutter standard
 * binary encoding or extensions thereof.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterStandardReaderWriter : NSObject
/**
 * Create a new `FlutterStandardWriter` for writing to \p data.
 */
- (FlutterStandardWriter*)writerWithData:(NSMutableData*)data;
/**
 * Create a new `FlutterStandardReader` for reading from \p data.
 */
- (FlutterStandardReader*)readerWithData:(NSData*)data;
@end

/**
 * A `FlutterMessageCodec` using the Flutter standard binary encoding.
 *
 * This codec is guaranteed to be compatible with the corresponding
 * [StandardMessageCodec](https://api.flutter.dev/flutter/services/StandardMessageCodec-class.html)
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * Supported messages are acyclic values of these forms:
 *
 * - `nil` or `NSNull`
 * - `NSNumber` (including their representation of Boolean values)
 * - `NSString`
 * - `FlutterStandardTypedData`
 * - `NSArray` of supported values
 * - `NSDictionary` with supported keys and values
 *
 * On the Dart side, these values are represented as follows:
 *
 * - `nil` or `NSNull`: null
 * - `NSNumber`: `bool`, `int`, or `double`, depending on the contained value.
 * - `NSString`: `String`
 * - `FlutterStandardTypedData`: `Uint8List`, `Int32List`, `Int64List`, or `Float64List`
 * - `NSArray`: `List`
 * - `NSDictionary`: `Map`
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterStandardMessageCodec : NSObject <FlutterMessageCodec>
/**
 * Create a `FlutterStandardMessageCodec` who will read and write to \p readerWriter.
 */
+ (instancetype)codecWithReaderWriter:(FlutterStandardReaderWriter*)readerWriter;
@end

/**
 * Command object representing a method call on a `FlutterMethodChannel`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterMethodCall : NSObject
/**
 * Creates a method call for invoking the specified named method with the
 * specified arguments.
 *
 * @param method the name of the method to call.
 * @param arguments the arguments value.
 */
+ (instancetype)methodCallWithMethodName:(NSString*)method arguments:(id _Nullable)arguments;

/**
 * The method name.
 */
@property(readonly, nonatomic) NSString* method;

/**
 * The arguments.
 */
@property(readonly, nonatomic, nullable) id arguments;
@end

/**
 * Error object representing an unsuccessful outcome of invoking a method
 * on a `FlutterMethodChannel`, or an error event on a `FlutterEventChannel`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterError : NSObject
/**
 * Creates a `FlutterError` with the specified error code, message, and details.
 *
 * @param code An error code string for programmatic use.
 * @param message A human-readable error message.
 * @param details Custom error details.
 */
+ (instancetype)errorWithCode:(NSString*)code
                      message:(NSString* _Nullable)message
                      details:(id _Nullable)details;
/**
 The error code.
 */
@property(readonly, nonatomic) NSString* code;

/**
 The error message.
 */
@property(readonly, nonatomic, nullable) NSString* message;

/**
 The error details.
 */
@property(readonly, nonatomic, nullable) id details;
@end

/**
 * Type of numeric data items encoded in a `FlutterStandardDataType`.
 *
 * - FlutterStandardDataTypeUInt8: plain bytes
 * - FlutterStandardDataTypeInt32: 32-bit signed integers
 * - FlutterStandardDataTypeInt64: 64-bit signed integers
 * - FlutterStandardDataTypeFloat64: 64-bit floats
 */
typedef NS_ENUM(NSInteger, FlutterStandardDataType) {
  // NOLINTBEGIN(readability-identifier-naming)
  FlutterStandardDataTypeUInt8,
  FlutterStandardDataTypeInt32,
  FlutterStandardDataTypeInt64,
  FlutterStandardDataTypeFloat32,
  FlutterStandardDataTypeFloat64,
  // NOLINTEND(readability-identifier-naming)
};

/**
 * A byte buffer holding `UInt8`, `SInt32`, `SInt64`, or `Float64` values, used
 * with `FlutterStandardMessageCodec` and `FlutterStandardMethodCodec`.
 *
 * Two's complement encoding is used for signed integers. IEEE754
 * double-precision representation is used for floats. The platform's native
 * endianness is assumed.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterStandardTypedData : NSObject
/**
 * Creates a `FlutterStandardTypedData` which interprets the specified data
 * as plain bytes.
 *
 * @param data the byte data.
 */
+ (instancetype)typedDataWithBytes:(NSData*)data;

/**
 * Creates a `FlutterStandardTypedData` which interprets the specified data
 * as 32-bit signed integers.
 *
 * @param data the byte data. The length must be divisible by 4.
 */
+ (instancetype)typedDataWithInt32:(NSData*)data;

/**
 * Creates a `FlutterStandardTypedData` which interprets the specified data
 * as 64-bit signed integers.
 *
 * @param data the byte data. The length must be divisible by 8.
 */
+ (instancetype)typedDataWithInt64:(NSData*)data;

/**
 * Creates a `FlutterStandardTypedData` which interprets the specified data
 * as 32-bit floats.
 *
 * @param data the byte data. The length must be divisible by 8.
 */
+ (instancetype)typedDataWithFloat32:(NSData*)data;

/**
 * Creates a `FlutterStandardTypedData` which interprets the specified data
 * as 64-bit floats.
 *
 * @param data the byte data. The length must be divisible by 8.
 */
+ (instancetype)typedDataWithFloat64:(NSData*)data;

/**
 * The raw underlying data buffer.
 */
@property(readonly, nonatomic) NSData* data;

/**
 * The type of the encoded values.
 */
@property(readonly, nonatomic, assign) FlutterStandardDataType type;

/**
 * The number of value items encoded.
 */
@property(readonly, nonatomic, assign) UInt32 elementCount;

/**
 * The number of bytes used by the encoding of a single value item.
 */
@property(readonly, nonatomic, assign) UInt8 elementSize;
@end

/**
 * An arbitrarily large integer value, used with `FlutterStandardMessageCodec`
 * and `FlutterStandardMethodCodec`.
 */
FLUTTER_DARWIN_EXPORT
FLUTTER_UNAVAILABLE("Unavailable on 2018-08-31. Deprecated on 2018-01-09. "
                    "FlutterStandardBigInteger was needed because the Dart 1.0 int type had no "
                    "size limit. With Dart 2.0, the int type is a fixed-size, 64-bit signed "
                    "integer. If you need to communicate larger integers, use NSString encoding "
                    "instead.")
@interface FlutterStandardBigInteger : NSObject
@end

/**
 * A codec for method calls and enveloped results.
 *
 * Method calls are encoded as binary messages with enough structure that the
 * codec can extract a method name `NSString` and an arguments `NSObject`,
 * possibly `nil`. These data items are used to populate a `FlutterMethodCall`.
 *
 * Result envelopes are encoded as binary messages with enough structure that
 * the codec can determine whether the result was successful or an error. In
 * the former case, the codec can extract the result `NSObject`, possibly `nil`.
 * In the latter case, the codec can extract an error code `NSString`, a
 * human-readable `NSString` error message (possibly `nil`), and a custom
 * error details `NSObject`, possibly `nil`. These data items are used to
 * populate a `FlutterError`.
 */
FLUTTER_DARWIN_EXPORT
@protocol FlutterMethodCodec
/**
 * Provides access to a shared instance this codec.
 *
 * @return The shared instance.
 */
+ (instancetype)sharedInstance;

/**
 * Encodes the specified method call into binary.
 *
 * @param methodCall The method call. The arguments value
 *   must be supported by this codec.
 * @return The binary encoding.
 */
- (NSData*)encodeMethodCall:(FlutterMethodCall*)methodCall;

/**
 * Decodes the specified method call from binary.
 *
 * @param methodCall The method call to decode.
 * @return The decoded method call.
 */
- (FlutterMethodCall*)decodeMethodCall:(NSData*)methodCall;

/**
 * Encodes the specified successful result into binary.
 *
 * @param result The result. Must be a value supported by this codec.
 * @return The binary encoding.
 */
- (NSData*)encodeSuccessEnvelope:(id _Nullable)result;

/**
 * Encodes the specified error result into binary.
 *
 * @param error The error object. The error details value must be supported
 *   by this codec.
 * @return The binary encoding.
 */
- (NSData*)encodeErrorEnvelope:(FlutterError*)error;

/**
 * Deccodes the specified result envelope from binary.
 *
 * @param envelope The error object.
 * @return The result value, if the envelope represented a successful result,
 *   or a `FlutterError` instance, if not.
 */
- (id _Nullable)decodeEnvelope:(NSData*)envelope;
@end

/**
 * A `FlutterMethodCodec` using UTF-8 encoded JSON method calls and result
 * envelopes.
 *
 * This codec is guaranteed to be compatible with the corresponding
 * [JSONMethodCodec](https://api.flutter.dev/flutter/services/JSONMethodCodec-class.html)
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * Values supported as methods arguments and result payloads are
 * those supported as top-level or leaf values by `FlutterJSONMessageCodec`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterJSONMethodCodec : NSObject <FlutterMethodCodec>
@end

/**
 * A `FlutterMethodCodec` using the Flutter standard binary encoding.
 *
 * This codec is guaranteed to be compatible with the corresponding
 * [StandardMethodCodec](https://api.flutter.dev/flutter/services/StandardMethodCodec-class.html)
 * on the Dart side. These parts of the Flutter SDK are evolved synchronously.
 *
 * Values supported as method arguments and result payloads are those supported by
 * `FlutterStandardMessageCodec`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterStandardMethodCodec : NSObject <FlutterMethodCodec>
/**
 * Create a `FlutterStandardMethodCodec` who will read and write to \p readerWriter.
 */
+ (instancetype)codecWithReaderWriter:(FlutterStandardReaderWriter*)readerWriter;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERCODECS_H_
