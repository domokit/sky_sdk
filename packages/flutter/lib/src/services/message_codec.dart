// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import 'platform_channel.dart';

export 'dart:typed_data' show ByteData;

/// A message encoding/decoding mechanism.
///
/// Both operations throw an exception, if conversion fails. Such situations
/// should be treated as programming errors.
///
/// See also:
///
///  * [BasicMessageChannel], which use [MessageCodec]s for communication
///    between Flutter and platform plugins.
abstract class MessageCodec<T> {
  /// Encodes the specified [message] in binary.
  ///
  /// Returns null if the message is null.
  ByteData? encodeMessage(T message);

  /// Decodes the specified [message] from binary.
  ///
  /// Returns null if the message is null.
  T? decodeMessage(ByteData? message);
}

/// A command object representing the invocation of a named method.
@pragma('vm:keep-name')
@immutable
class MethodCall {
  /// Creates a [MethodCall] representing the invocation of [method] with the
  /// specified [arguments].
  const MethodCall(this.method, [this.arguments]);

  /// The name of the method to be called.
  final String method;

  /// The arguments for the method.
  ///
  /// Must be a valid value for the [MethodCodec] used.
  ///
  /// This property is `dynamic`, which means type-checking is skipped when accessing
  /// this property. To minimize the risk of type errors at runtime, the value should
  /// be cast to `Object?` when accessed.
  final dynamic arguments;

  @override
  String toString() => '${objectRuntimeType(this, 'MethodCall')}($method, $arguments)';
}

/// A codec for method calls and enveloped results.
///
/// All operations throw an exception, if conversion fails.
///
/// See also:
///
///  * [MethodChannel], which use [MethodCodec]s for communication
///    between Flutter and platform plugins.
///  * [EventChannel], which use [MethodCodec]s for communication
///    between Flutter and platform plugins.
abstract class MethodCodec {
  /// Encodes the specified [methodCall] into binary.
  ByteData encodeMethodCall(MethodCall methodCall);

  /// Decodes the specified [methodCall] from binary.
  MethodCall decodeMethodCall(ByteData? methodCall);

  /// Decodes the specified result [envelope] from binary.
  ///
  /// Throws [PlatformException], if [envelope] represents an error, otherwise
  /// returns the enveloped result.
  ///
  /// The type returned from [decodeEnvelope] is `dynamic` (not `Object?`),
  /// which means *no type checking is performed on its return value*. It is
  /// strongly recommended that the return value be immediately cast to a known
  /// type to prevent runtime errors due to typos that the type checker could
  /// otherwise catch.
  dynamic decodeEnvelope(ByteData envelope);

  /// Encodes a successful [result] into a binary envelope.
  ByteData encodeSuccessEnvelope(Object? result);

  /// Encodes an error result into a binary envelope.
  ///
  /// The specified error [code], human-readable error [message] and error
  /// [details] correspond to the fields of [PlatformException].
  ByteData encodeErrorEnvelope({required String code, String? message, Object? details});
}

/// Thrown to indicate that a platform interaction failed in the platform
/// plugin.
///
/// See also:
///
///  * [MethodCodec], which throws a [PlatformException], if a received result
///    envelope represents an error.
///  * [MethodChannel.invokeMethod], which completes the returned future
///    with a [PlatformException], if invoking the platform plugin method
///    results in an error envelope.
///  * [EventChannel.receiveBroadcastStream], which emits
///    [PlatformException]s as error events, whenever an event received from the
///    platform plugin is wrapped in an error envelope.
class PlatformException implements Exception {
  /// Creates a [PlatformException] with the specified error [code] and optional
  /// [message], and with the optional error [details] which must be a valid
  /// value for the [MethodCodec] involved in the interaction.
  PlatformException({required this.code, this.message, this.details, this.stacktrace});

  /// An error code.
  final String code;

  /// A human-readable error message, possibly null.
  final String? message;

  /// Error details, possibly null.
  ///
  /// This property is `dynamic`, which means type-checking is skipped when accessing
  /// this property. To minimize the risk of type errors at runtime, the value should
  /// be cast to `Object?` when accessed.
  final dynamic details;

  /// Native stacktrace for the error, possibly null.
  ///
  /// This contains the native platform stack trace, not the Dart stack trace.
  ///
  /// The stack trace for Dart exceptions can be obtained using try-catch blocks, for example:
  ///
  /// ```dart
  /// try {
  ///   // ...
  /// } catch (e, stacktrace) {
  ///   print(stacktrace);
  /// }
  /// ```
  ///
  /// On Android this field is populated when a `RuntimeException` or a subclass of it is thrown in the method call handler,
  /// as shown in the following example:
  ///
  /// ```kotlin
  /// import androidx.annotation.NonNull
  /// import io.flutter.embedding.android.FlutterActivity
  /// import io.flutter.embedding.engine.FlutterEngine
  /// import io.flutter.plugin.common.MethodChannel
  ///
  /// class MainActivity: FlutterActivity() {
  ///   private val CHANNEL = "channel_name"
  ///
  ///   override fun configureFlutterEngine(@NonNull flutterEngine: FlutterEngine) {
  ///     super.configureFlutterEngine(flutterEngine)
  ///     MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CHANNEL).setMethodCallHandler {
  ///       call, result -> throw RuntimeException("Oh no")
  ///     }
  ///   }
  /// }
  /// ```
  ///
  /// It is also populated on Android if the method channel result is not serializable.
  /// If the result is not serializable, an exception gets thrown during the serialization process.
  /// This can be seen in the following example:
  ///
  /// ```kotlin
  /// import androidx.annotation.NonNull
  /// import io.flutter.embedding.android.FlutterActivity
  /// import io.flutter.embedding.engine.FlutterEngine
  /// import io.flutter.plugin.common.MethodChannel
  ///
  /// class MainActivity: FlutterActivity() {
  ///   private val CHANNEL = "channel_name"
  ///
  ///   override fun configureFlutterEngine(@NonNull flutterEngine: FlutterEngine) {
  ///     super.configureFlutterEngine(flutterEngine)
  ///     MethodChannel(flutterEngine.dartExecutor.binaryMessenger, CHANNEL).setMethodCallHandler {
  ///       call, result -> result.success(Object())
  ///     }
  ///   }
  /// }
  /// ```
  ///
  /// In the cases described above, the content of [stacktrace] will be the unprocessed output of calling `toString()` on the exception.
  ///
  /// MacOS, iOS, Linux and Windows don't support querying the native stacktrace.
  ///
  /// On custom Flutter embedders this value will be null on platforms that don't support querying the call stack.
  final String? stacktrace;

  @override
  String toString() => 'PlatformException($code, $message, $details, $stacktrace)';
}

/// Thrown to indicate that a platform interaction failed to find a handling
/// plugin.
///
/// See also:
///
///  * [MethodChannel.invokeMethod], which completes the returned future
///    with a [MissingPluginException], if no plugin handler for the method call
///    was found.
///  * [OptionalMethodChannel.invokeMethod], which completes the returned future
///    with null, if no plugin handler for the method call was found.
class MissingPluginException implements Exception {
  /// Creates a [MissingPluginException] with an optional human-readable
  /// error message.
  MissingPluginException([this.message]);

  /// A human-readable error message, possibly null.
  final String? message;

  @override
  String toString() => 'MissingPluginException($message)';
}
