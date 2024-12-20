// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:meta/meta.dart';

/// A message encoding/decoding mechanism.
///
/// Both operations throw an exception, if conversion fails. Such situations
/// should be treated as programming errors.
///
/// See also:
///
/// * [BasicMessageChannel], which use [MessageCodec]s for communication
///   between Flutter and platform plugins.
abstract class MessageCodec<T> {
  /// Encodes the specified [message] in binary.
  ///
  /// Returns null if the message is null.
  ByteData? encodeMessage(T message);

  /// Decodes the specified [message] from binary.
  ///
  /// Returns null if the message is null.
  T decodeMessage(ByteData message);
}

/// An command object representing the invocation of a named method.
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
  final dynamic arguments;

  @override
  String toString() => '$runtimeType($method, $arguments)';
}

/// A codec for method calls and enveloped results.
///
/// All operations throw an exception, if conversion fails.
///
/// See also:
///
/// * [MethodChannel], which use [MethodCodec]s for communication
///   between Flutter and platform plugins.
/// * [EventChannel], which use [MethodCodec]s for communication
///   between Flutter and platform plugins.
abstract class MethodCodec {
  /// Encodes the specified [methodCall] into binary.
  ByteData? encodeMethodCall(MethodCall methodCall);

  /// Decodes the specified [methodCall] from binary.
  MethodCall decodeMethodCall(ByteData? methodCall);

  /// Decodes the specified result [envelope] from binary.
  ///
  /// Throws [PlatformException], if [envelope] represents an error, otherwise
  /// returns the enveloped result.
  dynamic decodeEnvelope(ByteData envelope);

  /// Encodes a successful [result] into a binary envelope.
  ByteData? encodeSuccessEnvelope(dynamic result);

  /// Encodes an error result into a binary envelope.
  ///
  /// The specified error [code], human-readable error [message], and error
  /// [details] correspond to the fields of [PlatformException].
  ByteData? encodeErrorEnvelope({required String code, String? message, dynamic details});
}

/// Thrown to indicate that a platform interaction failed in the platform
/// plugin.
///
/// See also:
///
/// * [MethodCodec], which throws a [PlatformException], if a received result
///   envelope represents an error.
/// * [MethodChannel.invokeMethod], which completes the returned future
///   with a [PlatformException], if invoking the platform plugin method
///   results in an error envelope.
/// * [EventChannel.receiveBroadcastStream], which emits
///   [PlatformException]s as error events, whenever an event received from the
///   platform plugin is wrapped in an error envelope.
class PlatformException implements Exception {
  /// Creates a [PlatformException] with the specified error [code] and optional
  /// [message], and with the optional error [details] which must be a valid
  /// value for the [MethodCodec] involved in the interaction.
  PlatformException({required this.code, this.message, this.details});

  /// An error code.
  final String code;

  /// A human-readable error message, possibly null.
  final String? message;

  /// Error details, possibly null.
  final dynamic details;

  @override
  String toString() => 'PlatformException($code, $message, $details)';
}

/// Thrown to indicate that a platform interaction failed to find a handling
/// plugin.
///
/// See also:
///
/// * [MethodChannel.invokeMethod], which completes the returned future
///   with a [MissingPluginException], if no plugin handler for the method call
///   was found.
/// * [OptionalMethodChannel.invokeMethod], which completes the returned future
///   with null, if no plugin handler for the method call was found.
class MissingPluginException implements Exception {
  /// Creates a [MissingPluginException] with an optional human-readable
  /// error message.
  MissingPluginException([this.message]);

  /// A human-readable error message, possibly null.
  final String? message;

  @override
  String toString() => 'MissingPluginException($message)';
}
