// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:flutter/services.dart' as flt;
import 'package:flutter_driver/driver_extension.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:flutter_test/flutter_test.dart';

import 'src/common.dart';

/// An extension that forwards [NativeCommand]s to a registered plugin.
const CommandExtension nativeDriverCommands = NativeDriverCommandExtension(
  flt.MethodChannel('native_driver'),
);

/// An extension that forwards [NativeCommand]s to a registered plugin.
///
/// This extension is used to communicate with a platform plugin, and relays
/// [NativeCommand]s to the platform plugin, and resolves the result as a
/// [NativeResult], both thin wrappers around a `Map<String, Object?>` (JSON
/// compatible object).
///
/// A singleton default instance of this class is [nativeDriverCommands].
final class NativeDriverCommandExtension implements CommandExtension {
  /// Creates a new [NativeDriverCommandExtension] with the given [channel].
  ///
  /// Can be used in exceptional cases where a custom [MethodChannel] is needed;
  /// otherwise, use the singleton [nativeDriverCommands].
  const NativeDriverCommandExtension(this._channel);
  final flt.MethodChannel _channel;

  @override
  Future<Result> call(
    Command command,
    WidgetController prober,
    CreateFinderFactory finderFactory,
    CommandHandlerFactory handlerFactory,
  ) async {
    if (command is! NativeCommand) {
      throw ArgumentError.value(command, 'command', 'Expected a NativeCommand');
    }
    if (await _builtInCall(command.method) case final Result result) {
      return result;
    }
    final Object? result = await _channel.invokeMethod<Object>(command.method, command.arguments);
    if (result == null) {
      return const _MethodChannelResult(<String, Object?>{});
    }
    if (result is! Map<String, Object?>) {
      throw ArgumentError.value(result, 'result', 'Expected a Map<String, Object?>');
    }
    return _MethodChannelResult(result);
  }

  // While these could have been implemented in native code, they are already
  // available as engine-bundled platform plugins, so using them directly
  // reduces the amount of code to be written and maintained.
  Future<Result?> _builtInCall(String method) async {
    switch (method) {
      case 'rotate_landscape':
        await flt.SystemChrome.setPreferredOrientations(const <flt.DeviceOrientation>[
          flt.DeviceOrientation.landscapeLeft,
        ]);
        return Result.empty;
      case 'rotate_default':
        await flt.SystemChrome.setPreferredOrientations(const <flt.DeviceOrientation>[]);
        return Result.empty;
      default:
        return null;
    }
  }

  @override
  String get commandKind => 'native_driver';

  @override
  NativeCommand deserialize(
    Map<String, String> params,
    DeserializeFinderFactory finderFactory,
    DeserializeCommandFactory commandFactory,
  ) {
    final String? method = params['method'];
    if (method == null) {
      throw ArgumentError.value(params, 'params', 'Missing method');
    }
    final String? arguments = params['arguments'];
    final Map<String, Object?>? decoded;
    if (arguments == null) {
      decoded = null;
    } else {
      final Object? intermediate = json.decode(arguments);
      if (intermediate is! Map<String, Object?>) {
        throw ArgumentError.value(arguments, 'arguments', 'Expected a Map<String, Object?>');
      }
      decoded = intermediate;
    }
    return NativeCommand(method, arguments: decoded);
  }
}

final class _MethodChannelResult implements Result {
  const _MethodChannelResult(this._json);
  final Map<String, Object?> _json;

  @override
  Map<String, Object?> toJson() => _json;
}
