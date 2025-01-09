// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter_driver/flutter_driver.dart';
library;

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart' show RendererBinding;
import 'package:flutter/scheduler.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import '../common/deserialization_factory.dart';
import '../common/error.dart';
import '../common/find.dart';
import '../common/handler_factory.dart';
import '../common/message.dart';
import '_extension_io.dart' if (dart.library.js_interop) '_extension_web.dart';

const String _extensionMethodName = 'driver';

/// Signature for the handler passed to [enableFlutterDriverExtension].
///
/// Messages are described in string form and should return a [Future] which
/// eventually completes to a string response.
typedef DataHandler = Future<String> Function(String? message);

class _DriverBinding extends BindingBase
    with
        SchedulerBinding,
        ServicesBinding,
        GestureBinding,
        PaintingBinding,
        SemanticsBinding,
        RendererBinding,
        WidgetsBinding,
        TestDefaultBinaryMessengerBinding {
  _DriverBinding(
    this._handler,
    this._silenceErrors,
    this._enableTextEntryEmulation,
    this.finders,
    this.commands,
  );

  final DataHandler? _handler;
  final bool _silenceErrors;
  final bool _enableTextEntryEmulation;
  final List<FinderExtension>? finders;
  final List<CommandExtension>? commands;

  // Because you can't really control which zone a driver test uses,
  // we override the test for zones here.
  @override
  bool debugCheckZone(String entryPoint) {
    return true;
  }

  @override
  void initServiceExtensions() {
    super.initServiceExtensions();
    final FlutterDriverExtension extension = FlutterDriverExtension(
      _handler,
      _silenceErrors,
      _enableTextEntryEmulation,
      finders: finders ?? const <FinderExtension>[],
      commands: commands ?? const <CommandExtension>[],
    );
    registerServiceExtension(name: _extensionMethodName, callback: extension.call);
    if (kIsWeb) {
      registerWebServiceExtension(extension.call);
    }
  }
}

// Examples can assume:
// import 'package:flutter_driver/flutter_driver.dart';
// import 'package:flutter/widgets.dart';
// import 'package:flutter_driver/driver_extension.dart';
// import 'package:flutter_test/flutter_test.dart' hide find;
// import 'package:flutter_test/flutter_test.dart' as flutter_test;
// typedef MyHomeWidget = Placeholder;
// abstract class SomeWidget extends StatelessWidget { const SomeWidget({super.key, required this.title}); final String title; }
// late FlutterDriver driver;
// abstract class StubNestedCommand { int get times; SerializableFinder get finder; }
// class StubCommandResult extends Result { const StubCommandResult(this.arg); final String arg; @override Map<String, dynamic> toJson() => <String, dynamic>{}; }
// abstract class StubProberCommand { int get times; SerializableFinder get finder; }

/// Enables Flutter Driver VM service extension.
///
/// This extension is required for tests that use `package:flutter_driver` to
/// drive applications from a separate process. In order to allow the driver
/// to interact with the application, this method changes the behavior of the
/// framework in several ways - including keyboard interaction and text
/// editing. Applications intended for release should never include this
/// method.
///
/// Call this function prior to running your application, e.g. before you call
/// `runApp`.
///
/// Optionally you can pass a [DataHandler] callback. It will be called if the
/// test calls [FlutterDriver.requestData].
///
/// `silenceErrors` will prevent exceptions from being logged. This is useful
/// for tests where exceptions are expected. Defaults to false. Any errors
/// will still be returned in the `response` field of the result JSON along
/// with an `isError` boolean.
///
/// The `enableTextEntryEmulation` parameter controls whether the application interacts
/// with the system's text entry methods or a mocked out version used by Flutter Driver.
/// If it is set to false, [FlutterDriver.enterText] will fail,
/// but testing the application with real keyboard input is possible.
/// This value may be updated during a test by calling [FlutterDriver.setTextEntryEmulation].
///
/// The `finders` and `commands` parameters are optional and used to add custom
/// finders or commands, as in the following example.
///
/// ```dart
/// void main() {
///   enableFlutterDriverExtension(
///     finders: <FinderExtension>[ SomeFinderExtension() ],
///     commands: <CommandExtension>[ SomeCommandExtension() ],
///   );
///
///   runApp(const MyHomeWidget());
/// }
///
/// class SomeFinderExtension extends FinderExtension {
///   @override
///   String get finderType => 'SomeFinder';
///
///   @override
///   SerializableFinder deserialize(Map<String, String> params, DeserializeFinderFactory finderFactory) {
///     return SomeFinder(params['title']!);
///   }
///
///   @override
///   Finder createFinder(SerializableFinder finder, CreateFinderFactory finderFactory) {
///     final SomeFinder someFinder = finder as SomeFinder;
///
///     return flutter_test.find.byElementPredicate((Element element) {
///       final Widget widget = element.widget;
///       if (widget is SomeWidget) {
///         return widget.title == someFinder.title;
///       }
///       return false;
///     });
///   }
/// }
///
/// // Use this class in a test anywhere where a SerializableFinder is expected.
/// class SomeFinder extends SerializableFinder {
///   const SomeFinder(this.title);
///
///   final String title;
///
///   @override
///   String get finderType => 'SomeFinder';
///
///   @override
///   Map<String, String> serialize() => super.serialize()..addAll(<String, String>{
///     'title': title,
///   });
/// }
///
/// class SomeCommandExtension extends CommandExtension {
///   @override
///   String get commandKind => 'SomeCommand';
///
///   @override
///   Future<Result> call(Command command, WidgetController prober, CreateFinderFactory finderFactory, CommandHandlerFactory handlerFactory) async {
///     final SomeCommand someCommand = command as SomeCommand;
///
///     // Deserialize [Finder]:
///     final Finder finder = finderFactory.createFinder(someCommand.finder);
///
///     // Wait for [Element]:
///     handlerFactory.waitForElement(finder);
///
///     // Alternatively, wait for [Element] absence:
///     handlerFactory.waitForAbsentElement(finder);
///
///     // Submit known [Command]s:
///     for (int i = 0; i < someCommand.times; i++) {
///       await handlerFactory.handleCommand(Tap(someCommand.finder), prober, finderFactory);
///     }
///
///     // Alternatively, use [WidgetController]:
///     for (int i = 0; i < someCommand.times; i++) {
///       await prober.tap(finder);
///     }
///
///     return const SomeCommandResult('foo bar');
///   }
///
///   @override
///   Command deserialize(Map<String, String> params, DeserializeFinderFactory finderFactory, DeserializeCommandFactory commandFactory) {
///     return SomeCommand.deserialize(params, finderFactory);
///   }
/// }
///
/// // Pass an instance of this class to `FlutterDriver.sendCommand` to invoke
/// // the custom command during a test.
/// class SomeCommand extends CommandWithTarget {
///   SomeCommand(super.finder, this.times, {super.timeout});
///
///   SomeCommand.deserialize(super.json, super.finderFactory)
///       : times = int.parse(json['times']!),
///         super.deserialize();
///
///   @override
///   Map<String, String> serialize() {
///     return super.serialize()..addAll(<String, String>{'times': '$times'});
///   }
///
///   @override
///   String get kind => 'SomeCommand';
///
///   final int times;
/// }
///
/// class SomeCommandResult extends Result {
///   const SomeCommandResult(this.resultParam);
///
///   final String resultParam;
///
///   @override
///   Map<String, dynamic> toJson() {
///     return <String, dynamic>{
///       'resultParam': resultParam,
///     };
///   }
/// }
/// ```
void enableFlutterDriverExtension({
  DataHandler? handler,
  bool silenceErrors = false,
  bool enableTextEntryEmulation = true,
  List<FinderExtension>? finders,
  List<CommandExtension>? commands,
}) {
  _DriverBinding(
    handler,
    silenceErrors,
    enableTextEntryEmulation,
    finders ?? <FinderExtension>[],
    commands ?? <CommandExtension>[],
  );
  assert(WidgetsBinding.instance is _DriverBinding);
}

/// Signature for functions that handle a command and return a result.
typedef CommandHandlerCallback = Future<Result?> Function(Command c);

/// Signature for functions that deserialize a JSON map to a command object.
typedef CommandDeserializerCallback = Command Function(Map<String, String> params);

/// Used to expand the new [Finder].
abstract class FinderExtension {
  /// Identifies the type of finder to be used by the driver extension.
  String get finderType;

  /// Deserializes the finder from JSON generated by [SerializableFinder.serialize].
  ///
  /// Use [finderFactory] to deserialize nested [Finder]s.
  ///
  /// See also:
  ///   * [Ancestor], a finder that uses other [Finder]s as parameters.
  SerializableFinder deserialize(
    Map<String, String> params,
    DeserializeFinderFactory finderFactory,
  );

  /// Signature for functions that run the given finder and return the [Element]
  /// found, if any, or null otherwise.
  ///
  /// Call [finderFactory] to create known, nested [Finder]s from [SerializableFinder]s.
  Finder createFinder(SerializableFinder finder, CreateFinderFactory finderFactory);
}

/// Used to expand the new [Command].
///
/// See also:
///   * [CommandWithTarget], a base class for [Command]s with [Finder]s.
abstract class CommandExtension {
  /// Identifies the type of command to be used by the driver extension.
  String get commandKind;

  /// Deserializes the command from JSON generated by [Command.serialize].
  ///
  /// Use [finderFactory] to deserialize nested [Finder]s.
  /// Usually used for [CommandWithTarget]s.
  ///
  /// Call [commandFactory] to deserialize commands specified as parameters.
  ///
  /// See also:
  ///   * [CommandWithTarget], a base class for commands with target finders.
  ///   * [Tap], a command that uses [Finder]s as parameter.
  Command deserialize(
    Map<String, String> params,
    DeserializeFinderFactory finderFactory,
    DeserializeCommandFactory commandFactory,
  );

  /// Calls action for given [command].
  /// Returns action [Result].
  /// Invoke [prober] functions to perform widget actions.
  /// Use [finderFactory] to create [Finder]s from [SerializableFinder].
  /// Call [handlerFactory] to invoke other [Command]s or [CommandWithTarget]s.
  ///
  /// The following example shows invoking nested command with [handlerFactory].
  ///
  /// ```dart
  /// @override
  /// Future<Result> call(Command command, WidgetController prober, CreateFinderFactory finderFactory, CommandHandlerFactory handlerFactory) async {
  ///   final StubNestedCommand stubCommand = command as StubNestedCommand;
  ///   for (int i = 0; i < stubCommand.times; i++) {
  ///     await handlerFactory.handleCommand(Tap(stubCommand.finder), prober, finderFactory);
  ///   }
  ///   return const StubCommandResult('stub response');
  /// }
  /// ```
  ///
  /// Check the example below for direct [WidgetController] usage with [prober]:
  ///
  /// ```dart
  ///   @override
  /// Future<Result> call(Command command, WidgetController prober, CreateFinderFactory finderFactory, CommandHandlerFactory handlerFactory) async {
  ///   final StubProberCommand stubCommand = command as StubProberCommand;
  ///   for (int i = 0; i < stubCommand.times; i++) {
  ///     await prober.tap(finderFactory.createFinder(stubCommand.finder));
  ///   }
  ///   return const StubCommandResult('stub response');
  /// }
  /// ```
  Future<Result> call(
    Command command,
    WidgetController prober,
    CreateFinderFactory finderFactory,
    CommandHandlerFactory handlerFactory,
  );
}

/// The class that manages communication between a Flutter Driver test and the
/// application being remote-controlled, on the application side.
///
/// This is not normally used directly. It is instantiated automatically when
/// calling [enableFlutterDriverExtension].
@visibleForTesting
class FlutterDriverExtension
    with
        DeserializeFinderFactory,
        CreateFinderFactory,
        DeserializeCommandFactory,
        CommandHandlerFactory {
  /// Creates an object to manage a Flutter Driver connection.
  FlutterDriverExtension(
    this._requestDataHandler,
    this._silenceErrors,
    this._enableTextEntryEmulation, {
    List<FinderExtension> finders = const <FinderExtension>[],
    List<CommandExtension> commands = const <CommandExtension>[],
  }) {
    if (_enableTextEntryEmulation) {
      registerTextInput();
    }

    for (final FinderExtension finder in finders) {
      _finderExtensions[finder.finderType] = finder;
    }

    for (final CommandExtension command in commands) {
      _commandExtensions[command.commandKind] = command;
    }
  }

  final WidgetController _prober = LiveWidgetController(WidgetsBinding.instance);

  final DataHandler? _requestDataHandler;

  final bool _silenceErrors;

  final bool _enableTextEntryEmulation;

  void _log(String message) {
    driverLog('FlutterDriverExtension', message);
  }

  final Map<String, FinderExtension> _finderExtensions = <String, FinderExtension>{};
  final Map<String, CommandExtension> _commandExtensions = <String, CommandExtension>{};

  /// Processes a driver command configured by [params] and returns a result
  /// as an arbitrary JSON object.
  ///
  /// [params] must contain key "command" whose value is a string that
  /// identifies the kind of the command and its corresponding
  /// [CommandDeserializerCallback]. Other keys and values are specific to the
  /// concrete implementation of [Command] and [CommandDeserializerCallback].
  ///
  /// The returned JSON is command specific. Generally the caller deserializes
  /// the result into a subclass of [Result], but that's not strictly required.
  @visibleForTesting
  Future<Map<String, dynamic>> call(Map<String, String> params) async {
    final String commandKind = params['command']!;
    try {
      final Command command = deserializeCommand(params, this);
      assert(
        WidgetsBinding.instance.isRootWidgetAttached || !command.requiresRootWidgetAttached,
        'No root widget is attached; have you remembered to call runApp()?',
      );
      Future<Result> responseFuture = handleCommand(command, _prober, this);
      if (command.timeout != null) {
        responseFuture = responseFuture.timeout(command.timeout!);
      }
      final Result response = await responseFuture;
      return _makeResponse(response.toJson());
    } on TimeoutException catch (error, stackTrace) {
      final String message = 'Timeout while executing $commandKind: $error\n$stackTrace';
      _log(message);
      return _makeResponse(message, isError: true);
    } catch (error, stackTrace) {
      final String message =
          'Uncaught extension error while executing $commandKind: $error\n$stackTrace';
      if (!_silenceErrors) {
        _log(message);
      }
      return _makeResponse(message, isError: true);
    }
  }

  Map<String, dynamic> _makeResponse(dynamic response, {bool isError = false}) {
    return <String, dynamic>{'isError': isError, 'response': response};
  }

  @override
  SerializableFinder deserializeFinder(Map<String, String> json) {
    final String? finderType = json['finderType'];
    if (_finderExtensions.containsKey(finderType)) {
      return _finderExtensions[finderType]!.deserialize(json, this);
    }

    return super.deserializeFinder(json);
  }

  @override
  Finder createFinder(SerializableFinder finder) {
    final String finderType = finder.finderType;
    if (_finderExtensions.containsKey(finderType)) {
      return _finderExtensions[finderType]!.createFinder(finder, this);
    }

    return super.createFinder(finder);
  }

  @override
  Command deserializeCommand(Map<String, String> params, DeserializeFinderFactory finderFactory) {
    final String? kind = params['command'];
    if (_commandExtensions.containsKey(kind)) {
      return _commandExtensions[kind]!.deserialize(params, finderFactory, this);
    }

    return super.deserializeCommand(params, finderFactory);
  }

  @override
  @protected
  DataHandler? getDataHandler() {
    return _requestDataHandler;
  }

  @override
  Future<Result> handleCommand(
    Command command,
    WidgetController prober,
    CreateFinderFactory finderFactory,
  ) {
    final String kind = command.kind;
    if (_commandExtensions.containsKey(kind)) {
      return _commandExtensions[kind]!.call(command, prober, finderFactory, this);
    }

    return super.handleCommand(command, prober, finderFactory);
  }
}
