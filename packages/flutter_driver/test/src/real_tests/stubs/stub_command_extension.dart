// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/driver_extension.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:flutter_test/flutter_test.dart';

import 'stub_command.dart';

class StubNestedCommandExtension extends CommandExtension {
  @override
  String get commandKind => 'StubNestedCommand';

  @override
  Future<Result> call(
    Command command,
    WidgetController prober,
    CreateFinderFactory finderFactory,
    CommandHandlerFactory handlerFactory,
  ) async {
    final StubNestedCommand stubCommand = command as StubNestedCommand;
    handlerFactory.waitForElement(finderFactory.createFinder(stubCommand.finder));
    for (int index = 0; index < stubCommand.times; index++) {
      await handlerFactory.handleCommand(Tap(stubCommand.finder), prober, finderFactory);
    }
    return const StubCommandResult('stub response');
  }

  @override
  Command deserialize(
    Map<String, String> params,
    DeserializeFinderFactory finderFactory,
    DeserializeCommandFactory commandFactory,
  ) {
    return StubNestedCommand.deserialize(params, finderFactory);
  }
}

class StubProberCommandExtension extends CommandExtension {
  @override
  String get commandKind => 'StubProberCommand';

  @override
  Future<Result> call(
    Command command,
    WidgetController prober,
    CreateFinderFactory finderFactory,
    CommandHandlerFactory handlerFactory,
  ) async {
    final StubProberCommand stubCommand = command as StubProberCommand;
    final Finder finder = finderFactory.createFinder(stubCommand.finder);
    handlerFactory.waitForElement(finder);
    for (int index = 0; index < stubCommand.times; index++) {
      await prober.tap(finder);
    }
    return const StubCommandResult('stub response');
  }

  @override
  Command deserialize(
    Map<String, String> params,
    DeserializeFinderFactory finderFactory,
    DeserializeCommandFactory commandFactory,
  ) {
    return StubProberCommand.deserialize(params, finderFactory);
  }
}
