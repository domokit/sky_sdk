// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';

class StubNestedCommand extends CommandWithTarget {
  StubNestedCommand(super.finder, this.times, {super.timeout});

  StubNestedCommand.deserialize(super.json, super.finderFactory)
    : times = int.parse(json['times']!),
      super.deserialize();

  @override
  Map<String, String> serialize() {
    return super.serialize()..addAll(<String, String>{'times': '$times'});
  }

  @override
  String get kind => 'StubNestedCommand';

  final int times;
}

class StubProberCommand extends CommandWithTarget {
  StubProberCommand(super.finder, this.times, {super.timeout});

  StubProberCommand.deserialize(super.json, super.finderFactory)
    : times = int.parse(json['times']!),
      super.deserialize();

  @override
  Map<String, String> serialize() {
    return super.serialize()..addAll(<String, String>{'times': '$times'});
  }

  @override
  String get kind => 'StubProberCommand';

  final int times;
}

class StubCommandResult extends Result {
  const StubCommandResult(this.resultParam);

  final String resultParam;

  @override
  Map<String, dynamic> toJson() {
    return <String, dynamic>{'resultParam': resultParam};
  }
}
