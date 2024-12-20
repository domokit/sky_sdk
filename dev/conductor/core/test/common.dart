// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:conductor_core/src/stdio.dart';
import 'package:test/fake.dart';
import 'package:test/test.dart';

export 'package:test/fake.dart';
export 'package:test/test.dart' hide isInstanceOf;
export '../../../../packages/flutter_tools/test/src/fake_process_manager.dart';

Matcher throwsAssertionWith(String messageSubString) {
  return throwsA(
    isA<AssertionError>().having(
      (AssertionError e) => e.toString(),
      'description',
      contains(messageSubString),
    ),
  );
}

Matcher throwsExceptionWith(String messageSubString) {
  return throwsA(
    isA<Exception>().having(
      (Exception e) => e.toString(),
      'description',
      contains(messageSubString),
    ),
  );
}

class TestStdio extends Stdio {
  TestStdio({this.verbose = false, List<String>? stdin}) : stdin = stdin ?? <String>[];

  String get error => logs.where((String log) => log.startsWith(r'[error] ')).join('\n');

  String get stdout => logs
      .where((String log) {
        return log.startsWith(r'[status] ') ||
            log.startsWith(r'[trace] ') ||
            log.startsWith(r'[write] ');
      })
      .join('\n');

  final bool verbose;
  final List<String> stdin;

  @override
  String readLineSync() {
    if (stdin.isEmpty) {
      throw Exception('Unexpected call to readLineSync! Last stdout was ${logs.last}');
    }
    return stdin.removeAt(0);
  }
}

class FakeArgResults extends Fake implements ArgResults {
  FakeArgResults({
    required String? level,
    required String candidateBranch,
    String remote = 'upstream',
    bool justPrint = false,
    bool autoApprove = true, // so we don't have to mock stdin
    bool help = false,
    bool force = false,
    bool skipTagging = false,
  }) : _parsedArgs = <String, dynamic>{
         'increment': level,
         'candidate-branch': candidateBranch,
         'remote': remote,
         'just-print': justPrint,
         'yes': autoApprove,
         'help': help,
         'force': force,
         'skip-tagging': skipTagging,
       };

  @override
  String? name;

  @override
  ArgResults? command;

  @override
  final List<String> rest = <String>[];

  final Map<String, dynamic> _parsedArgs;

  @override
  dynamic operator [](String name) {
    return _parsedArgs[name];
  }
}
