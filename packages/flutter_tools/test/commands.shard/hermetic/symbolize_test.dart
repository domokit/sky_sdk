// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/common.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/symbolize.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  late MemoryFileSystem fileSystem;
  late FakeStdio stdio;

  setUpAll(() {
    Cache.disableLocking();
  });

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    stdio = FakeStdio();
  });

  testUsingContext('Regression test for type error in codec', () async {
    final DwarfSymbolizationService symbolizationService = DwarfSymbolizationService.test();
    final StreamController<List<int>> output = StreamController<List<int>>();

    unawaited(
      symbolizationService.decode(
        input: Stream<Uint8List>.fromIterable(<Uint8List>[
          const Utf8Encoder().convert('Hello, World\n'),
        ]),
        symbols: Uint8List(0),
        output: IOSink(output.sink),
      ),
    );

    await expectLater(
      output.stream.transform(utf8.decoder).transform(const LineSplitter()),
      emits('Hello, World'),
    );
  });

  testUsingContext(
    'symbolize exits when --debug-info and --unit-id-debug-info arguments are missing',
    () async {
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      final Future<void> result = createTestCommandRunner(command).run(const <String>['symbolize']);

      expect(
        result,
        throwsToolExit(
          message:
              'Either "--debug-info" or "--unit-id-debug-info" is required to symbolize stack traces.',
        ),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --debug-info dwarf file is missing',
    () async {
      const String fileName = 'app.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--debug-info=$fileName']);

      expect(result, throwsToolExit(message: 'File not found: $fileName'));
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --unit-id-debug-info dwarf file is missing',
    () async {
      const String fileName = 'app.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--unit-id-debug-info=$rootLoadingUnitId:$fileName']);

      expect(result, throwsToolExit(message: 'File not found: $fileName'));
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --debug-info dSYM is missing',
    () async {
      const String fileName = 'app.dSYM';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--debug-info=$fileName']);

      expect(result, throwsToolExit(message: 'File not found: $fileName'));
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --unit-id-debug-info dSYM is missing',
    () async {
      const String fileName = 'app.dSYM';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--unit-id-debug-info=$rootLoadingUnitId:$fileName']);

      expect(result, throwsToolExit(message: 'File not found: $fileName'));
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --debug-info dSYM is not a directory',
    () async {
      const String fileName = 'app.dSYM';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName).createSync();
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--debug-info=$fileName']);

      expect(result, throwsToolExit(message: '$fileName is not a dSYM package directory'));
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --unit-id-debug-info dSYM is not a directory',
    () async {
      const String fileName = 'app.dSYM';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName).createSync();
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--unit-id-debug-info=$rootLoadingUnitId:$fileName']);

      expect(result, throwsToolExit(message: '$fileName is not a dSYM package directory'));
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits if --unit-id-debug-info is just given a path',
    () async {
      const String fileName = 'app.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName).createSync();
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--unit-id-debug-info=$fileName']);

      expect(
        result,
        throwsToolExit(
          message:
              'The argument to "--unit-id-debug-info" must contain a unit ID and path,'
              ' separated by ":": "$fileName".',
        ),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits if the unit id for --unit-id-debug-info is not a valid integer',
    () async {
      const String fileName = 'app.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName).createSync();
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--unit-id-debug-info=foo:$fileName']);

      expect(
        result,
        throwsToolExit(
          message:
              'The argument to "--unit-id-debug-info" must begin with'
              ' a unit ID: "foo" is not an integer.',
        ),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when different paths are given for the root loading unit via --debug-info and --unit-id-debug-info',
    () async {
      const String fileName1 = 'app.debug';
      const String fileName2 = 'app2.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName1).createSync();
      fileSystem.file(fileName2).createSync();
      final Future<void> result = createTestCommandRunner(command).run(const <String>[
        'symbolize',
        '--debug-info=$fileName1',
        '--unit-id-debug-info=$rootLoadingUnitId:$fileName2',
      ]);

      expect(
        result,
        throwsToolExit(
          message:
              'Different paths were given for'
              ' the same loading unit $rootLoadingUnitId: "$fileName1" and'
              ' "$fileName2".',
        ),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when different paths are given for a non-root loading unit via --unit-id-debug-info',
    () async {
      const String fileName1 = 'app.debug';
      const String fileName2 = 'app2.debug';
      const String fileName3 = 'app3.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName1).createSync();
      fileSystem.file(fileName2).createSync();
      fileSystem.file(fileName3).createSync();
      final Future<void> result = createTestCommandRunner(command).run(const <String>[
        'symbolize',
        '--debug-info=$fileName1',
        '--unit-id-debug-info=${rootLoadingUnitId + 1}:$fileName2',
        '--unit-id-debug-info=${rootLoadingUnitId + 1}:$fileName3',
      ]);

      expect(
        result,
        throwsToolExit(
          message:
              'Different paths were given for'
              ' the same loading unit ${rootLoadingUnitId + 1}: "$fileName2" and'
              ' "$fileName3".',
        ),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --input file is missing',
    () async {
      const String fileName = 'app.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName).createSync();
      final Future<void> result = createTestCommandRunner(command).run(const <String>[
        'symbolize',
        '--debug-info=$fileName',
        '--input=foo.stack',
        '--output=results/foo.result',
      ]);

      expect(result, throwsToolExit(message: ''));
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize exits when --debug-info argument is missing and --unit-id-debug-info is not provided for the root loading unit',
    () async {
      const String fileName = 'app.debug';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(fileName).createSync();
      final Future<void> result = createTestCommandRunner(
        command,
      ).run(const <String>['symbolize', '--unit-id-debug-info=${rootLoadingUnitId + 1}:$fileName']);

      expect(
        result,
        throwsToolExit(
          message: 'Missing debug info for the root loading unit (id $rootLoadingUnitId).',
        ),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize succeeds when DwarfSymbolizationService does not throw',
    () async {
      const String debugName = 'app.debug';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      await createTestCommandRunner(command).run(const <String>[
        'symbolize',
        '--debug-info=$debugName',
        '--input=$inputName',
        '--output=$outputPath',
      ]);

      expect(fileSystem.file(outputPath), exists);
      expect(fileSystem.file(outputPath).readAsBytesSync(), <int>[
        104,
        101,
        108,
        108,
        111,
        10,
      ]); // hello
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize succeeds when DwarfSymbolizationService with a single --unit-id-debug-info argument for the root loading unit does not throw',
    () async {
      const String debugName = 'app.debug';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      await createTestCommandRunner(command).run(const <String>[
        'symbolize',
        '--unit-id-debug-info=$rootLoadingUnitId:$debugName',
        '--input=$inputName',
        '--output=$outputPath',
      ]);

      expect(fileSystem.file(outputPath), exists);
      expect(fileSystem.file(outputPath).readAsBytesSync(), <int>[
        104,
        101,
        108,
        108,
        111,
        10,
      ]); // hello
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize succeeds when DwarfSymbolizationService with --debug-info and --unit-id-debug-info arguments does not throw',
    () async {
      const String debugName = 'app.debug';
      const String debugName2 = '$debugName-2.part.so';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(debugName2).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      await createTestCommandRunner(command).run(const <String>[
        'symbolize',
        '--debug-info=$debugName',
        '--unit-id-debug-info=${rootLoadingUnitId + 1}:$debugName2',
        '--input=$inputName',
        '--output=$outputPath',
      ]);

      expect(fileSystem.file(outputPath), exists);
      expect(fileSystem.file(outputPath).readAsBytesSync(), <int>[
        104,
        101,
        108,
        108,
        111,
        10,
      ]); // hello
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize succeeds when DwarfSymbolizationService with multiple --unit-id-debug-info arguments does not throw',
    () async {
      const String debugName = 'app.debug';
      const String debugName2 = '$debugName-2.part.so';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: DwarfSymbolizationService.test(),
      );
      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(debugName2).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      await createTestCommandRunner(command).run(const <String>[
        'symbolize',
        '--unit-id-debug-info=$rootLoadingUnitId:$debugName',
        '--unit-id-debug-info=${rootLoadingUnitId + 1}:$debugName2',
        '--input=$inputName',
        '--output=$outputPath',
      ]);

      expect(fileSystem.file(outputPath), exists);
      expect(fileSystem.file(outputPath).readAsBytesSync(), <int>[
        104,
        101,
        108,
        108,
        111,
        10,
      ]); // hello
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize throws when DwarfSymbolizationService throws',
    () async {
      const String debugName = 'app.debug';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: ThrowingDwarfSymbolizationService(),
      );

      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      expect(
        createTestCommandRunner(command).run(const <String>[
          'symbolize',
          '--debug-info=$debugName',
          '--input=$inputName',
          '--output=$outputPath',
        ]),
        throwsToolExit(message: 'test'),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize throws when DwarfSymbolizationService with a single --unit-id-debug-info argument for the root loading unit throws',
    () async {
      const String debugName = 'app.debug';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: ThrowingDwarfSymbolizationService(),
      );

      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      expect(
        createTestCommandRunner(command).run(const <String>[
          'symbolize',
          '--unit-id-debug-info=$rootLoadingUnitId:$debugName',
          '--input=$inputName',
          '--output=$outputPath',
        ]),
        throwsToolExit(message: 'test'),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize throws when DwarfSymbolizationService with --debug-info and --unit-id-debug-info arguments throws',
    () async {
      const String debugName = 'app.debug';
      const String debugName2 = '$debugName-2.part.so';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: ThrowingDwarfSymbolizationService(),
      );

      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(debugName2).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      expect(
        createTestCommandRunner(command).run(const <String>[
          'symbolize',
          '--debug-info=$debugName',
          '--unit-id-debug-info=${rootLoadingUnitId + 1}:$debugName2',
          '--input=$inputName',
          '--output=$outputPath',
        ]),
        throwsToolExit(message: 'test'),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );

  testUsingContext(
    'symbolize throws when DwarfSymbolizationService with multiple --unit-id-debug-info arguments throws',
    () async {
      const String debugName = 'app.debug';
      const String debugName2 = '$debugName-2.part.so';
      const String inputName = 'foo.stack';
      const String outputPath = 'results/foo.result';
      final SymbolizeCommand command = SymbolizeCommand(
        stdio: stdio,
        fileSystem: fileSystem,
        dwarfSymbolizationService: ThrowingDwarfSymbolizationService(),
      );

      fileSystem.file(debugName).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(debugName2).writeAsBytesSync(<int>[1, 2, 3]);
      fileSystem.file(inputName).writeAsStringSync('hello');

      expect(
        createTestCommandRunner(command).run(const <String>[
          'symbolize',
          '--unit-id-debug-info=$rootLoadingUnitId:$debugName',
          '--unit-id-debug-info=${rootLoadingUnitId + 1}:$debugName2',
          '--input=$inputName',
          '--output=$outputPath',
        ]),
        throwsToolExit(message: 'test'),
      );
    },
    overrides: <Type, Generator>{OutputPreferences: () => OutputPreferences.test()},
  );
}

class ThrowingDwarfSymbolizationService extends Fake implements DwarfSymbolizationService {
  @override
  Future<void> decodeWithUnits({
    required Stream<List<int>> input,
    required IOSink output,
    required Map<int, Uint8List> unitSymbols,
  }) async {
    throwToolExit('test');
  }
}
