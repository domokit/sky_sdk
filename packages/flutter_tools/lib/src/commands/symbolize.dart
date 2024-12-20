// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:native_stack_traces/native_stack_traces.dart';

import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../convert.dart';
import '../runner/flutter_command.dart';

const int rootLoadingUnitId = 1;

/// Support for symbolizing a Dart stack trace.
///
/// This command accepts either paths to an input file containing the
/// stack trace and an output file for the symbolizing trace to be
/// written, or it accepts a stack trace over stdin and outputs it
/// over stdout.
class SymbolizeCommand extends FlutterCommand {
  SymbolizeCommand({
    required Stdio stdio,
    required FileSystem fileSystem,
    DwarfSymbolizationService dwarfSymbolizationService = const DwarfSymbolizationService(),
  }) : _stdio = stdio,
       _fileSystem = fileSystem,
       _dwarfSymbolizationService = dwarfSymbolizationService {
    argParser.addOption(
      'debug-info',
      abbr: 'd',
      valueHelp: '/out/android/app.arm64.symbols',
      help: 'A path to the symbols file generated with "--split-debug-info".',
    );
    argParser.addMultiOption(
      'unit-id-debug-info',
      abbr: 'u',
      valueHelp: '2:/out/android/app.arm64.symbols-2.part.so',
      help:
          'A loading unit id and the path to the symbols file for that'
          ' unit generated with "--split-debug-info".',
    );
    argParser.addOption(
      'input',
      abbr: 'i',
      valueHelp: '/crashes/stack_trace.err',
      help: 'A file path containing a Dart stack trace.',
    );
    argParser.addOption(
      'output',
      abbr: 'o',
      help: 'A file path for a symbolized stack trace to be written to.',
    );
  }

  final Stdio _stdio;
  final FileSystem _fileSystem;
  final DwarfSymbolizationService _dwarfSymbolizationService;

  @override
  String get description => 'Symbolize a stack trace from an AOT-compiled Flutter app.';

  @override
  String get name => 'symbolize';

  @override
  final String category = FlutterCommandCategory.tools;

  @override
  bool get shouldUpdateCache => false;

  File _handleDSYM(String fileName) {
    final FileSystemEntityType type = _fileSystem.typeSync(fileName);
    final bool isDSYM = fileName.endsWith('.dSYM');
    if (type == FileSystemEntityType.notFound) {
      throw FileNotFoundException(fileName);
    }
    if (type == FileSystemEntityType.directory) {
      if (!isDSYM) {
        throw StateError('$fileName is a directory, not a file');
      }
      final Directory dwarfDir = _fileSystem
          .directory(fileName)
          .childDirectory('Contents')
          .childDirectory('Resources')
          .childDirectory('DWARF');
      // The DWARF directory inside the .dSYM contains a single MachO file.
      return dwarfDir.listSync().single as File;
    }
    if (isDSYM) {
      throw StateError('$fileName is not a dSYM package directory');
    }
    return _fileSystem.file(fileName);
  }

  Map<int, File> _unitDebugInfoPathMap() {
    final Map<int, File> map = <int, File>{};
    final String? rootInfo = stringArg('debug-info');
    if (rootInfo != null) {
      map[rootLoadingUnitId] = _handleDSYM(rootInfo);
    }
    for (final String arg in stringsArg('unit-id-debug-info')) {
      final int separatorIndex = arg.indexOf(':');
      final String unitIdString = arg.substring(0, separatorIndex);
      final int unitId = int.parse(unitIdString);
      final String unitDebugPath = arg.substring(separatorIndex + 1);
      if (map.containsKey(unitId) && map[unitId]!.path != unitDebugPath) {
        throw StateError(
          'Different paths were given for the same loading unit'
          ' $unitId: "${map[unitId]!.path}" and "$unitDebugPath".',
        );
      }
      map[unitId] = _handleDSYM(unitDebugPath);
    }
    return map;
  }

  @override
  Future<void> validateCommand() async {
    if (argResults?.wasParsed('debug-info') != true &&
        argResults?.wasParsed('unit-id-debug-info') != true) {
      throwToolExit(
        'Either "--debug-info" or "--unit-id-debug-info" is required to symbolize stack traces.',
      );
    }
    for (final String arg in stringsArg('unit-id-debug-info')) {
      final int separatorIndex = arg.indexOf(':');
      if (separatorIndex == -1) {
        throwToolExit(
          'The argument to "--unit-id-debug-info" must contain a unit ID and path,'
          ' separated by ":": "$arg".',
        );
      }
      final String unitIdString = arg.substring(0, separatorIndex);
      final int? unitId = int.tryParse(unitIdString);
      if (unitId == null) {
        throwToolExit(
          'The argument to "--unit-id-debug-info" must begin with'
          ' a unit ID: "$unitIdString" is not an integer.',
        );
      }
    }
    late final Map<int, File> map;
    try {
      map = _unitDebugInfoPathMap();
    } on Object catch (e) {
      throwToolExit(e.toString());
    }
    if (!map.containsKey(rootLoadingUnitId)) {
      throwToolExit(
        'Missing debug info for the root loading unit'
        ' (id $rootLoadingUnitId).',
      );
    }
    if ((argResults?.wasParsed('input') ?? false) &&
        !await _fileSystem.isFile(stringArg('input')!)) {
      throwToolExit('${stringArg('input')} does not exist.');
    }
    return super.validateCommand();
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    // Configure output to either specified file or stdout.
    late final IOSink output;
    if (argResults?.wasParsed('output') ?? false) {
      final File outputFile = _fileSystem.file(stringArg('output'));
      if (!outputFile.parent.existsSync()) {
        outputFile.parent.createSync(recursive: true);
      }
      output = outputFile.openWrite();
    } else {
      final StreamController<List<int>> outputController = StreamController<List<int>>();
      outputController.stream.transform(utf8.decoder).listen(_stdio.stdoutWrite);
      output = IOSink(outputController);
    }

    // Configure input from either specified file or stdin.
    final Stream<List<int>> input =
        (argResults?.wasParsed('input') ?? false)
            ? _fileSystem.file(stringArg('input')).openRead()
            : _stdio.stdin;

    final Map<int, Uint8List> unitSymbols = <int, Uint8List>{
      for (final MapEntry<int, File> entry in _unitDebugInfoPathMap().entries)
        entry.key: entry.value.readAsBytesSync(),
    };

    await _dwarfSymbolizationService.decodeWithUnits(
      input: input,
      output: output,
      unitSymbols: unitSymbols,
    );

    return FlutterCommandResult.success();
  }
}

typedef SymbolsTransformer = StreamTransformer<String, String> Function(Uint8List);
typedef UnitSymbolsTransformer = StreamTransformer<String, String> Function(Map<int, Uint8List>);

StreamTransformer<String, String> _defaultTransformer(Uint8List symbols) {
  return _defaultUnitsTransformer(<int, Uint8List>{rootLoadingUnitId: symbols});
}

StreamTransformer<String, String> _defaultUnitsTransformer(Map<int, Uint8List> unitSymbols) {
  final Map<int, Dwarf> map = <int, Dwarf>{};
  for (final int unitId in unitSymbols.keys) {
    final Uint8List symbols = unitSymbols[unitId]!;
    final Dwarf? dwarf = Dwarf.fromBytes(symbols);
    if (dwarf == null) {
      throwToolExit('Failed to decode symbols file for loading unit $unitId');
    }
    map[unitId] = dwarf;
  }
  if (!map.containsKey(rootLoadingUnitId)) {
    throwToolExit('Missing symbols file for root loading unit (id $rootLoadingUnitId)');
  }
  return DwarfStackTraceDecoder(
    map[rootLoadingUnitId]!,
    includeInternalFrames: true,
    dwarfByUnitId: map,
  );
}

// A no-op transformer for `DwarfSymbolizationService.test`
StreamTransformer<String, String> _testUnitsTransformer(Map<int, Uint8List> buffer) {
  return StreamTransformer<String, String>.fromHandlers(
    handleData: (String data, EventSink<String> sink) {
      sink.add(data);
    },
    handleDone: (EventSink<String> sink) {
      sink.close();
    },
    handleError: (Object error, StackTrace stackTrace, EventSink<String> sink) {
      sink.addError(error, stackTrace);
    },
  );
}

/// A service which decodes stack traces from Dart applications.
class DwarfSymbolizationService {
  const DwarfSymbolizationService({SymbolsTransformer symbolsTransformer = _defaultTransformer})
    : _transformer = symbolsTransformer,
      _unitsTransformer = _defaultUnitsTransformer;

  const DwarfSymbolizationService.withUnits({
    UnitSymbolsTransformer unitSymbolsTransformer = _defaultUnitsTransformer,
  }) : _transformer = null,
       _unitsTransformer = unitSymbolsTransformer;

  /// Create a DwarfSymbolizationService with a no-op transformer for testing.
  @visibleForTesting
  factory DwarfSymbolizationService.test() {
    return const DwarfSymbolizationService.withUnits(unitSymbolsTransformer: _testUnitsTransformer);
  }

  final SymbolsTransformer? _transformer;
  final UnitSymbolsTransformer _unitsTransformer;

  /// Decode a stack trace from [input] and place the results in [output].
  ///
  /// Requires [symbols] to be a buffer created from the `--split-debug-info`
  /// command line flag.
  ///
  /// Throws a [ToolExit] if the symbols cannot be parsed or the stack trace
  /// cannot be decoded.
  Future<void> decode({
    required Stream<List<int>> input,
    required IOSink output,
    required Uint8List symbols,
  }) async {
    await decodeWithUnits(
      input: input,
      output: output,
      unitSymbols: <int, Uint8List>{rootLoadingUnitId: symbols},
    );
  }

  /// Decode a stack trace from [input] and place the results in [output].
  ///
  /// Requires [unitSymbols] to map integer unit IDs to buffers created from
  /// the `--split-debug-info` command line flag.
  ///
  /// Throws a [ToolExit] if the symbols cannot be parsed or the stack trace
  /// cannot be decoded.
  Future<void> decodeWithUnits({
    required Stream<List<int>> input,
    required IOSink output,
    required Map<int, Uint8List> unitSymbols,
  }) async {
    final UnitSymbolsTransformer unitSymbolsTransformer =
        _transformer != null
            ? ((Map<int, Uint8List> m) => _transformer(m[rootLoadingUnitId]!))
            : _unitsTransformer;
    final Completer<void> onDone = Completer<void>();
    StreamSubscription<void>? subscription;
    subscription = input
        .cast<List<int>>()
        .transform(const Utf8Decoder())
        .transform(const LineSplitter())
        .transform(unitSymbolsTransformer(unitSymbols))
        .listen(
          (String line) {
            try {
              output.writeln(line);
            } on Exception catch (e, s) {
              subscription?.cancel().whenComplete(() {
                if (!onDone.isCompleted) {
                  onDone.completeError(e, s);
                }
              });
            }
          },
          onDone: onDone.complete,
          onError: onDone.completeError,
        );

    try {
      await onDone.future;
      await output.close();
    } on Exception catch (err) {
      throwToolExit('Failed to symbolize stack trace:\n $err');
    }
  }
}
