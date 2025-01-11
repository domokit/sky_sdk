import 'dart:io' as io;

import 'package:path/path.dart' as p;

/// Returns what `ImpellerBackend` was requested in the current application.
///
/// This is a high confidence signal that the application is running the
/// requested backend, but it is not definitive; a test harness might inspect
/// the output of the `flutter` CLI to be more certain.
///
/// See also:
/// - <https://docs.flutter.dev/perf/impeller#android>
/// - <https://github.com/flutter/flutter/blob/master/engine/src/flutter/impeller/README.md>
Future<String> get requestedImpellerBackend async {
  final io.File androidManifestXml = io.File(
    p.join('android', 'app', 'src', 'main', 'AndroidManifest.xml'),
  );
  final String contents = await androidManifestXml.readAsString();
  final String backend = _findImpellerBackend.firstMatch(contents)!.group(1)!;
  return backend;
}

final RegExp _findImpellerBackend = RegExp(r'ImpellerBackend"\sandroid:value="(.*)"');
