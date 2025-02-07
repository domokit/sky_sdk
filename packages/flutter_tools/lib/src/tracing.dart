// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:vm_service/vm_service.dart' as vm_service;

import 'base/common.dart';
import 'base/error_handling_io.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/utils.dart';
import 'convert.dart';
import 'vmservice.dart';

// Names of some of the Timeline events we care about.
const String kFlutterEngineMainEnterEventName = 'FlutterEngineMainEnter';
const String kFrameworkInitEventName = 'Framework initialization';
const String kFirstFrameBuiltEventName = 'Widgets built first useful frame';
const String kFirstFrameRasterizedEventName = 'Rasterized first useful frame';

class Tracing {
  Tracing({required this.vmService, required Logger logger}) : _logger = logger;

  static const String firstUsefulFrameEventName = kFirstFrameRasterizedEventName;

  final FlutterVmService vmService;
  final Logger _logger;

  Future<void> startTracing() async {
    await vmService.setTimelineFlags(<String>['Compiler', 'Dart', 'Embedder', 'GC']);
    await vmService.service.clearVMTimeline();
  }

  /// Stops tracing; optionally wait for first frame.
  Future<Map<String, Object?>> stopTracingAndDownloadTimeline({
    bool awaitFirstFrame = false,
  }) async {
    if (awaitFirstFrame) {
      final Status status = _logger.startProgress(
        'Waiting for application to render first frame...',
      );
      try {
        final Completer<void> whenFirstFrameRendered = Completer<void>();
        try {
          await vmService.service.streamListen(vm_service.EventStreams.kExtension);
        } on vm_service.RPCError {
          // It is safe to ignore this error because we expect an error to be
          // thrown if we're already subscribed.
        }
        final StringBuffer bufferedEvents = StringBuffer();
        void Function(String) handleBufferedEvent = bufferedEvents.writeln;
        vmService.service.onExtensionEvent.listen((vm_service.Event event) {
          handleBufferedEvent('${event.extensionKind}: ${event.extensionData}');
          if (event.extensionKind == 'Flutter.FirstFrame') {
            whenFirstFrameRendered.complete();
          }
        });
        bool done = false;
        final List<FlutterView> views = await vmService.getFlutterViews();
        for (final FlutterView view in views) {
          final String? uiIsolateId = view.uiIsolate?.id;
          if (uiIsolateId != null &&
              await vmService.flutterAlreadyPaintedFirstUsefulFrame(isolateId: uiIsolateId)) {
            done = true;
            break;
          }
        }
        if (!done) {
          final Timer timer = Timer(const Duration(seconds: 10), () async {
            _logger.printStatus('First frame is taking longer than expected...');
            for (final FlutterView view in views) {
              final String? isolateId = view.uiIsolate?.id;
              _logger.printTrace('View ID: ${view.id}');
              if (isolateId == null) {
                _logger.printTrace('No isolate ID associated with the view.');
                continue;
              }
              final vm_service.Isolate? isolate = await vmService.getIsolateOrNull(isolateId);
              if (isolate == null) {
                _logger.printTrace('Isolate $isolateId not found.');
                continue;
              }
              _logger.printTrace('Isolate $isolateId state:');
              final Map<String, Object?> isolateState = isolate.toJson();
              // "libraries" has very long output and is likely unrelated to any first-frame issues.
              isolateState.remove('libraries');
              _logger.printTrace(jsonEncode(isolateState));
            }
            _logger.printTrace('Received VM events:');
            _logger.printTrace(bufferedEvents.toString());
            // Swap to just printing new events instead of buffering.
            handleBufferedEvent = _logger.printTrace;
          });
          await whenFirstFrameRendered.future;
          timer.cancel();
        }
        // The exception is rethrown, so don't catch only Exceptions.
      } catch (exception) {
        status.cancel();
        rethrow;
      }
      status.stop();
    }
    final vm_service.Response? timeline = await vmService.getTimeline();
    await vmService.setTimelineFlags(<String>[]);
    final Map<String, Object?>? timelineJson = timeline?.json;
    if (timelineJson == null) {
      throwToolExit('The device disconnected before the timeline could be retrieved.');
    }
    return timelineJson;
  }
}

/// Download the startup trace information from the given VM Service client and
/// store it to `$output/start_up_info.json`.
Future<void> downloadStartupTrace(
  FlutterVmService vmService, {
  bool awaitFirstFrame = true,
  required Logger logger,
  required Directory output,
}) async {
  final File traceInfoFile = output.childFile('start_up_info.json');

  // Delete old startup data, if any.
  ErrorHandlingFileSystem.deleteIfExists(traceInfoFile);

  // Create "build" directory, if missing.
  if (!traceInfoFile.parent.existsSync()) {
    traceInfoFile.parent.createSync();
  }

  final Tracing tracing = Tracing(vmService: vmService, logger: logger);

  final Map<String, Object?> timeline = await tracing.stopTracingAndDownloadTimeline(
    awaitFirstFrame: awaitFirstFrame,
  );

  final File traceTimelineFile = output.childFile('start_up_timeline.json');
  traceTimelineFile.writeAsStringSync(toPrettyJson(timeline));

  int? extractInstantEventTimestamp(String eventName) {
    final List<Object?>? traceEvents = timeline['traceEvents'] as List<Object?>?;
    if (traceEvents == null) {
      return null;
    }
    final List<Map<String, Object?>> events = List<Map<String, Object?>>.from(traceEvents);
    Map<String, Object?>? matchedEvent;
    for (final Map<String, Object?> event in events) {
      if (event['name'] == eventName) {
        matchedEvent = event;
      }
    }
    return matchedEvent == null ? null : (matchedEvent['ts'] as int?);
  }

  String message = 'No useful metrics were gathered.';

  final int? engineEnterTimestampMicros = extractInstantEventTimestamp(
    kFlutterEngineMainEnterEventName,
  );
  final int? frameworkInitTimestampMicros = extractInstantEventTimestamp(kFrameworkInitEventName);

  if (engineEnterTimestampMicros == null) {
    logger.printTrace('Engine start event is missing in the timeline: $timeline');
    throwToolExit('Engine start event is missing in the timeline. Cannot compute startup time.');
  }

  final Map<String, Object?> traceInfo = <String, Object?>{
    'engineEnterTimestampMicros': engineEnterTimestampMicros,
  };

  if (frameworkInitTimestampMicros != null) {
    final int timeToFrameworkInitMicros = frameworkInitTimestampMicros - engineEnterTimestampMicros;
    traceInfo['timeToFrameworkInitMicros'] = timeToFrameworkInitMicros;
    message = 'Time to framework init: ${timeToFrameworkInitMicros ~/ 1000}ms.';
  }

  if (awaitFirstFrame) {
    final int? firstFrameBuiltTimestampMicros = extractInstantEventTimestamp(
      kFirstFrameBuiltEventName,
    );
    final int? firstFrameRasterizedTimestampMicros = extractInstantEventTimestamp(
      kFirstFrameRasterizedEventName,
    );
    if (firstFrameBuiltTimestampMicros == null || firstFrameRasterizedTimestampMicros == null) {
      logger.printTrace('First frame events are missing in the timeline: $timeline');
      throwToolExit('First frame events are missing in the timeline. Cannot compute startup time.');
    }

    // To keep our old benchmarks valid, we'll preserve the
    // timeToFirstFrameMicros as the firstFrameBuiltTimestampMicros.
    // Additionally, we add timeToFirstFrameRasterizedMicros for a more accurate
    // benchmark.
    traceInfo['timeToFirstFrameRasterizedMicros'] =
        firstFrameRasterizedTimestampMicros - engineEnterTimestampMicros;
    final int timeToFirstFrameMicros = firstFrameBuiltTimestampMicros - engineEnterTimestampMicros;
    traceInfo['timeToFirstFrameMicros'] = timeToFirstFrameMicros;
    message = 'Time to first frame: ${timeToFirstFrameMicros ~/ 1000}ms.';
    if (frameworkInitTimestampMicros != null) {
      traceInfo['timeAfterFrameworkInitMicros'] =
          firstFrameBuiltTimestampMicros - frameworkInitTimestampMicros;
    }
  }

  traceInfoFile.writeAsStringSync(toPrettyJson(traceInfo));

  logger.printStatus(message);
  logger.printStatus('Saved startup trace info in ${traceInfoFile.path}.');
}
