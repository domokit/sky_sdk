// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert' show json;
import 'dart:js_interop';
import 'dart:math' as math;

import 'package:web/web.dart' as web;

import 'src/web/bench_build_image.dart';
import 'src/web/bench_build_material_checkbox.dart';
import 'src/web/bench_card_infinite_scroll.dart';
import 'src/web/bench_child_layers.dart';
import 'src/web/bench_clipped_out_pictures.dart';
import 'src/web/bench_default_target_platform.dart';
import 'src/web/bench_draw_rect.dart';
import 'src/web/bench_dynamic_clip_on_static_picture.dart';
import 'src/web/bench_harness.dart';
import 'src/web/bench_image_decoding.dart';
import 'src/web/bench_material_3.dart';
import 'src/web/bench_material_3_semantics.dart';
import 'src/web/bench_mouse_region_grid_hover.dart';
import 'src/web/bench_mouse_region_grid_scroll.dart';
import 'src/web/bench_mouse_region_mixed_grid_hover.dart';
import 'src/web/bench_pageview_scroll_linethrough.dart';
import 'src/web/bench_paths.dart';
import 'src/web/bench_picture_recording.dart';
import 'src/web/bench_platform_view_infinite_scroll.dart';
import 'src/web/bench_simple_lazy_text_scroll.dart';
import 'src/web/bench_text_layout.dart';
import 'src/web/bench_text_out_of_picture_bounds.dart';
import 'src/web/bench_wrapbox_scroll.dart';
import 'src/web/recorder.dart';

typedef RecorderFactory = Recorder Function();

/// List of all benchmarks that run in the devicelab.
///
/// When adding a new benchmark, add it to this map. Make sure that the name
/// of your benchmark is unique.
final Map<String, RecorderFactory> benchmarks = <String, RecorderFactory>{
  // Benchmarks the overhead of the benchmark harness itself.
  BenchRawRecorder.benchmarkName: () => BenchRawRecorder(),
  BenchWidgetRecorder.benchmarkName: () => BenchWidgetRecorder(),
  BenchWidgetBuildRecorder.benchmarkName: () => BenchWidgetBuildRecorder(),
  BenchSceneBuilderRecorder.benchmarkName: () => BenchSceneBuilderRecorder(),

  // Benchmarks that run in all renderers.
  BenchDefaultTargetPlatform.benchmarkName: () => BenchDefaultTargetPlatform(),
  BenchBuildImage.benchmarkName: () => BenchBuildImage(),
  BenchCardInfiniteScroll.benchmarkName: () => BenchCardInfiniteScroll.forward(),
  BenchCardInfiniteScroll.benchmarkNameBackward: () => BenchCardInfiniteScroll.backward(),
  BenchClippedOutPictures.benchmarkName: () => BenchClippedOutPictures(),
  BenchDrawRect.benchmarkName: () => BenchDrawRect.staticPaint(),
  BenchDrawRect.variablePaintBenchmarkName: () => BenchDrawRect.variablePaint(),
  BenchPathRecording.benchmarkName: () => BenchPathRecording(),
  BenchTextOutOfPictureBounds.benchmarkName: () => BenchTextOutOfPictureBounds(),
  BenchSimpleLazyTextScroll.benchmarkName: () => BenchSimpleLazyTextScroll(),
  BenchBuildMaterialCheckbox.benchmarkName: () => BenchBuildMaterialCheckbox(),
  BenchDynamicClipOnStaticPicture.benchmarkName: () => BenchDynamicClipOnStaticPicture(),
  BenchPageViewScrollLineThrough.benchmarkName: () => BenchPageViewScrollLineThrough(),
  BenchPictureRecording.benchmarkName: () => BenchPictureRecording(),
  BenchUpdateManyChildLayers.benchmarkName: () => BenchUpdateManyChildLayers(),
  BenchMouseRegionGridScroll.benchmarkName: () => BenchMouseRegionGridScroll(),
  BenchMouseRegionGridHover.benchmarkName: () => BenchMouseRegionGridHover(),
  BenchMouseRegionMixedGridHover.benchmarkName: () => BenchMouseRegionMixedGridHover(),
  BenchWrapBoxScroll.benchmarkName: () => BenchWrapBoxScroll(),
  BenchPlatformViewInfiniteScroll.benchmarkName: () => BenchPlatformViewInfiniteScroll.forward(),
  BenchPlatformViewInfiniteScroll.benchmarkNameBackward:
      () => BenchPlatformViewInfiniteScroll.backward(),
  BenchMaterial3Components.benchmarkName: () => BenchMaterial3Components(),
  BenchMaterial3Semantics.benchmarkName: () => BenchMaterial3Semantics(),
  BenchMaterial3ScrollSemantics.benchmarkName: () => BenchMaterial3ScrollSemantics(),

  BenchTextLayout.benchmarkName: () => BenchTextLayout(),
  BenchBuildColorsGrid.benchmarkName: () => BenchBuildColorsGrid(),
  BenchTextCachedLayout.benchmarkName: () => BenchTextCachedLayout(),

  BenchImageDecoding.benchmarkName: () => BenchImageDecoding(),
};

final LocalBenchmarkServerClient _client = LocalBenchmarkServerClient();

Future<void> main() async {
  // Check if the benchmark server wants us to run a specific benchmark.
  final String nextBenchmark = await _client.requestNextBenchmark();

  if (nextBenchmark == LocalBenchmarkServerClient.kManualFallback) {
    _fallbackToManual('The server did not tell us which benchmark to run next.');
    return;
  }

  await _runBenchmark(nextBenchmark);
  web.window.location.reload();
}

Future<void> _runBenchmark(String benchmarkName) async {
  final RecorderFactory? recorderFactory = benchmarks[benchmarkName];

  if (recorderFactory == null) {
    _fallbackToManual('Benchmark $benchmarkName not found.');
    return;
  }

  await runZoned<Future<void>>(
    () async {
      final Recorder recorder = recorderFactory();
      final Runner runner =
          recorder.isTracingEnabled && !_client.isInManualMode
              ? Runner(
                recorder: recorder,
                setUpAllDidRun: () => _client.startPerformanceTracing(benchmarkName),
                tearDownAllWillRun: _client.stopPerformanceTracing,
              )
              : Runner(recorder: recorder);

      final Profile profile = await runner.run();
      if (!_client.isInManualMode) {
        await _client.sendProfileData(profile);
      } else {
        _printResultsToScreen(profile);
        print(profile);
      }
    },
    zoneSpecification: ZoneSpecification(
      print: (Zone self, ZoneDelegate parent, Zone zone, String line) async {
        if (_client.isInManualMode) {
          parent.print(zone, '[$benchmarkName] $line');
        } else {
          await _client.printToConsole(line);
        }
      },
      handleUncaughtError: (
        Zone self,
        ZoneDelegate parent,
        Zone zone,
        Object error,
        StackTrace stackTrace,
      ) async {
        if (_client.isInManualMode) {
          parent.print(zone, '[$benchmarkName] $error, $stackTrace');
          parent.handleUncaughtError(zone, error, stackTrace);
        } else {
          await _client.reportError(error, stackTrace);
        }
      },
    ),
  );
}

extension WebHTMLElementExtension on web.HTMLElement {
  void appendHtml(String html) {
    final web.HTMLDivElement div = web.document.createElement('div') as web.HTMLDivElement;
    div.innerHTML = html.toJS;
    final web.DocumentFragment fragment = web.document.createDocumentFragment();
    fragment.append(div as JSAny);
    web.document.adoptNode(fragment);
    append(fragment as JSAny);
  }
}

void _fallbackToManual(String error) {
  web.document.body!.appendHtml('''
    <div id="manual-panel">
      <h3>$error</h3>

      <p>Choose one of the following benchmarks:</p>

      <!-- Absolutely position it so it receives the clicks and not the glasspane -->
      <ul style="position: absolute">
        ${benchmarks.keys.map((String name) => '<li><button id="$name">$name</button></li>').join('\n')}
      </ul>
    </div>
  ''');

  for (final String benchmarkName in benchmarks.keys) {
    final web.Element button = web.document.querySelector('#$benchmarkName')!;
    button.addEventListener(
      'click',
      (JSObject _) {
        final web.Element? manualPanel = web.document.querySelector('#manual-panel');
        manualPanel?.remove();
        _runBenchmark(benchmarkName);
      }.toJS,
    );
  }
}

/// Visualizes results on the Web page for manual inspection.
void _printResultsToScreen(Profile profile) {
  web.document.body!.remove();
  web.document.body = web.document.createElement('body') as web.HTMLBodyElement;
  web.document.body!.appendHtml('<h2>${profile.name}</h2>');

  profile.scoreData.forEach((String scoreKey, Timeseries timeseries) {
    web.document.body!.appendHtml('<h2>$scoreKey</h2>');
    web.document.body!.appendHtml('<pre>${timeseries.computeStats()}</pre>');
    web.document.body!.append(TimeseriesVisualization(timeseries).render() as JSAny);
  });
}

/// Draws timeseries data and statistics on a canvas.
class TimeseriesVisualization {
  TimeseriesVisualization(this._timeseries) {
    _stats = _timeseries.computeStats();
    _canvas = web.document.createElement('canvas') as web.HTMLCanvasElement;
    _screenWidth = web.window.screen.width;
    _canvas.width = _screenWidth;
    _canvas.height = (_kCanvasHeight * web.window.devicePixelRatio).round();
    _canvas.style
      ..setProperty('width', '100%')
      ..setProperty('height', '${_kCanvasHeight}px')
      ..setProperty('outline', '1px solid green');
    _ctx = _canvas.getContext('2d')! as web.CanvasRenderingContext2D;

    // The amount of vertical space available on the chart. Because some
    // outliers can be huge they can dwarf all the useful values. So we
    // limit it to 1.5 x the biggest non-outlier.
    _maxValueChartRange =
        1.5 *
        _stats.samples
            .where((AnnotatedSample sample) => !sample.isOutlier)
            .map<double>((AnnotatedSample sample) => sample.magnitude)
            .fold<double>(0, math.max);
  }

  static const double _kCanvasHeight = 200;

  final Timeseries _timeseries;
  late TimeseriesStats _stats;
  late web.HTMLCanvasElement _canvas;
  late web.CanvasRenderingContext2D _ctx;
  late int _screenWidth;

  // Used to normalize benchmark values to chart height.
  late double _maxValueChartRange;

  /// Converts a sample value to vertical canvas coordinates.
  ///
  /// This does not work for horizontal coordinates.
  double _normalized(double value) {
    return _kCanvasHeight * value / _maxValueChartRange;
  }

  /// A utility for drawing lines.
  void drawLine(num x1, num y1, num x2, num y2) {
    _ctx.beginPath();
    _ctx.moveTo(x1.toDouble(), y1.toDouble());
    _ctx.lineTo(x2.toDouble(), y2.toDouble());
    _ctx.stroke();
  }

  /// Renders the timeseries into a `<canvas>` and returns the canvas element.
  web.HTMLCanvasElement render() {
    _ctx.translate(0, _kCanvasHeight * web.window.devicePixelRatio);
    _ctx.scale(1, -web.window.devicePixelRatio);

    final double barWidth = _screenWidth / _stats.samples.length;
    double xOffset = 0;
    for (int i = 0; i < _stats.samples.length; i++) {
      final AnnotatedSample sample = _stats.samples[i];

      if (sample.isWarmUpValue) {
        // Put gray background behind warm-up samples.
        _ctx.fillStyle = 'rgba(200,200,200,1)'.toJS;
        _ctx.fillRect(xOffset, 0, barWidth, _normalized(_maxValueChartRange));
      }

      if (sample.magnitude > _maxValueChartRange) {
        // The sample value is so big it doesn't fit on the chart. Paint it purple.
        _ctx.fillStyle = 'rgba(100,50,100,0.8)'.toJS;
      } else if (sample.isOutlier) {
        // The sample is an outlier, color it light red.
        _ctx.fillStyle = 'rgba(255,50,50,0.6)'.toJS;
      } else {
        // A non-outlier sample, color it light blue.
        _ctx.fillStyle = 'rgba(50,50,255,0.6)'.toJS;
      }

      _ctx.fillRect(xOffset, 0, barWidth - 1, _normalized(sample.magnitude));
      xOffset += barWidth;
    }

    // Draw a horizontal solid line corresponding to the average.
    _ctx.lineWidth = 1;
    drawLine(0, _normalized(_stats.average), _screenWidth, _normalized(_stats.average));

    // Draw a horizontal dashed line corresponding to the outlier cut off.
    _ctx.setLineDash(<JSNumber>[5.toJS, 5.toJS].toJS);
    drawLine(0, _normalized(_stats.outlierCutOff), _screenWidth, _normalized(_stats.outlierCutOff));

    // Draw a light red band that shows the noise (1 stddev in each direction).
    _ctx.fillStyle = 'rgba(255,50,50,0.3)'.toJS;
    _ctx.fillRect(
      0,
      _normalized(_stats.average * (1 - _stats.noise)),
      _screenWidth.toDouble(),
      _normalized(2 * _stats.average * _stats.noise),
    );

    return _canvas;
  }
}

/// Implements the client REST API for the local benchmark server.
///
/// The local server is optional. If it is not available the benchmark UI must
/// implement a manual fallback. This allows debugging benchmarks using plain
/// `flutter run`.
class LocalBenchmarkServerClient {
  /// This value is returned by [requestNextBenchmark].
  static const String kManualFallback = '__manual_fallback__';

  /// Whether we fell back to manual mode.
  ///
  /// This happens when you run benchmarks using plain `flutter run` rather than
  /// devicelab test harness. The test harness spins up a special server that
  /// provides API for automatically picking the next benchmark to run.
  bool isInManualMode = false;

  /// Asks the local server for the name of the next benchmark to run.
  ///
  /// Returns [kManualFallback] if local server is not available (uses 404 as a
  /// signal).
  Future<String> requestNextBenchmark() async {
    final web.XMLHttpRequest request = await _requestXhr(
      '/next-benchmark',
      method: 'POST',
      mimeType: 'application/json',
      sendData: json.encode(benchmarks.keys.toList()),
    );

    // 404 is expected in the following cases:
    // - The benchmark is ran using plain `flutter run`, which does not provide "next-benchmark" handler.
    // - We ran all benchmarks and the benchmark is telling us there are no more benchmarks to run.
    if (request.status != 200) {
      isInManualMode = true;
      return kManualFallback;
    }

    isInManualMode = false;
    return request.responseText;
  }

  void _checkNotManualMode() {
    if (isInManualMode) {
      throw StateError('Operation not supported in manual fallback mode.');
    }
  }

  /// Asks the local server to begin tracing performance.
  ///
  /// This uses the chrome://tracing tracer, which is not available from within
  /// the page itself, and therefore must be controlled from outside using the
  /// DevTools Protocol.
  Future<void> startPerformanceTracing(String benchmarkName) async {
    _checkNotManualMode();
    await _requestXhr(
      '/start-performance-tracing?label=$benchmarkName',
      method: 'POST',
      mimeType: 'application/json',
    );
  }

  /// Stops the performance tracing session started by [startPerformanceTracing].
  Future<void> stopPerformanceTracing() async {
    _checkNotManualMode();
    await _requestXhr('/stop-performance-tracing', method: 'POST', mimeType: 'application/json');
  }

  /// Sends the profile data collected by the benchmark to the local benchmark
  /// server.
  Future<void> sendProfileData(Profile profile) async {
    _checkNotManualMode();
    final web.XMLHttpRequest request = await _requestXhr(
      '/profile-data',
      method: 'POST',
      mimeType: 'application/json',
      sendData: json.encode(profile.toJson()),
    );
    if (request.status != 200) {
      throw Exception(
        'Failed to report profile data to benchmark server. '
        'The server responded with status code ${request.status}.',
      );
    }
  }

  /// Reports an error to the benchmark server.
  ///
  /// The server will halt the devicelab task and log the error.
  Future<void> reportError(dynamic error, StackTrace stackTrace) async {
    _checkNotManualMode();
    await _requestXhr(
      '/on-error',
      method: 'POST',
      mimeType: 'application/json',
      sendData: json.encode(<String, dynamic>{'error': '$error', 'stackTrace': '$stackTrace'}),
    );
  }

  /// Reports a message about the demo to the benchmark server.
  Future<void> printToConsole(String report) async {
    _checkNotManualMode();
    await _requestXhr(
      '/print-to-console',
      method: 'POST',
      mimeType: 'text/plain',
      sendData: report,
    );
  }

  /// This is the same as calling [html.HttpRequest.request] but it doesn't
  /// crash on 404, which we use to detect `flutter run`.
  Future<web.XMLHttpRequest> _requestXhr(
    String url, {
    String? method,
    bool? withCredentials,
    String? responseType,
    String? mimeType,
    Map<String, String>? requestHeaders,
    dynamic sendData,
  }) {
    final Completer<web.XMLHttpRequest> completer = Completer<web.XMLHttpRequest>();
    final web.XMLHttpRequest xhr = web.XMLHttpRequest();

    method ??= 'GET';
    xhr.open(method, url, true);

    if (withCredentials != null) {
      xhr.withCredentials = withCredentials;
    }

    if (responseType != null) {
      xhr.responseType = responseType;
    }

    if (mimeType != null) {
      xhr.overrideMimeType(mimeType);
    }

    if (requestHeaders != null) {
      requestHeaders.forEach((String header, String value) {
        xhr.setRequestHeader(header, value);
      });
    }

    xhr.addEventListener(
      'load',
      (web.ProgressEvent e) {
        completer.complete(xhr);
      }.toJS,
    );

    xhr.addEventListener(
      'error',
      (JSObject error) {
        return completer.completeError(error);
      }.toJS,
    );

    if (sendData != null) {
      xhr.send((sendData as Object?).jsify());
    } else {
      xhr.send();
    }

    return completer.future;
  }
}
