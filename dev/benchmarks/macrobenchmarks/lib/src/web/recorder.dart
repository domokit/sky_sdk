// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:js_interop';
import 'dart:math' as math;
import 'dart:ui';
import 'dart:ui_web' as ui_web;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:meta/meta.dart';
import 'package:web/web.dart' as web;

/// The default number of samples from warm-up iterations.
///
/// This value is used when [Profile.useCustomWarmUp] is set to false.
///
/// The benchmark is warmed up prior to measuring to allow JIT and caches to settle.
const int _kDefaultWarmUpSampleCount = 200;

/// The default number of samples collected to compute benchmark statistics.
///
/// This value is used when [Profile.useCustomWarmUp] is set to false.
const int _kDefaultMeasuredSampleCount = 100;

/// The default total number of samples collected by a benchmark.
///
/// This value is used when [Profile.useCustomWarmUp] is set to false.
const int kDefaultTotalSampleCount = _kDefaultWarmUpSampleCount + _kDefaultMeasuredSampleCount;

/// A benchmark metric that includes frame-related computations prior to
/// submitting layer and picture operations to the underlying renderer, such as
/// CanvasKit. During this phase we compute transforms, clips, and other
/// information needed for rendering.
const String kProfilePrerollFrame = 'preroll_frame';

/// A benchmark metric that includes submitting layer and picture information
/// to the renderer.
const String kProfileApplyFrame = 'apply_frame';

/// Measures the amount of time [action] takes.
///
/// See also:
///
///  * [timeAsyncAction], which measures the time of asynchronous work.
Duration timeAction(VoidCallback action) {
  final Stopwatch stopwatch = Stopwatch()..start();
  action();
  stopwatch.stop();
  return stopwatch.elapsed;
}

/// Measures the amount of time the future returned by [action] takes to complete.
///
/// See also:
///
///  * [timeAction], which measures the time of synchronous work.
Future<Duration> timeAsyncAction(AsyncCallback action) async {
  final Stopwatch stopwatch = Stopwatch()..start();
  await action();
  stopwatch.stop();
  return stopwatch.elapsed;
}

/// A function that performs asynchronous work.
typedef AsyncVoidCallback = Future<void> Function();

/// An [AsyncVoidCallback] that doesn't do anything.
///
/// This is used just so we don't have to deal with null all over the place.
Future<void> _dummyAsyncVoidCallback() async {}

/// Runs the benchmark using the given [recorder].
///
/// Notifies about "set up" and "tear down" events via the [setUpAllDidRun]
/// and [tearDownAllWillRun] callbacks.
@sealed
class Runner {
  /// Creates a runner for the [recorder].
  Runner({
    required this.recorder,
    this.setUpAllDidRun = _dummyAsyncVoidCallback,
    this.tearDownAllWillRun = _dummyAsyncVoidCallback,
  });

  /// The recorder that will run and record the benchmark.
  final Recorder recorder;

  /// Called immediately after [Recorder.setUpAll] future is resolved.
  ///
  /// This is useful, for example, to kick off a profiler or a tracer such that
  /// the "set up" computations are not included in the metrics.
  final AsyncVoidCallback setUpAllDidRun;

  /// Called just before calling [Recorder.tearDownAll].
  ///
  /// This is useful, for example, to stop a profiler or a tracer such that
  /// the "tear down" computations are not included in the metrics.
  final AsyncVoidCallback tearDownAllWillRun;

  /// Runs the benchmark and reports the results.
  Future<Profile> run() async {
    await recorder.setUpAll();
    await setUpAllDidRun();
    final Profile profile = await recorder.run();
    await tearDownAllWillRun();
    await recorder.tearDownAll();
    return profile;
  }
}

/// Base class for benchmark recorders.
///
/// Each benchmark recorder has a [name] and a [run] method at a minimum.
abstract class Recorder {
  Recorder._(this.name, this.isTracingEnabled);

  /// Whether this recorder requires tracing using Chrome's DevTools Protocol's
  /// "Tracing" API.
  final bool isTracingEnabled;

  /// The name of the benchmark.
  ///
  /// The results displayed in the Flutter Dashboard will use this name as a
  /// prefix.
  final String name;

  /// Returns the recorded profile.
  ///
  /// This value is only available while the benchmark is running.
  Profile? get profile;

  /// Whether the benchmark should continue running.
  ///
  /// Returns `false` if the benchmark collected enough data and it's time to
  /// stop.
  bool shouldContinue() => profile?.shouldContinue() ?? true;

  /// Called once before all runs of this benchmark recorder.
  ///
  /// This is useful for doing one-time setup work that's needed for the
  /// benchmark.
  Future<void> setUpAll() async {}

  /// The implementation of the benchmark that will produce a [Profile].
  Future<Profile> run();

  /// Called once after all runs of this benchmark recorder.
  ///
  /// This is useful for doing one-time clean up work after the benchmark is
  /// complete.
  Future<void> tearDownAll() async {}
}

/// A recorder for benchmarking raw execution of Dart code.
///
/// This is useful for benchmarks that don't need frames or widgets.
///
/// Example:
///
/// ```dart
/// class BenchForLoop extends RawRecorder {
///   BenchForLoop() : super(name: benchmarkName);
///
///   static const String benchmarkName = 'for_loop';
///
///   @override
///   void body(Profile profile) {
///     profile.record('loop', () {
///       double x = 0;
///       for (int i = 0; i < 10000000; i++) {
///         x *= 1.5;
///       }
///     });
///   }
/// }
/// ```
abstract class RawRecorder extends Recorder {
  RawRecorder({required String name, bool useCustomWarmUp = false})
    : _useCustomWarmUp = useCustomWarmUp,
      super._(name, false);

  /// Whether to delimit warm-up frames in a custom way.
  final bool _useCustomWarmUp;

  /// The body of the benchmark.
  ///
  /// This is the part that records measurements of the benchmark.
  FutureOr<void> body(Profile profile);

  @override
  Profile? get profile => _profile;
  Profile? _profile;

  @override
  @nonVirtual
  Future<Profile> run() async {
    _profile = Profile(name: name, useCustomWarmUp: _useCustomWarmUp);
    do {
      await Future<void>.delayed(Duration.zero);
      final FutureOr<void> result = body(_profile!);
      if (result is Future) {
        await result;
      }
    } while (shouldContinue());
    return _profile!;
  }
}

/// A recorder for benchmarking interactions with the engine without the
/// framework by directly exercising [SceneBuilder].
///
/// To implement a benchmark, extend this class and implement [onDrawFrame].
///
/// Example:
///
/// ```dart
/// class BenchDrawCircle extends SceneBuilderRecorder {
///   BenchDrawCircle() : super(name: benchmarkName);
///
///   static const String benchmarkName = 'draw_circle';
///
///   @override
///   void onDrawFrame(SceneBuilder sceneBuilder) {
///     final PictureRecorder pictureRecorder = PictureRecorder();
///     final Canvas canvas = Canvas(pictureRecorder);
///     final Paint paint = Paint()..color = const Color.fromARGB(255, 255, 0, 0);
///     final Size windowSize = window.physicalSize;
///     canvas.drawCircle(windowSize.center(Offset.zero), 50.0, paint);
///     final Picture picture = pictureRecorder.endRecording();
///     sceneBuilder.addPicture(picture);
///   }
/// }
/// ```
abstract class SceneBuilderRecorder extends Recorder {
  SceneBuilderRecorder({required String name}) : super._(name, true);

  @override
  Profile? get profile => _profile;
  Profile? _profile;

  /// Called from [dart:ui.PlatformDispatcher.onBeginFrame].
  @mustCallSuper
  void onBeginFrame() {}

  /// Called on every frame.
  ///
  /// An implementation should exercise the [sceneBuilder] to build a frame.
  /// However, it must not call [SceneBuilder.build] or
  /// [dart:ui.FlutterView.render]. Instead the benchmark harness will call them
  /// and time them appropriately.
  void onDrawFrame(SceneBuilder sceneBuilder);

  @override
  Future<Profile> run() {
    final Completer<Profile> profileCompleter = Completer<Profile>();
    _profile = Profile(name: name);

    PlatformDispatcher.instance.onBeginFrame = (_) {
      try {
        startMeasureFrame(profile!);
        onBeginFrame();
      } catch (error, stackTrace) {
        profileCompleter.completeError(error, stackTrace);
        rethrow;
      }
    };
    PlatformDispatcher.instance.onDrawFrame = () {
      try {
        _profile!.record('drawFrameDuration', () {
          final SceneBuilder sceneBuilder = SceneBuilder();
          onDrawFrame(sceneBuilder);
          _profile!.record('sceneBuildDuration', () {
            final Scene scene = sceneBuilder.build();
            _profile!.record('windowRenderDuration', () {
              view.render(scene);
            }, reported: false);
          }, reported: false);
        }, reported: true);
        endMeasureFrame();

        if (shouldContinue()) {
          PlatformDispatcher.instance.scheduleFrame();
        } else {
          profileCompleter.complete(_profile!);
        }
      } catch (error, stackTrace) {
        profileCompleter.completeError(error, stackTrace);
        rethrow;
      }
    };
    PlatformDispatcher.instance.scheduleFrame();
    return profileCompleter.future;
  }

  FlutterView get view {
    assert(
      PlatformDispatcher.instance.implicitView != null,
      'This benchmark requires the embedder to provide an implicit view.',
    );
    return PlatformDispatcher.instance.implicitView!;
  }
}

/// A recorder for benchmarking interactions with the framework by creating
/// widgets.
///
/// To implement a benchmark, extend this class and implement [createWidget].
///
/// Example:
///
/// ```dart
/// class BenchListView extends WidgetRecorder {
///   BenchListView() : super(name: benchmarkName);
///
///   static const String benchmarkName = 'bench_list_view';
///
///   @override
///   Widget createWidget() {
///     return Directionality(
///       textDirection: TextDirection.ltr,
///       child: _TestListViewWidget(),
///     );
///   }
/// }
///
/// class _TestListViewWidget extends StatefulWidget {
///   @override
///   State<StatefulWidget> createState() {
///     return _TestListViewWidgetState();
///   }
/// }
///
/// class _TestListViewWidgetState extends State<_TestListViewWidget> {
///   ScrollController scrollController;
///
///   @override
///   void initState() {
///     super.initState();
///     scrollController = ScrollController();
///     Timer.run(() async {
///       bool forward = true;
///       while (true) {
///         await scrollController.animateTo(
///           forward ? 300 : 0,
///           curve: Curves.linear,
///           duration: const Duration(seconds: 1),
///         );
///         forward = !forward;
///       }
///     });
///   }
///
///   @override
///   Widget build(BuildContext context) {
///     return ListView.builder(
///       controller: scrollController,
///       itemCount: 10000,
///       itemBuilder: (BuildContext context, int index) {
///         return Text('Item #$index');
///       },
///     );
///   }
/// }
/// ```
abstract class WidgetRecorder extends Recorder implements FrameRecorder {
  WidgetRecorder({required String name, this.useCustomWarmUp = false}) : super._(name, true);

  /// Creates a widget to be benchmarked.
  ///
  /// The widget must create its own animation to drive the benchmark. The
  /// animation should continue indefinitely. The benchmark harness will stop
  /// pumping frames automatically.
  Widget createWidget();

  final List<VoidCallback> _didStopCallbacks = <VoidCallback>[];
  @override
  void registerDidStop(VoidCallback cb) {
    _didStopCallbacks.add(cb);
  }

  @override
  Profile? profile;
  Completer<void>? _runCompleter;

  /// Whether to delimit warm-up frames in a custom way.
  final bool useCustomWarmUp;

  late Stopwatch _drawFrameStopwatch;

  @override
  @mustCallSuper
  void frameWillDraw() {
    startMeasureFrame(profile!);
    _drawFrameStopwatch = Stopwatch()..start();
  }

  @override
  @mustCallSuper
  void frameDidDraw() {
    endMeasureFrame();
    profile!.addDataPoint('drawFrameDuration', _drawFrameStopwatch.elapsed, reported: true);

    if (shouldContinue()) {
      PlatformDispatcher.instance.scheduleFrame();
    } else {
      for (final VoidCallback fn in _didStopCallbacks) {
        fn();
      }
      _runCompleter!.complete();
    }
  }

  @override
  void _onError(Object error, StackTrace? stackTrace) {
    _runCompleter!.completeError(error, stackTrace);
  }

  late final _RecordingWidgetsBinding _binding;

  @override
  @mustCallSuper
  Future<void> setUpAll() async {
    _binding = _RecordingWidgetsBinding.ensureInitialized();
  }

  @override
  Future<Profile> run() async {
    _runCompleter = Completer<void>();
    final Profile localProfile = profile = Profile(name: name, useCustomWarmUp: useCustomWarmUp);
    final Widget widget = createWidget();

    registerEngineBenchmarkValueListener(kProfilePrerollFrame, (num value) {
      localProfile.addDataPoint(
        kProfilePrerollFrame,
        Duration(microseconds: value.toInt()),
        reported: false,
      );
    });
    registerEngineBenchmarkValueListener(kProfileApplyFrame, (num value) {
      localProfile.addDataPoint(
        kProfileApplyFrame,
        Duration(microseconds: value.toInt()),
        reported: false,
      );
    });

    _binding._beginRecording(this, widget);

    try {
      await _runCompleter!.future;
      return localProfile;
    } finally {
      stopListeningToEngineBenchmarkValues(kProfilePrerollFrame);
      stopListeningToEngineBenchmarkValues(kProfileApplyFrame);
      _runCompleter = null;
      profile = null;
    }
  }
}

/// A recorder for measuring the performance of building a widget from scratch
/// starting from an empty frame.
///
/// The recorder will call [createWidget] and render it, then it will pump
/// another frame that clears the screen. It repeats this process, measuring the
/// performance of frames that render the widget and ignoring the frames that
/// clear the screen.
abstract class WidgetBuildRecorder extends Recorder implements FrameRecorder {
  WidgetBuildRecorder({required String name}) : super._(name, true);

  /// Creates a widget to be benchmarked.
  ///
  /// The widget is not expected to animate as we only care about construction
  /// of the widget. If you are interested in benchmarking an animation,
  /// consider using [WidgetRecorder].
  Widget createWidget();

  final List<VoidCallback> _didStopCallbacks = <VoidCallback>[];
  @override
  void registerDidStop(VoidCallback cb) {
    _didStopCallbacks.add(cb);
  }

  @override
  Profile? profile;
  Completer<void>? _runCompleter;

  late Stopwatch _drawFrameStopwatch;

  /// Whether in this frame we should call [createWidget] and render it.
  ///
  /// If false, then this frame will clear the screen.
  bool showWidget = true;

  /// The state that hosts the widget under test.
  late _WidgetBuildRecorderHostState _hostState;

  Widget? _getWidgetForFrame() {
    if (showWidget) {
      return createWidget();
    } else {
      return null;
    }
  }

  late final _RecordingWidgetsBinding _binding;

  @override
  @mustCallSuper
  Future<void> setUpAll() async {
    _binding = _RecordingWidgetsBinding.ensureInitialized();
  }

  @override
  @mustCallSuper
  void frameWillDraw() {
    if (showWidget) {
      startMeasureFrame(profile!);
      _drawFrameStopwatch = Stopwatch()..start();
    }
  }

  @override
  @mustCallSuper
  void frameDidDraw() {
    // Only record frames that show the widget.
    if (showWidget) {
      endMeasureFrame();
      profile!.addDataPoint('drawFrameDuration', _drawFrameStopwatch.elapsed, reported: true);
    }

    if (shouldContinue()) {
      showWidget = !showWidget;
      _hostState._setStateTrampoline();
    } else {
      for (final VoidCallback fn in _didStopCallbacks) {
        fn();
      }
      _runCompleter!.complete();
    }
  }

  @override
  void _onError(Object error, StackTrace? stackTrace) {
    _runCompleter!.completeError(error, stackTrace);
  }

  @override
  Future<Profile> run() async {
    _runCompleter = Completer<void>();
    final Profile localProfile = profile = Profile(name: name);
    _binding._beginRecording(this, _WidgetBuildRecorderHost(this));

    try {
      await _runCompleter!.future;
      return localProfile;
    } finally {
      _runCompleter = null;
      profile = null;
    }
  }
}

/// Hosts widgets created by [WidgetBuildRecorder].
class _WidgetBuildRecorderHost extends StatefulWidget {
  const _WidgetBuildRecorderHost(this.recorder);

  final WidgetBuildRecorder recorder;

  @override
  State<StatefulWidget> createState() => _WidgetBuildRecorderHostState();
}

class _WidgetBuildRecorderHostState extends State<_WidgetBuildRecorderHost> {
  @override
  void initState() {
    super.initState();
    widget.recorder._hostState = this;
  }

  // This is just to bypass the @protected on setState.
  void _setStateTrampoline() {
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox.expand(child: widget.recorder._getWidgetForFrame());
  }
}

/// Series of time recordings indexed in time order.
///
/// A timeseries is expected to contain at least one warm-up frame added by
/// calling [add] with `isWarmUpValue` set to true, followed by at least one
/// measured value added by calling [add] with `isWarmUpValue` set to false.
class Timeseries {
  /// Creates an empty timeseries.
  ///
  /// The [name] is a unique name of this timeseries. If [isReported] is true
  /// this timeseries is reported to the benchmark dashboard.
  Timeseries(this.name, this.isReported);

  /// The label of this timeseries used for debugging and result inspection.
  final String name;

  /// Whether this timeseries is reported to the benchmark dashboard.
  ///
  /// If `true` a new benchmark card is created for the timeseries and is
  /// visible on the dashboard.
  ///
  /// If `false` the data is stored but it does not show up on the dashboard.
  /// Use unreported metrics for metrics that are useful for manual inspection
  /// but that are too fine-grained to be useful for tracking on the dashboard.
  final bool isReported;

  /// The number of samples ignored as warm-up frames.
  int _warmUpSampleCount = 0;

  /// List of all the values that have been recorded.
  ///
  /// This list has no limit.
  final List<double> _allValues = <double>[];

  /// The total amount of data collected, including ones that were dropped
  /// because of the sample size limit.
  int get count => _allValues.length;

  /// Extracts useful statistics out of this timeseries.
  ///
  /// See [TimeseriesStats] for more details.
  TimeseriesStats computeStats() {
    // Assertions do not use the `assert` keyword because benchmarks run in
    // profile mode, where asserts are tree-shaken out.
    if (_warmUpSampleCount == 0) {
      throw StateError(
        'The benchmark did not warm-up. Use at least one sample to warm-up '
        'the benchmark to reduce noise.',
      );
    }
    if (_warmUpSampleCount >= count) {
      throw StateError(
        'The benchmark did not report any measured samples. Add at least one '
        'sample after warm-up is done. There were $_warmUpSampleCount warm-up '
        'samples, and no measured samples in this timeseries.',
      );
    }

    // The first few values we simply discard and never look at. They're from the warm-up phase.
    final List<double> warmUpValues = _allValues.sublist(0, _warmUpSampleCount);

    // Values we analyze.
    final List<double> candidateValues = _allValues.sublist(_warmUpSampleCount);

    // The average that includes outliers.
    final double dirtyAverage = _computeAverage(name, candidateValues);

    // The standard deviation that includes outliers.
    final double dirtyStandardDeviation = _computeStandardDeviationForPopulation(
      name,
      candidateValues,
    );

    // Any value that's higher than this is considered an outlier.
    // Two standard deviations captures 95% of a normal distribution.
    final double outlierCutOff = dirtyAverage + dirtyStandardDeviation * 2;

    // Candidates with outliers removed.
    final Iterable<double> cleanValues = candidateValues.where(
      (double value) => value <= outlierCutOff,
    );

    // Outlier candidates.
    final Iterable<double> outliers = candidateValues.where(
      (double value) => value > outlierCutOff,
    );

    // Final statistics.
    final double cleanAverage = _computeAverage(name, cleanValues);
    final double standardDeviation = _computeStandardDeviationForPopulation(name, cleanValues);
    final double noise = cleanAverage > 0.0 ? standardDeviation / cleanAverage : 0.0;

    // Compute outlier average. If there are no outliers the outlier average is
    // the same as clean value average. In other words, in a perfect benchmark
    // with no noise the difference between average and outlier average is zero,
    // which the best possible outcome. Noise produces a positive difference
    // between the two.
    final double outlierAverage =
        outliers.isNotEmpty ? _computeAverage(name, outliers) : cleanAverage;

    final List<AnnotatedSample> annotatedValues = <AnnotatedSample>[
      for (final double warmUpValue in warmUpValues)
        AnnotatedSample(
          magnitude: warmUpValue,
          isOutlier: warmUpValue > outlierCutOff,
          isWarmUpValue: true,
        ),
      for (final double candidate in candidateValues)
        AnnotatedSample(
          magnitude: candidate,
          isOutlier: candidate > outlierCutOff,
          isWarmUpValue: false,
        ),
    ];

    return TimeseriesStats(
      name: name,
      average: cleanAverage,
      outlierCutOff: outlierCutOff,
      outlierAverage: outlierAverage,
      standardDeviation: standardDeviation,
      noise: noise,
      cleanSampleCount: cleanValues.length,
      outlierSampleCount: outliers.length,
      samples: annotatedValues,
    );
  }

  // Whether the timeseries is in the warm-up phase.
  bool _isWarmingUp = true;

  /// Adds a value to this timeseries.
  void add(double value, {required bool isWarmUpValue}) {
    if (value < 0.0) {
      throw StateError('Timeseries $name: negative metric values are not supported. Got: $value');
    }
    if (isWarmUpValue) {
      if (!_isWarmingUp) {
        throw StateError(
          'A warm-up value was added to the timeseries after the warm-up phase finished.',
        );
      }
      _warmUpSampleCount += 1;
    } else if (_isWarmingUp) {
      _isWarmingUp = false;
    }
    _allValues.add(value);
  }
}

/// Various statistics about a [Timeseries].
///
/// See the docs on the individual fields for more details.
@sealed
class TimeseriesStats {
  const TimeseriesStats({
    required this.name,
    required this.average,
    required this.outlierCutOff,
    required this.outlierAverage,
    required this.standardDeviation,
    required this.noise,
    required this.cleanSampleCount,
    required this.outlierSampleCount,
    required this.samples,
  });

  /// The label used to refer to the corresponding timeseries.
  final String name;

  /// The average value of the measured samples without outliers.
  final double average;

  /// The standard deviation in the measured samples without outliers.
  final double standardDeviation;

  /// The noise as a multiple of the [average] value takes from clean samples.
  ///
  /// This value can be multiplied by 100.0 to get noise as a percentage of
  /// the average.
  ///
  /// If [average] is zero, treats the result as perfect score, returns zero.
  final double noise;

  /// The maximum value a sample can have without being considered an outlier.
  ///
  /// See [Timeseries.computeStats] for details on how this value is computed.
  final double outlierCutOff;

  /// The average of outlier samples.
  ///
  /// This value can be used to judge how badly we jank, when we jank.
  ///
  /// Another useful metrics is the difference between [outlierAverage] and
  /// [average]. The smaller the value the more predictable is the performance
  /// of the corresponding benchmark.
  final double outlierAverage;

  /// The number of measured samples after outlier are removed.
  final int cleanSampleCount;

  /// The number of outliers.
  final int outlierSampleCount;

  /// All collected samples, annotated with statistical information.
  ///
  /// See [AnnotatedSample] for more details.
  final List<AnnotatedSample> samples;

  /// Outlier average divided by clean average.
  ///
  /// This is a measure of performance consistency. The higher this number the
  /// worse is jank when it happens. Smaller is better, with 1.0 being the
  /// perfect score. If [average] is zero, this value defaults to 1.0.
  double get outlierRatio =>
      average > 0.0
          ? outlierAverage / average
          : 1.0; // this can only happen in perfect benchmark that reports only zeros

  @override
  String toString() {
    final StringBuffer buffer = StringBuffer();
    buffer.writeln(
      '$name: (samples: $cleanSampleCount clean/$outlierSampleCount '
      'outliers/${cleanSampleCount + outlierSampleCount} '
      'measured/${samples.length} total)',
    );
    buffer.writeln(' | average: $average μs');
    buffer.writeln(' | outlier average: $outlierAverage μs');
    buffer.writeln(' | outlier/clean ratio: ${outlierRatio}x');
    buffer.writeln(' | noise: ${_ratioToPercent(noise)}');
    return buffer.toString();
  }
}

/// Annotates a single measurement with statistical information.
@sealed
class AnnotatedSample {
  const AnnotatedSample({
    required this.magnitude,
    required this.isOutlier,
    required this.isWarmUpValue,
  });

  /// The non-negative raw result of the measurement.
  final double magnitude;

  /// Whether this sample was considered an outlier.
  final bool isOutlier;

  /// Whether this sample was taken during the warm-up phase.
  ///
  /// If this value is `true`, this sample does not participate in
  /// statistical computations. However, the sample would still be
  /// shown in the visualization of results so that the benchmark
  /// can be inspected manually to make sure there's a predictable
  /// warm-up regression slope.
  final bool isWarmUpValue;
}

/// Base class for a profile collected from running a benchmark.
class Profile {
  /// Creates an empty profile that can be populated with benchmark samples
  /// using [record], [recordAsync], and [addDataPoint] methods.
  ///
  /// The [name] is the unique name of this profile that distinguishes is from
  /// other profiles. Typically, the name will describe the benchmark.
  ///
  /// If [useCustomWarmUp] is true the benchmark will continue running until
  /// [stopBenchmark] is called. Otherwise, the benchmark collects the
  /// [kDefaultTotalSampleCount] samples and stops automatically.
  Profile({required this.name, this.useCustomWarmUp = false});

  /// The name of the benchmark that produced this profile.
  final String name;

  /// Whether to delimit warm-up frames in a custom way.
  final bool useCustomWarmUp;

  /// True if the benchmark is currently measuring warm-up frames.
  bool get isWarmingUp => _isWarmingUp;
  bool _isWarmingUp = true;

  /// True if the benchmark is currently running.
  bool get isRunning => _isRunning;
  bool _isRunning = true;

  /// Stops the warm-up phase.
  ///
  /// After calling this method, subsequent calls to [record], [recordAsync],
  /// and [addDataPoint] will record measured data samples.
  ///
  /// Call this method only once for each profile and only when [isWarmingUp]
  /// is true.
  void stopWarmingUp() {
    if (!_isWarmingUp) {
      throw StateError('Warm-up already stopped.');
    } else {
      _isWarmingUp = false;
    }
  }

  /// Stops the benchmark.
  ///
  /// Call this method only once for each profile and only when [isWarmingUp]
  /// is false (i.e. after calling [stopWarmingUp]).
  void stopBenchmark() {
    if (_isWarmingUp) {
      throw StateError(
        'Warm-up has not finished yet. Benchmark should only be stopped after '
        'it recorded at least one sample after the warm-up.',
      );
    } else if (scoreData.isEmpty) {
      throw StateError('The benchmark did not collect any data.');
    } else {
      _isRunning = false;
    }
  }

  /// This data will be used to display cards in the Flutter Dashboard.
  final Map<String, Timeseries> scoreData = <String, Timeseries>{};

  /// This data isn't displayed anywhere. It's stored for completeness purposes.
  final Map<String, dynamic> extraData = <String, dynamic>{};

  /// Invokes [callback] and records the duration of its execution under [key].
  ///
  /// See also:
  ///
  ///  * [recordAsync], which records asynchronous work.
  Duration record(String key, VoidCallback callback, {required bool reported}) {
    final Duration duration = timeAction(callback);
    addDataPoint(key, duration, reported: reported);
    return duration;
  }

  /// Invokes [callback] and records the amount of time the returned future takes.
  ///
  /// See also:
  ///
  ///  * [record], which records synchronous work.
  Future<Duration> recordAsync(String key, AsyncCallback callback, {required bool reported}) async {
    final Duration duration = await timeAsyncAction(callback);
    addDataPoint(key, duration, reported: reported);
    return duration;
  }

  /// Adds a timed sample to the timeseries corresponding to [key].
  ///
  /// Set [reported] to `true` to report the timeseries to the dashboard UI.
  ///
  /// Set [reported] to `false` to store the data, but not show it on the
  /// dashboard UI.
  void addDataPoint(String key, Duration duration, {required bool reported}) {
    scoreData
        .putIfAbsent(key, () => Timeseries(key, reported))
        .add(duration.inMicroseconds.toDouble(), isWarmUpValue: isWarmingUp);

    if (!useCustomWarmUp) {
      // The stopWarmingUp and stopBenchmark will not be called. Use the
      // auto-stopping logic.
      _autoUpdateBenchmarkPhase();
    }
  }

  /// A convenience wrapper over [addDataPoint] for adding [AggregatedTimedBlock]
  /// to the profile.
  ///
  /// Uses [AggregatedTimedBlock.name] as the name of the data point, and
  /// [AggregatedTimedBlock.duration] as the duration.
  void addTimedBlock(AggregatedTimedBlock timedBlock, {required bool reported}) {
    addDataPoint(
      timedBlock.name,
      Duration(microseconds: timedBlock.duration.toInt()),
      reported: reported,
    );
  }

  /// Checks the samples collected so far and sets the appropriate benchmark phase.
  ///
  /// If enough warm-up samples have been collected, stops the warm-up phase and
  /// begins the measuring phase.
  ///
  /// If enough total samples have been collected, stops the benchmark.
  void _autoUpdateBenchmarkPhase() {
    if (useCustomWarmUp) {
      StateError(
        'Must not call _autoUpdateBenchmarkPhase if custom warm-up is used. '
        'Call `stopWarmingUp` and `stopBenchmark` instead.',
      );
    }

    if (_isWarmingUp) {
      final bool doesHaveEnoughWarmUpSamples = scoreData.keys.every(
        (String key) => scoreData[key]!.count >= _kDefaultWarmUpSampleCount,
      );
      if (doesHaveEnoughWarmUpSamples) {
        stopWarmingUp();
      }
    } else if (_isRunning) {
      final bool doesHaveEnoughTotalSamples = scoreData.keys.every(
        (String key) => scoreData[key]!.count >= kDefaultTotalSampleCount,
      );
      if (doesHaveEnoughTotalSamples) {
        stopBenchmark();
      }
    }
  }

  /// Decides whether the data collected so far is sufficient to stop, or
  /// whether the benchmark should continue collecting more data.
  ///
  /// The signals used are sample size, noise, and duration.
  ///
  /// If any of the timeseries doesn't satisfy the noise requirements, this
  /// method will return true (asking the benchmark to continue collecting
  /// data).
  bool shouldContinue() {
    // If there are no `Timeseries` in the `scoreData`, then we haven't
    // recorded anything yet. Don't stop.
    if (scoreData.isEmpty) {
      return true;
    }

    return isRunning;
  }

  /// Returns a JSON representation of the profile that will be sent to the
  /// server.
  Map<String, dynamic> toJson() {
    final List<String> scoreKeys = <String>[];
    final Map<String, dynamic> json = <String, dynamic>{'name': name, 'scoreKeys': scoreKeys};

    for (final String key in scoreData.keys) {
      final Timeseries timeseries = scoreData[key]!;

      if (timeseries.isReported) {
        scoreKeys.add('$key.average');
        // Report `outlierRatio` rather than `outlierAverage`, because
        // the absolute value of outliers is less interesting than the
        // ratio.
        scoreKeys.add('$key.outlierRatio');
      }

      final TimeseriesStats stats = timeseries.computeStats();
      json['$key.average'] = stats.average;
      json['$key.outlierAverage'] = stats.outlierAverage;
      json['$key.outlierRatio'] = stats.outlierRatio;
      json['$key.noise'] = stats.noise;
    }

    json.addAll(extraData);

    return json;
  }

  @override
  String toString() {
    final StringBuffer buffer = StringBuffer();
    buffer.writeln('name: $name');
    for (final String key in scoreData.keys) {
      final Timeseries timeseries = scoreData[key]!;
      final TimeseriesStats stats = timeseries.computeStats();
      buffer.writeln(stats.toString());
    }
    for (final String key in extraData.keys) {
      final dynamic value = extraData[key];
      if (value is List) {
        buffer.writeln('$key:');
        for (final dynamic item in value) {
          buffer.writeln(' - $item');
        }
      } else {
        buffer.writeln('$key: $value');
      }
    }
    return buffer.toString();
  }
}

/// Computes the arithmetic mean (or average) of given [values].
double _computeAverage(String label, Iterable<double> values) {
  if (values.isEmpty) {
    throw StateError('$label: attempted to compute an average of an empty value list.');
  }

  final double sum = values.reduce((double a, double b) => a + b);
  return sum / values.length;
}

/// Computes population standard deviation.
///
/// Unlike sample standard deviation, which divides by N - 1, this divides by N.
///
/// See also:
///
///  * <https://en.wikipedia.org/wiki/Standard_deviation>
double _computeStandardDeviationForPopulation(String label, Iterable<double> population) {
  if (population.isEmpty) {
    throw StateError('$label: attempted to compute the standard deviation of empty population.');
  }
  final double mean = _computeAverage(label, population);
  final double sumOfSquaredDeltas = population.fold<double>(
    0.0,
    (double previous, double value) => previous += math.pow(value - mean, 2),
  );
  return math.sqrt(sumOfSquaredDeltas / population.length);
}

String _ratioToPercent(double value) {
  return '${(value * 100).toStringAsFixed(2)}%';
}

/// Implemented by recorders that use [_RecordingWidgetsBinding] to receive
/// frame life-cycle calls.
abstract class FrameRecorder {
  /// Add a callback that will be called by the recorder when it stops recording.
  void registerDidStop(VoidCallback cb);

  /// Called just before calling [SchedulerBinding.handleDrawFrame].
  void frameWillDraw();

  /// Called immediately after calling [SchedulerBinding.handleDrawFrame].
  void frameDidDraw();

  /// Reports an error.
  ///
  /// The implementation is expected to halt benchmark execution as soon as possible.
  void _onError(Object error, StackTrace? stackTrace);
}

/// A variant of [WidgetsBinding] that collaborates with a [Recorder] to decide
/// when to stop pumping frames.
///
/// A normal [WidgetsBinding] typically always pumps frames whenever a widget
/// instructs it to do so by calling [scheduleFrame] (transitively via
/// `setState`). This binding will stop pumping new frames as soon as benchmark
/// parameters are satisfactory (e.g. when the metric noise levels become low
/// enough).
class _RecordingWidgetsBinding extends BindingBase
    with
        GestureBinding,
        SchedulerBinding,
        ServicesBinding,
        PaintingBinding,
        SemanticsBinding,
        RendererBinding,
        WidgetsBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  /// The singleton instance of this object.
  ///
  /// Provides access to the features exposed by this class. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [_RecordingWidgetsBinding.ensureInitialized].
  static _RecordingWidgetsBinding get instance => BindingBase.checkInstance(_instance);
  static _RecordingWidgetsBinding? _instance;

  /// Returns an instance of the [_RecordingWidgetsBinding], creating and
  /// initializing it if necessary.
  ///
  /// See also:
  ///
  ///  * [WidgetsFlutterBinding.ensureInitialized], the equivalent in the widgets framework.
  static _RecordingWidgetsBinding ensureInitialized() {
    if (_instance == null) {
      _RecordingWidgetsBinding();
    }
    return instance;
  }

  FrameRecorder? _recorder;
  bool _hasErrored = false;

  /// To short-circuit all frame lifecycle methods when the benchmark has
  /// stopped collecting data.
  bool _benchmarkStopped = false;

  void _beginRecording(FrameRecorder recorder, Widget widget) {
    if (_recorder != null) {
      throw Exception('Cannot call _RecordingWidgetsBinding._beginRecording more than once');
    }
    final FlutterExceptionHandler? originalOnError = FlutterError.onError;

    recorder.registerDidStop(() {
      _benchmarkStopped = true;
    });

    // Fail hard and fast on errors. Benchmarks should not have any errors.
    FlutterError.onError = (FlutterErrorDetails details) {
      _haltBenchmarkWithError(details.exception, details.stack);
      originalOnError?.call(details);
    };
    _recorder = recorder;
    runApp(widget);
  }

  void _haltBenchmarkWithError(Object error, StackTrace? stackTrace) {
    if (_hasErrored) {
      return;
    }
    _recorder?._onError(error, stackTrace);
    _hasErrored = true;
  }

  @override
  void handleBeginFrame(Duration? rawTimeStamp) {
    // Don't keep on truckin' if there's an error or the benchmark has stopped.
    if (_hasErrored || _benchmarkStopped) {
      return;
    }
    try {
      super.handleBeginFrame(rawTimeStamp);
    } catch (error, stackTrace) {
      _haltBenchmarkWithError(error, stackTrace);
      rethrow;
    }
  }

  @override
  void scheduleFrame() {
    // Don't keep on truckin' if there's an error or the benchmark has stopped.
    if (_hasErrored || _benchmarkStopped) {
      return;
    }
    super.scheduleFrame();
  }

  @override
  void handleDrawFrame() {
    // Don't keep on truckin' if there's an error or the benchmark has stopped.
    if (_hasErrored || _benchmarkStopped) {
      return;
    }
    try {
      _recorder?.frameWillDraw();
      super.handleDrawFrame();
      _recorder?.frameDidDraw();
    } catch (error, stackTrace) {
      _haltBenchmarkWithError(error, stackTrace);
      rethrow;
    }
  }
}

int _currentFrameNumber = 1;

/// If [_calledStartMeasureFrame] is true, we have called [startMeasureFrame]
/// but have not its pairing [endMeasureFrame] yet.
///
/// This flag ensures that [startMeasureFrame] and [endMeasureFrame] are always
/// called in pairs, with [startMeasureFrame] followed by [endMeasureFrame].
bool _calledStartMeasureFrame = false;

/// Whether we are recording a measured frame.
///
/// This flag ensures that we always stop measuring a frame if we
/// have started one. Because we want to skip warm-up frames, this flag
/// is necessary.
bool _isMeasuringFrame = false;

/// Adds a marker indication the beginning of frame rendering.
///
/// This adds an event to the performance trace used to find measured frames in
/// Chrome tracing data. The tracing data contains all frames, but some
/// benchmarks are only interested in a subset of frames. For example,
/// [WidgetBuildRecorder] only measures frames that build widgets, and ignores
/// frames that clear the screen.
///
/// Warm-up frames are not measured. If [profile.isWarmingUp] is true,
/// this function does nothing.
void startMeasureFrame(Profile profile) {
  if (_calledStartMeasureFrame) {
    throw Exception('`startMeasureFrame` called twice in a row.');
  }

  _calledStartMeasureFrame = true;

  if (!profile.isWarmingUp) {
    // Tell the browser to mark the beginning of the frame.
    web.window.performance.mark('measured_frame_start#$_currentFrameNumber');
    _isMeasuringFrame = true;
  }
}

/// Signals the end of a measured frame.
///
/// See [startMeasureFrame] for details on what this instrumentation is used
/// for.
///
/// Warm-up frames are not measured. If [profile.isWarmingUp] was true
/// when the corresponding [startMeasureFrame] was called,
/// this function does nothing.
void endMeasureFrame() {
  if (!_calledStartMeasureFrame) {
    throw Exception('`startMeasureFrame` has not been called before calling `endMeasureFrame`');
  }

  _calledStartMeasureFrame = false;

  if (_isMeasuringFrame) {
    // Tell the browser to mark the end of the frame, and measure the duration.
    web.window.performance.mark('measured_frame_end#$_currentFrameNumber');
    web.window.performance.measure(
      'measured_frame',
      'measured_frame_start#$_currentFrameNumber'.toJS,
      'measured_frame_end#$_currentFrameNumber',
    );

    // Increment the current frame number.
    _currentFrameNumber += 1;

    _isMeasuringFrame = false;
  }
}

/// A function that receives a benchmark value from the framework.
typedef EngineBenchmarkValueListener = void Function(num value);

// Maps from a value label name to a listener.
final Map<String, EngineBenchmarkValueListener> _engineBenchmarkListeners =
    <String, EngineBenchmarkValueListener>{};

/// Registers a [listener] for engine benchmark values labeled by [name].
///
/// If another listener is already registered, overrides it.
void registerEngineBenchmarkValueListener(String name, EngineBenchmarkValueListener listener) {
  if (_engineBenchmarkListeners.containsKey(name)) {
    throw StateError(
      'A listener for "$name" is already registered.\n'
      'Call `stopListeningToEngineBenchmarkValues` to unregister the previous '
      'listener before registering a new one.',
    );
  }

  if (_engineBenchmarkListeners.isEmpty) {
    // The first listener is being registered. Register the global listener.
    ui_web.benchmarkValueCallback = _dispatchEngineBenchmarkValue;
  }
  _engineBenchmarkListeners[name] = listener;
}

/// Stops listening to engine benchmark values under labeled by [name].
void stopListeningToEngineBenchmarkValues(String name) {
  _engineBenchmarkListeners.remove(name);
  if (_engineBenchmarkListeners.isEmpty) {
    // The last listener unregistered. Remove the global listener.
    ui_web.benchmarkValueCallback = null;
  }
}

// Dispatches a benchmark value reported by the engine to the relevant listener.
//
// If there are no listeners registered for [name], ignores the value.
void _dispatchEngineBenchmarkValue(String name, double value) {
  final EngineBenchmarkValueListener? listener = _engineBenchmarkListeners[name];
  if (listener != null) {
    listener(value);
  }
}
