// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'goldens.dart';
/// @docImport 'widget_tester.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';

// A Future<ui.Image> that stores the resolved result.
class _AsyncImage {
  _AsyncImage(Future<ui.Image> task) {
    _task = task.then((ui.Image image) {
      _result = image;
    });
  }

  // Returns the resolved image.
  Future<ui.Image> result() async {
    if (_result != null) {
      return _result!;
    }
    await _task;
    assert(_result != null);
    return _result!;
  }

  late final Future<void> _task;
  ui.Image? _result;

  // Wait for a list of `_AsyncImage` and returns the list of its resolved
  // images.
  static Future<List<ui.Image>> resolveList(List<_AsyncImage> targets) {
    final Iterable<Future<ui.Image>> images = targets.map<Future<ui.Image>>(
      (_AsyncImage target) => target.result(),
    );
    return Future.wait<ui.Image>(images);
  }
}

/// Records the frames of an animating widget, and later displays the frames as a
/// grid in an animation sheet.
///
/// This class does not support Web, because the animation sheet utilizes taking
/// screenshots, which is unsupported on the Web. Tests that use this class must
/// be noted with `skip: isBrowser`.
/// (https://github.com/flutter/flutter/issues/56001)
///
/// Using this class includes the following steps:
///
///  * Create an instance of this class.
///  * Register [dispose] to the test's tear down callbacks.
///  * Pump frames that render the target widget wrapped in [record]. Every frame
///    that has `recording` being true will be recorded.
///  * Acquire the output image with [collate] and compare against the golden
///    file.
///
/// {@tool snippet}
/// The following example shows how to record an animation sheet of an [InkWell]
/// being pressed then released.
///
/// ```dart
/// testWidgets('Inkwell animation sheet', (WidgetTester tester) async {
///   // Create instance
///   final AnimationSheetBuilder animationSheet = AnimationSheetBuilder(frameSize: const Size(48, 24));
///   addTearDown(animationSheet.dispose);
///
///   final Widget target = Material(
///     child: Directionality(
///       textDirection: TextDirection.ltr,
///       child: InkWell(
///         splashColor: Colors.blue,
///         onTap: () {},
///       ),
///     ),
///   );
///
///   // Optional: setup before recording (`recording` is false)
///   await tester.pumpWidget(animationSheet.record(
///     target,
///     recording: false,
///   ));
///
///   final TestGesture gesture = await tester.startGesture(tester.getCenter(find.byType(InkWell)));
///
///   // Start recording (`recording` is true)
///   await tester.pumpFrames(animationSheet.record(
///     target,
///     recording: true, // ignore: avoid_redundant_argument_values
///   ), const Duration(seconds: 1));
///
///   await gesture.up();
///
///   await tester.pumpFrames(animationSheet.record(
///     target,
///     recording: true, // ignore: avoid_redundant_argument_values
///   ), const Duration(seconds: 1));
///
///   // Compare against golden file
///   await expectLater(
///     animationSheet.collate(800),
///     matchesGoldenFile('inkwell.press.animation.png'),
///   );
/// }, skip: isBrowser); // Animation sheet does not support browser https://github.com/flutter/flutter/issues/56001
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [GoldenFileComparator], which introduces Golden File Testing.
class AnimationSheetBuilder {
  /// Starts a session of building an animation sheet.
  ///
  /// The [frameSize] is a tight constraint for the child to be recorded, and must not
  /// be null.
  ///
  /// The [allLayers] controls whether to record elements drawn out of the subtree,
  /// and defaults to false.
  AnimationSheetBuilder({required this.frameSize, this.allLayers = false}) : assert(!kIsWeb);

  /// Dispose all recorded frames and result images.
  ///
  /// This method must be called before the test case ends (usually as a tear
  /// down callback) to properly deallocate the images.
  ///
  /// After this method is called, there will be no frames to [collate].
  Future<void> dispose() async {
    final List<_AsyncImage> targets = <_AsyncImage>[..._recordedFrames, ..._results];
    _recordedFrames.clear();
    _results.clear();
    for (final ui.Image image in await _AsyncImage.resolveList(targets)) {
      image.dispose();
    }
  }

  /// The size of the child to be recorded.
  ///
  /// This size is applied as a tight layout constraint for the child, and is
  /// fixed throughout the building session.
  final Size frameSize;

  /// Whether the captured image comes from the entire tree, or only the
  /// subtree of [record].
  ///
  /// If [allLayers] is false, then the [record] widget will capture the image
  /// composited by its subtree. If [allLayers] is true, then the [record] will
  /// capture the entire tree composited and clipped by [record]'s region.
  ///
  /// The two modes are identical if there is nothing in front of [record].
  /// But in rare cases, what needs to be captured has to be rendered out of
  /// [record]'s subtree in its front. By setting [allLayers] to true, [record]
  /// captures everything within its region even if drawn outside of its
  /// subtree.
  ///
  /// Defaults to false.
  final bool allLayers;

  final List<_AsyncImage> _recordedFrames = <_AsyncImage>[];

  /// Returns a widget that renders a widget in a box that can be recorded.
  ///
  /// The returned widget wraps `child` in a box with a fixed size specified by
  /// [frameSize]. The `key` is also applied to the returned widget.
  ///
  /// The frame is only recorded if the `recording` argument is true, or during
  /// a procedure that is wrapped within [recording]. In either case, the
  /// painted result of each frame will be stored and later available for
  /// [collate]. If neither condition is met, the frames are not recorded, which
  /// is useful during setup phases.
  ///
  /// See also:
  ///
  ///  * [WidgetTester.pumpFrames], which renders a widget in a series of frames
  ///    with a fixed time interval.
  Widget record(Widget child, {Key? key, bool recording = true}) {
    return _AnimationSheetRecorder(
      key: key,
      size: frameSize,
      allLayers: allLayers,
      handleRecorded:
          !recording
              ? null
              : (Future<ui.Image> futureImage) {
                _recordedFrames.add(
                  _AsyncImage(() async {
                    final ui.Image image = await futureImage;
                    assert(
                      image.width == frameSize.width && image.height == frameSize.height,
                      'Unexpected size mismatch: frame has (${image.width}, ${image.height}) '
                      'while `frameSize` is $frameSize.',
                    );
                    return image;
                  }()),
                );
              },
      child: child,
    );
  }

  // The result images generated by `collate`.
  //
  // They're stored here to be disposed by [dispose].
  final List<_AsyncImage> _results = <_AsyncImage>[];

  /// Returns an result image by putting all frames together in a table.
  ///
  /// This method returns an image that arranges the captured frames in a table,
  /// which has `cellsPerRow` images per row with the order from left to right,
  /// top to bottom.
  ///
  /// The result image of this method is managed by [AnimationSheetBuilder],
  /// and should not be disposed by the caller.
  ///
  /// An example of using this method can be found at [AnimationSheetBuilder].
  Future<ui.Image> collate(int cellsPerRow) async {
    assert(
      _recordedFrames.isNotEmpty,
      'No frames are collected. Have you forgot to set `recording` to true?',
    );
    final _AsyncImage result = _AsyncImage(_collateFrames(_recordedFrames, frameSize, cellsPerRow));
    _results.add(result);
    return result.result();
  }
}

typedef _RecordedHandler = void Function(Future<ui.Image> image);

class _AnimationSheetRecorder extends StatefulWidget {
  const _AnimationSheetRecorder({
    this.handleRecorded,
    required this.child,
    required this.size,
    required this.allLayers,
    super.key,
  });

  final _RecordedHandler? handleRecorded;
  final Widget child;
  final Size size;
  final bool allLayers;

  @override
  State<StatefulWidget> createState() => _AnimationSheetRecorderState();
}

class _AnimationSheetRecorderState extends State<_AnimationSheetRecorder> {
  GlobalKey boundaryKey = GlobalKey();

  void _record(Duration duration) {
    assert(widget.handleRecorded != null);
    final _RenderRootableRepaintBoundary boundary =
        boundaryKey.currentContext!.findRenderObject()! as _RenderRootableRepaintBoundary;
    if (widget.allLayers) {
      widget.handleRecorded!(boundary.allLayersToImage());
    } else {
      widget.handleRecorded!(boundary.toImage());
    }
  }

  @override
  Widget build(BuildContext context) {
    return Align(
      alignment: Alignment.topLeft,
      child: SizedBox.fromSize(
        size: widget.size,
        child: _RootableRepaintBoundary(
          key: boundaryKey,
          child: _PostFrameCallbacker(
            callback: widget.handleRecorded == null ? null : _record,
            child: widget.child,
          ),
        ),
      ),
    );
  }
}

// Invokes `callback` and [markNeedsPaint] during the post-frame callback phase
// of every frame.
//
// If `callback` is non-null, `_PostFrameCallbacker` adds a post-frame callback
// every time it paints, during which it calls the provided `callback` then
// invokes [markNeedsPaint].
//
// If `callback` is null, `_PostFrameCallbacker` is equivalent to a proxy box.
class _PostFrameCallbacker extends SingleChildRenderObjectWidget {
  const _PostFrameCallbacker({super.child, this.callback});

  final FrameCallback? callback;

  @override
  _RenderPostFrameCallbacker createRenderObject(BuildContext context) =>
      _RenderPostFrameCallbacker(callback: callback);

  @override
  void updateRenderObject(BuildContext context, _RenderPostFrameCallbacker renderObject) {
    renderObject.callback = callback;
  }
}

class _RenderPostFrameCallbacker extends RenderProxyBox {
  _RenderPostFrameCallbacker({FrameCallback? callback}) : _callback = callback;

  FrameCallback? get callback => _callback;
  FrameCallback? _callback;
  set callback(FrameCallback? value) {
    _callback = value;
    if (value != null) {
      markNeedsPaint();
    }
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (callback != null) {
      SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
        callback!(duration);
        markNeedsPaint();
      });
    }
    super.paint(context, offset);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(FlagProperty('callback', value: callback != null, ifTrue: 'has a callback'));
  }
}

Future<ui.Image> _collateFrames(
  List<_AsyncImage> futureFrames,
  Size frameSize,
  int cellsPerRow,
) async {
  final List<ui.Image> frames = await _AsyncImage.resolveList(futureFrames);
  final int rowNum = (frames.length / cellsPerRow).ceil();

  final ui.PictureRecorder recorder = ui.PictureRecorder();
  final Canvas canvas = Canvas(
    recorder,
    Rect.fromLTWH(0, 0, frameSize.width * cellsPerRow, frameSize.height * rowNum),
  );
  for (int i = 0; i < frames.length; i += 1) {
    canvas.drawImage(
      frames[i],
      Offset(frameSize.width * (i % cellsPerRow), frameSize.height * (i / cellsPerRow).floor()),
      Paint(),
    );
  }
  final ui.Picture picture = recorder.endRecording();
  final ui.Image image = await picture.toImage(
    (frameSize.width * cellsPerRow).toInt(),
    (frameSize.height * rowNum).toInt(),
  );
  picture.dispose();
  return image;
}

class _RenderRootableRepaintBoundary extends RenderRepaintBoundary {
  // Like [toImage], but captures an image of all layers (composited by
  // RenderView and its children) clipped by the region of this object.
  Future<ui.Image> allLayersToImage() {
    final TransformLayer rootLayer = _rootLayer();
    final Matrix4 rootTransform = (rootLayer.transform ?? Matrix4.identity()).clone();
    final Matrix4 transform = rootTransform.multiplied(getTransformTo(null));
    final Rect rect = MatrixUtils.transformRect(transform, Offset.zero & size);
    // The scale was used to fit the actual device. Revert it since the target
    // is the logical display. Take transform[0] as the scale.
    return rootLayer.toImage(rect, pixelRatio: 1 / transform[0]);
  }

  TransformLayer _rootLayer() {
    Layer layer = this.layer!;
    while (layer.parent != null) {
      layer = layer.parent!;
    }
    return layer as TransformLayer;
  }
}

// A [RepaintBoundary], except that its render object has a `fullscreenToImage` method.
class _RootableRepaintBoundary extends SingleChildRenderObjectWidget {
  /// Creates a widget that isolates repaints.
  const _RootableRepaintBoundary({super.key, super.child});

  @override
  _RenderRootableRepaintBoundary createRenderObject(BuildContext context) =>
      _RenderRootableRepaintBoundary();
}
