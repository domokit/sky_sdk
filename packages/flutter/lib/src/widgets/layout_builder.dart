// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'basic.dart';
/// @docImport 'single_child_scroll_view.dart';
/// @docImport 'sliver_layout_builder.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';

import 'debug.dart';
import 'framework.dart';

/// The signature of the [LayoutBuilder] builder function.
typedef LayoutWidgetBuilder = Widget Function(BuildContext context, BoxConstraints constraints);

/// An abstract superclass for widgets that defer their building until layout.
///
/// Similar to the [Builder] widget except that the framework calls the [builder]
/// function at layout time and provides the constraints that this widget should
/// adhere to. This is useful when the parent constrains the child's size and layout,
/// and doesn't depend on the child's intrinsic size.
///
/// {@template flutter.widgets.ConstrainedLayoutBuilder}
/// The [builder] function is called in the following situations:
///
/// * The first time the widget is laid out.
/// * When the parent widget passes different layout constraints.
/// * When the parent widget updates this widget.
/// * When the dependencies that the [builder] function subscribes to change.
///
/// The [builder] function is _not_ called during layout if the parent passes
/// the same constraints repeatedly.
/// {@endtemplate}
///
/// Subclasses must return a [RenderObject] that mixes in
/// [RenderConstrainedLayoutBuilder].
abstract class ConstrainedLayoutBuilder<ConstraintType extends Constraints>
    extends RenderObjectWidget {
  /// Creates a widget that defers its building until layout.
  const ConstrainedLayoutBuilder({super.key, required this.builder});

  @override
  RenderObjectElement createElement() => _LayoutBuilderElement<ConstraintType>(this);

  /// Called at layout time to construct the widget tree.
  ///
  /// The builder must not return null.
  final Widget Function(BuildContext context, ConstraintType constraints) builder;

  /// Whether [builder] needs to be called again even if the layout constraints
  /// are the same.
  ///
  /// When this widget's configuration is updated, the [builder] callback most
  /// likely needs to be called to build this widget's child. However,
  /// subclasses may provide ways in which the widget can be updated without
  /// needing to rebuild the child. Such subclasses can use this method to tell
  /// the framework when the child widget should be rebuilt.
  ///
  /// When this method is called by the framework, the newly configured widget
  /// is asked if it requires a rebuild, and it is passed the old widget as a
  /// parameter.
  ///
  /// See also:
  ///
  ///  * [State.setState] and [State.didUpdateWidget], which talk about widget
  ///    configuration changes and how they're triggered.
  ///  * [Element.update], the method that actually updates the widget's
  ///    configuration.
  @protected
  bool updateShouldRebuild(covariant ConstrainedLayoutBuilder<ConstraintType> oldWidget) => true;

  // updateRenderObject is redundant with the logic in the LayoutBuilderElement below.
}

class _LayoutBuilderElement<ConstraintType extends Constraints> extends RenderObjectElement {
  _LayoutBuilderElement(ConstrainedLayoutBuilder<ConstraintType> super.widget);

  @override
  RenderConstrainedLayoutBuilder<ConstraintType, RenderObject> get renderObject =>
      super.renderObject as RenderConstrainedLayoutBuilder<ConstraintType, RenderObject>;

  Element? _child;

  @override
  BuildScope get buildScope => _buildScope;

  late final BuildScope _buildScope = BuildScope(scheduleRebuild: _scheduleRebuild);

  // To schedule a rebuild, markNeedsLayout needs to be called on this Element's
  // render object (as the rebuilding is done in its performLayout call). However,
  // the render tree should typically be kept clean during the postFrameCallbacks
  // and the idle phase, so the layout data can be safely read.
  bool _deferredCallbackScheduled = false;
  void _scheduleRebuild() {
    if (_deferredCallbackScheduled) {
      return;
    }

    final bool deferMarkNeedsLayout = switch (SchedulerBinding.instance.schedulerPhase) {
      SchedulerPhase.idle || SchedulerPhase.postFrameCallbacks => true,
      SchedulerPhase.transientCallbacks ||
      SchedulerPhase.midFrameMicrotasks ||
      SchedulerPhase.persistentCallbacks => false,
    };
    if (!deferMarkNeedsLayout) {
      renderObject.markNeedsLayout();
      return;
    }
    _deferredCallbackScheduled = true;
    SchedulerBinding.instance.scheduleFrameCallback(_frameCallback);
  }

  void _frameCallback(Duration timestamp) {
    _deferredCallbackScheduled = false;
    // This method is only called when the render tree is stable, if the Element
    // is deactivated it will never be reincorporated back to the tree.
    if (mounted) {
      renderObject.markNeedsLayout();
    }
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    if (_child != null) {
      visitor(_child!);
    }
  }

  @override
  void forgetChild(Element child) {
    assert(child == _child);
    _child = null;
    super.forgetChild(child);
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot); // Creates the renderObject.
    renderObject.updateCallback(_rebuildWithConstraints);
  }

  @override
  void update(ConstrainedLayoutBuilder<ConstraintType> newWidget) {
    assert(widget != newWidget);
    final ConstrainedLayoutBuilder<ConstraintType> oldWidget =
        widget as ConstrainedLayoutBuilder<ConstraintType>;
    super.update(newWidget);
    assert(widget == newWidget);

    renderObject.updateCallback(_rebuildWithConstraints);
    if (newWidget.updateShouldRebuild(oldWidget)) {
      _needsBuild = true;
      renderObject.markNeedsLayout();
    }
  }

  @override
  void markNeedsBuild() {
    // Calling super.markNeedsBuild is not needed. This Element does not need
    // to performRebuild since this call already does what performRebuild does,
    // So the element is clean as soon as this method returns and does not have
    // to be added to the dirty list or marked as dirty.
    renderObject.markNeedsLayout();
    _needsBuild = true;
  }

  @override
  void performRebuild() {
    // This gets called if markNeedsBuild() is called on us.
    // That might happen if, e.g., our builder uses Inherited widgets.

    // Force the callback to be called, even if the layout constraints are the
    // same. This is because that callback may depend on the updated widget
    // configuration, or an inherited widget.
    renderObject.markNeedsLayout();
    _needsBuild = true;
    super.performRebuild(); // Calls widget.updateRenderObject (a no-op in this case).
  }

  @override
  void unmount() {
    renderObject.updateCallback(null);
    super.unmount();
  }

  // The constraints that were passed to this class last time it was laid out.
  // These constraints are compared to the new constraints to determine whether
  // [ConstrainedLayoutBuilder.builder] needs to be called.
  ConstraintType? _previousConstraints;
  bool _needsBuild = true;

  void _rebuildWithConstraints(ConstraintType constraints) {
    @pragma('vm:notify-debugger-on-exception')
    void updateChildCallback() {
      Widget built;
      try {
        built = (widget as ConstrainedLayoutBuilder<ConstraintType>).builder(this, constraints);
        debugWidgetBuilderValue(widget, built);
      } catch (e, stack) {
        built = ErrorWidget.builder(
          _reportException(
            ErrorDescription('building $widget'),
            e,
            stack,
            informationCollector:
                () => <DiagnosticsNode>[
                  if (kDebugMode) DiagnosticsDebugCreator(DebugCreator(this)),
                ],
          ),
        );
      }
      try {
        _child = updateChild(_child, built, null);
        assert(_child != null);
      } catch (e, stack) {
        built = ErrorWidget.builder(
          _reportException(
            ErrorDescription('building $widget'),
            e,
            stack,
            informationCollector:
                () => <DiagnosticsNode>[
                  if (kDebugMode) DiagnosticsDebugCreator(DebugCreator(this)),
                ],
          ),
        );
        _child = updateChild(null, built, slot);
      } finally {
        _needsBuild = false;
        _previousConstraints = constraints;
      }
    }

    final VoidCallback? callback =
        _needsBuild || (constraints != _previousConstraints) ? updateChildCallback : null;
    owner!.buildScope(this, callback);
  }

  @override
  void insertRenderObjectChild(RenderObject child, Object? slot) {
    final RenderObjectWithChildMixin<RenderObject> renderObject = this.renderObject;
    assert(slot == null);
    assert(renderObject.debugValidateChild(child));
    renderObject.child = child;
    assert(renderObject == this.renderObject);
  }

  @override
  void moveRenderObjectChild(RenderObject child, Object? oldSlot, Object? newSlot) {
    assert(false);
  }

  @override
  void removeRenderObjectChild(RenderObject child, Object? slot) {
    final RenderConstrainedLayoutBuilder<ConstraintType, RenderObject> renderObject =
        this.renderObject;
    assert(renderObject.child == child);
    renderObject.child = null;
    assert(renderObject == this.renderObject);
  }
}

/// Generic mixin for [RenderObject]s created by [ConstrainedLayoutBuilder].
///
/// Provides a callback that should be called at layout time, typically in
/// [RenderObject.performLayout].
mixin RenderConstrainedLayoutBuilder<
  ConstraintType extends Constraints,
  ChildType extends RenderObject
>
    on RenderObjectWithChildMixin<ChildType> {
  LayoutCallback<ConstraintType>? _callback;

  /// Change the layout callback.
  void updateCallback(LayoutCallback<ConstraintType>? value) {
    if (value == _callback) {
      return;
    }
    _callback = value;
    markNeedsLayout();
  }

  /// Invoke the callback supplied via [updateCallback].
  ///
  /// Typically this results in [ConstrainedLayoutBuilder.builder] being called
  /// during layout.
  void rebuildIfNecessary() {
    assert(_callback != null);
    invokeLayoutCallback(_callback!);
  }
}

/// Builds a widget tree that can depend on the parent widget's size.
///
/// Similar to the [Builder] widget except that the framework calls the [builder]
/// function at layout time and provides the parent widget's constraints. This
/// is useful when the parent constrains the child's size and doesn't depend on
/// the child's intrinsic size. The [LayoutBuilder]'s final size will match its
/// child's size.
///
/// {@macro flutter.widgets.ConstrainedLayoutBuilder}
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=IYDVcriKjsw}
///
/// If the child should be smaller than the parent, consider wrapping the child
/// in an [Align] widget. If the child might want to be bigger, consider
/// wrapping it in a [SingleChildScrollView] or [OverflowBox].
///
/// {@tool dartpad}
/// This example uses a [LayoutBuilder] to build a different widget depending on the available width. Resize the
/// DartPad window to see [LayoutBuilder] in action!
///
/// ** See code in examples/api/lib/widgets/layout_builder/layout_builder.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SliverLayoutBuilder], the sliver counterpart of this widget.
///  * [Builder], which calls a `builder` function at build time.
///  * [StatefulBuilder], which passes its `builder` function a `setState` callback.
///  * [CustomSingleChildLayout], which positions its child during layout.
///  * The [catalog of layout widgets](https://flutter.dev/widgets/layout/).
class LayoutBuilder extends ConstrainedLayoutBuilder<BoxConstraints> {
  /// Creates a widget that defers its building until layout.
  const LayoutBuilder({super.key, required super.builder});

  @override
  RenderObject createRenderObject(BuildContext context) => _RenderLayoutBuilder();
}

class _RenderLayoutBuilder extends RenderBox
    with
        RenderObjectWithChildMixin<RenderBox>,
        RenderConstrainedLayoutBuilder<BoxConstraints, RenderBox> {
  @override
  double computeMinIntrinsicWidth(double height) {
    assert(_debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    assert(_debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMinIntrinsicHeight(double width) {
    assert(_debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    assert(_debugThrowIfNotCheckingIntrinsics());
    return 0.0;
  }

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    assert(
      debugCannotComputeDryLayout(
        reason:
            'Calculating the dry layout would require running the layout callback '
            'speculatively, which might mutate the live render object tree.',
      ),
    );
    return Size.zero;
  }

  @override
  double? computeDryBaseline(BoxConstraints constraints, TextBaseline baseline) {
    assert(
      debugCannotComputeDryLayout(
        reason:
            'Calculating the dry baseline would require running the layout callback '
            'speculatively, which might mutate the live render object tree.',
      ),
    );
    return null;
  }

  @override
  void performLayout() {
    final BoxConstraints constraints = this.constraints;
    rebuildIfNecessary();
    if (child != null) {
      child!.layout(constraints, parentUsesSize: true);
      size = constraints.constrain(child!.size);
    } else {
      size = constraints.biggest;
    }
  }

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return child?.getDistanceToActualBaseline(baseline) ??
        super.computeDistanceToActualBaseline(baseline);
  }

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return child?.hitTest(result, position: position) ?? false;
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    if (child != null) {
      context.paintChild(child!, offset);
    }
  }

  bool _debugThrowIfNotCheckingIntrinsics() {
    assert(() {
      if (!RenderObject.debugCheckingIntrinsics) {
        throw FlutterError(
          'LayoutBuilder does not support returning intrinsic dimensions.\n'
          'Calculating the intrinsic dimensions would require running the layout '
          'callback speculatively, which might mutate the live render object tree.',
        );
      }
      return true;
    }());

    return true;
  }
}

FlutterErrorDetails _reportException(
  DiagnosticsNode context,
  Object exception,
  StackTrace stack, {
  InformationCollector? informationCollector,
}) {
  final FlutterErrorDetails details = FlutterErrorDetails(
    exception: exception,
    stack: stack,
    library: 'widgets library',
    context: context,
    informationCollector: informationCollector,
  );
  FlutterError.reportError(details);
  return details;
}
