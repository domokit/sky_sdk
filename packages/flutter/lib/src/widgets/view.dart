// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'basic.dart';
library;

import 'dart:collection';
import 'dart:ui'
    show FlutterView, SemanticsUpdate, ViewFocusDirection, ViewFocusEvent, ViewFocusState;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

import 'binding.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'focus_traversal.dart';
import 'framework.dart';
import 'lookup_boundary.dart';
import 'media_query.dart';

/// Bootstraps a render tree that is rendered into the provided [FlutterView].
///
/// The content rendered into that view is determined by the provided [child].
/// Descendants within the same [LookupBoundary] can look up the view they are
/// rendered into via [View.of] and [View.maybeOf].
///
/// The provided [child] is wrapped in a [MediaQuery] constructed from the given
/// [view], a [FocusScope], and a [RawView] widget.
///
/// For most use cases, using [MediaQuery.of], or its associated "...Of" methods
/// are a more appropriate way of obtaining the information that a [FlutterView]
/// exposes. For example, using [MediaQuery.sizeOf] will expose the _logical_
/// device size ([MediaQueryData.size]) rather than the physical size
/// ([FlutterView.physicalSize]). Similarly, while [FlutterView.padding] conveys
/// the information from the operating system, the [MediaQueryData.padding]
/// attribute (obtained from [MediaQuery.paddingOf]) further adjusts this
/// information to be aware of the context of the widget; e.g. the [Scaffold]
/// widget adjusts the values for its various children.
///
/// Each [FlutterView] can be associated with at most one [View] or [RawView]
/// widget in the widget tree. Two or more [View] or [RawView] widgets
/// configured with the same [FlutterView] must never exist within the same
/// widget tree at the same time. This limitation is enforced by a
/// [GlobalObjectKey] that derives its identity from the [view] provided to this
/// widget.
///
/// Since the [View] widget bootstraps its own independent render tree using its
/// embedded [RawView], neither it nor any of its descendants will insert a
/// [RenderObject] into an existing render tree. Therefore, the [View] widget
/// can only be used in those parts of the widget tree where it is not required
/// to participate in the construction of the surrounding render tree. In other
/// words, the widget may only be used in a non-rendering zone of the widget
/// tree (see [WidgetsBinding] for a definition of rendering and non-rendering
/// zones).
///
/// In practical terms, the widget is typically used at the root of the widget
/// tree outside of any other [View] or [RawView] widget, as a child of a
/// [ViewCollection] widget, or in the [ViewAnchor.view] slot of a [ViewAnchor]
/// widget. It is not required to be a direct child, though, since other
/// non-[RenderObjectWidget]s (e.g. [InheritedWidget]s, [Builder]s, or
/// [StatefulWidget]s/[StatelessWidget]s that only produce
/// non-[RenderObjectWidget]s) are allowed to be present between those widgets
/// and the [View] widget.
///
/// See also:
///
/// * [RawView], the workhorse that [View] uses to create the render tree, but
///   without the [MediaQuery] and [FocusScope] that [View] adds.
/// * [Element.debugExpectsRenderObjectForSlot], which defines whether a [View]
///   widget is allowed in a given child slot.
class View extends StatefulWidget {
  /// Create a [View] widget to bootstrap a render tree that is rendered into
  /// the provided [FlutterView].
  ///
  /// The content rendered into that [view] is determined by the given [child]
  /// widget.
  View({
    super.key,
    required this.view,
    @Deprecated(
      'Do not use. '
      'This parameter only exists to implement the deprecated RendererBinding.pipelineOwner property until it is removed. '
      'This feature was deprecated after v3.10.0-12.0.pre.',
    )
    PipelineOwner? deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner,
    @Deprecated(
      'Do not use. '
      'This parameter only exists to implement the deprecated RendererBinding.renderView property until it is removed. '
      'This feature was deprecated after v3.10.0-12.0.pre.',
    )
    RenderView? deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView,
    required this.child,
  }) : _deprecatedPipelineOwner = deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner,
       _deprecatedRenderView = deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView,
       assert(
         (deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner == null) ==
             (deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView == null),
       ),
       assert(
         deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView == null ||
             deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView.flutterView == view,
       );

  /// The [FlutterView] into which [child] is drawn.
  final FlutterView view;

  /// The widget below this widget in the tree, which will be drawn into the
  /// [view].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  final PipelineOwner? _deprecatedPipelineOwner;
  final RenderView? _deprecatedRenderView;

  /// Returns the [FlutterView] that the provided `context` will render into.
  ///
  /// Returns null if the `context` is not associated with a [FlutterView].
  ///
  /// The method creates a dependency on the `context`, which will be informed
  /// when the identity of the [FlutterView] changes (i.e. the `context` is
  /// moved to render into a different [FlutterView] then before). The context
  /// will not be informed when the _properties_ on the [FlutterView] itself
  /// change their values. To access the property values of a [FlutterView] it
  /// is best practice to use [MediaQuery.maybeOf] instead, which will ensure
  /// that the `context` is informed when the view properties change.
  ///
  /// See also:
  ///
  ///  * [View.of], which throws instead of returning null if no [FlutterView]
  ///    is found.
  static FlutterView? maybeOf(BuildContext context) {
    return LookupBoundary.dependOnInheritedWidgetOfExactType<_ViewScope>(context)?.view;
  }

  /// Returns the [FlutterView] that the provided `context` will render into.
  ///
  /// Throws if the `context` is not associated with a [FlutterView].
  ///
  /// The method creates a dependency on the `context`, which will be informed
  /// when the identity of the [FlutterView] changes (i.e. the `context` is
  /// moved to render into a different [FlutterView] then before). The context
  /// will not be informed when the _properties_ on the [FlutterView] itself
  /// change their values. To access the property values of a [FlutterView]
  /// prefer using the access methods on [MediaQuery], such as
  /// [MediaQuery.sizeOf], which will ensure that the `context` is informed when
  /// the view properties change.
  ///
  /// See also:
  ///
  ///  * [View.maybeOf], which throws instead of returning null if no
  ///    [FlutterView] is found.
  static FlutterView of(BuildContext context) {
    final FlutterView? result = maybeOf(context);
    assert(() {
      if (result == null) {
        final bool hiddenByBoundary =
            LookupBoundary.debugIsHidingAncestorWidgetOfExactType<_ViewScope>(context);
        final List<DiagnosticsNode> information = <DiagnosticsNode>[
          if (hiddenByBoundary) ...<DiagnosticsNode>[
            ErrorSummary(
              'View.of() was called with a context that does not have access to a View widget.',
            ),
            ErrorDescription(
              'The context provided to View.of() does have a View widget ancestor, but it is hidden by a LookupBoundary.',
            ),
          ] else ...<DiagnosticsNode>[
            ErrorSummary(
              'View.of() was called with a context that does not contain a View widget.',
            ),
            ErrorDescription(
              'No View widget ancestor could be found starting from the context that was passed to View.of().',
            ),
          ],
          ErrorDescription(
            'The context used was:\n'
            '  $context',
          ),
          ErrorHint('This usually means that the provided context is not associated with a View.'),
        ];
        throw FlutterError.fromParts(information);
      }
      return true;
    }());
    return result!;
  }

  /// Returns the [PipelineOwner] parent to which a child [View] should attach
  /// its [PipelineOwner] to.
  ///
  /// If `context` has a [View] ancestor, it returns the [PipelineOwner]
  /// responsible for managing the render tree of that view. If there is no
  /// [View] ancestor, [RendererBinding.rootPipelineOwner] is returned instead.
  static PipelineOwner pipelineOwnerOf(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<_PipelineOwnerScope>()?.pipelineOwner ??
        RendererBinding.instance.rootPipelineOwner;
  }

  @override
  State<View> createState() => _ViewState();
}

class _ViewState extends State<View> with WidgetsBindingObserver {
  final FocusScopeNode _scopeNode = FocusScopeNode(debugLabel: kReleaseMode ? null : 'View Scope');
  final FocusTraversalPolicy _policy = ReadingOrderTraversalPolicy();
  bool _viewHasFocus = false;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _scopeNode.addListener(_scopeFocusChangeListener);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _scopeNode.removeListener(_scopeFocusChangeListener);
    _scopeNode.dispose();
    super.dispose();
  }

  void _scopeFocusChangeListener() {
    if (_viewHasFocus == _scopeNode.hasFocus || !_scopeNode.hasFocus) {
      return;
    }
    // Scope has gained focus, and it doesn't match the view focus, so inform
    // the view so it knows to change its focus.
    WidgetsBinding.instance.platformDispatcher.requestViewFocusChange(
      direction: ViewFocusDirection.forward,
      state: ViewFocusState.focused,
      viewId: widget.view.viewId,
    );
  }

  @override
  void didChangeViewFocus(ViewFocusEvent event) {
    _viewHasFocus = switch (event.state) {
      ViewFocusState.focused => event.viewId == widget.view.viewId,
      ViewFocusState.unfocused => false,
    };
    if (event.viewId != widget.view.viewId) {
      return;
    }
    FocusNode nextFocus;
    switch (event.state) {
      case ViewFocusState.focused:
        switch (event.direction) {
          case ViewFocusDirection.forward:
            nextFocus = _policy.findFirstFocus(_scopeNode, ignoreCurrentFocus: true) ?? _scopeNode;
          case ViewFocusDirection.backward:
            nextFocus = _policy.findLastFocus(_scopeNode, ignoreCurrentFocus: true);
          case ViewFocusDirection.undefined:
            nextFocus = _scopeNode;
        }
        nextFocus.requestFocus();
      case ViewFocusState.unfocused:
        // Focusing on the root scope node will "park" the focus, so that no
        // descendant node will be given focus, and there's no widget that can
        // receive keyboard events.
        FocusManager.instance.rootScope.requestScopeFocus();
    }
  }

  @override
  Widget build(BuildContext context) {
    return RawView(
      view: widget.view,
      deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner: widget._deprecatedPipelineOwner,
      deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView: widget._deprecatedRenderView,
      child: MediaQuery.fromView(
        view: widget.view,
        child: FocusTraversalGroup(
          policy: _policy,
          child: FocusScope.withExternalFocusNode(
            includeSemantics: false,
            focusScopeNode: _scopeNode,
            child: widget.child,
          ),
        ),
      ),
    );
  }
}

/// The lower level workhorse widget for [View] that bootstraps a render tree
/// for a view.
///
/// Typically, the [View] widget is used instead of a [RawView] widget to create
/// a view, since, in addition to creating a view, it also adds some useful
/// widgets, such as a [MediaQuery] and [FocusScope]. The [RawView] widget is
/// only used directly if it is not desirable to have these additional widgets
/// around the resulting widget tree. The [View] widget uses the [RawView]
/// widget internally to manage its [FlutterView].
///
/// This widget can be used at the root of the widget tree outside of any other
/// [View] or [RawView] widget, as a child to a [ViewCollection], or in the
/// [ViewAnchor.view] slot of a [ViewAnchor] widget. It is not required to be a
/// direct child of those widgets; other non-[RenderObjectWidget]s may appear in
/// between the two (such as an [InheritedWidget]).
///
/// Each [FlutterView] can be associated with at most one [View] or [RawView]
/// widget in the widget tree. Two or more [View] or [RawView] widgets
/// configured with the same [FlutterView] must never exist within the same
/// widget tree at the same time. This limitation is enforced by a
/// [GlobalObjectKey] that derives its identity from the [view] provided to this
/// widget.
///
/// Since the [RawView] widget bootstraps its own independent render tree,
/// neither it nor any of its descendants will insert a [RenderObject] into an
/// existing render tree. Therefore, the [RawView] widget can only be used in
/// those parts of the widget tree where it is not required to participate in
/// the construction of the surrounding render tree. In other words, the widget
/// may only be used in a non-rendering zone of the widget tree (see
/// [WidgetsBinding] for a definition of rendering and non-rendering zones).
///
/// To find the [FlutterView] associated with a [BuildContext], use [View.of] or
/// [View.maybeOf], even if the view was created using [RawView] instead of
/// [View].
///
/// See also:
///
/// * [View] for a higher level interface that also sets up a [MediaQuery] and
///   [FocusScope] for the view's widget tree.
class RawView extends StatelessWidget {
  /// Creates a [RawView] widget.
  RawView({
    super.key,
    required this.view,
    @Deprecated(
      'Do not use. '
      'This parameter only exists to implement the deprecated RendererBinding.pipelineOwner property until it is removed. '
      'This feature was deprecated after v3.10.0-12.0.pre.',
    )
    PipelineOwner? deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner,
    @Deprecated(
      'Do not use. '
      'This parameter only exists to implement the deprecated RendererBinding.renderView property until it is removed. '
      'This feature was deprecated after v3.10.0-12.0.pre.',
    )
    RenderView? deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView,
    required this.child,
  }) : _deprecatedPipelineOwner = deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner,
       _deprecatedRenderView = deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView,
       assert(
         (deprecatedDoNotUseWillBeRemovedWithoutNoticePipelineOwner == null) ==
             (deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView == null),
       ),
       assert(
         deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView == null ||
             deprecatedDoNotUseWillBeRemovedWithoutNoticeRenderView.flutterView == view,
       );

  /// The [FlutterView] into which [child] is drawn.
  final FlutterView view;

  /// The widget below this widget in the tree, which will be drawn into the
  /// [view].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  final PipelineOwner? _deprecatedPipelineOwner;
  final RenderView? _deprecatedRenderView;

  @override
  Widget build(BuildContext context) {
    return _RawViewInternal(
      view: view,
      deprecatedPipelineOwner: _deprecatedPipelineOwner,
      deprecatedRenderView: _deprecatedRenderView,
      builder: (BuildContext context, PipelineOwner owner) {
        return _ViewScope(
          view: view,
          child: _PipelineOwnerScope(pipelineOwner: owner, child: child),
        );
      },
    );
  }
}

/// A builder for the content [Widget] of a [_RawViewInternal].
///
/// The widget returned by the builder defines the content that is drawn into
/// the [FlutterView] configured on the [_RawViewInternal].
///
/// The builder is given the [PipelineOwner] that the [_RawViewInternal] uses to
/// manage its render tree. Typical builder implementations make that pipeline
/// owner available as an attachment point for potential child views.
///
/// Used by [_RawViewInternal.builder].
typedef _RawViewContentBuilder = Widget Function(BuildContext context, PipelineOwner owner);

/// The workhorse behind the [RawView] widget that actually bootstraps a render
/// tree.
///
/// It instantiates the [RenderView] as the root of that render tree and adds it
/// to the [RendererBinding] via [RendererBinding.addRenderView]. It also owns
/// the [PipelineOwner] that manages this render tree and adds it as a child to
/// the surrounding parent [PipelineOwner] obtained with [View.pipelineOwnerOf].
/// This ensures that the render tree bootstrapped by this widget participates
/// properly in frame production and hit testing.
class _RawViewInternal extends RenderObjectWidget {
  /// Create a [_RawViewInternal] widget to bootstrap a render tree that is
  /// rendered into the provided [FlutterView].
  ///
  /// The content rendered into that [view] is determined by the [Widget]
  /// returned by [builder].
  _RawViewInternal({
    required this.view,
    required PipelineOwner? deprecatedPipelineOwner,
    required RenderView? deprecatedRenderView,
    required this.builder,
  }) : _deprecatedPipelineOwner = deprecatedPipelineOwner,
       _deprecatedRenderView = deprecatedRenderView,
       assert(deprecatedRenderView == null || deprecatedRenderView.flutterView == view),
       // TODO(goderbauer): Replace this with GlobalObjectKey(view) when the deprecated properties are removed.
       super(key: _DeprecatedRawViewKey(view, deprecatedPipelineOwner, deprecatedRenderView));

  /// The [FlutterView] into which the [Widget] returned by [builder] is drawn.
  final FlutterView view;

  /// Determines the content [Widget] that is drawn into the [view].
  ///
  /// The [builder] is given the [PipelineOwner] responsible for the render tree
  /// bootstrapped by this widget and typically makes it available as an
  /// attachment point for potential child views.
  final _RawViewContentBuilder builder;

  final PipelineOwner? _deprecatedPipelineOwner;
  final RenderView? _deprecatedRenderView;

  @override
  RenderObjectElement createElement() => _RawViewElement(this);

  @override
  RenderObject createRenderObject(BuildContext context) {
    return _deprecatedRenderView ?? RenderView(view: view);
  }

  // No need to implement updateRenderObject: RawView uses the view as a
  // GlobalKey, so we never need to update the RenderObject with a new view.
}

class _RawViewElement extends RenderTreeRootElement {
  _RawViewElement(super.widget);

  late final PipelineOwner _pipelineOwner = PipelineOwner(
    onSemanticsOwnerCreated: _handleSemanticsOwnerCreated,
    onSemanticsUpdate: _handleSemanticsUpdate,
    onSemanticsOwnerDisposed: _handleSemanticsOwnerDisposed,
  );

  PipelineOwner get _effectivePipelineOwner =>
      (widget as _RawViewInternal)._deprecatedPipelineOwner ?? _pipelineOwner;

  void _handleSemanticsOwnerCreated() {
    (_effectivePipelineOwner.rootNode as RenderView?)?.scheduleInitialSemantics();
  }

  void _handleSemanticsOwnerDisposed() {
    (_effectivePipelineOwner.rootNode as RenderView?)?.clearSemantics();
  }

  void _handleSemanticsUpdate(SemanticsUpdate update) {
    (widget as _RawViewInternal).view.updateSemantics(update);
  }

  @override
  RenderView get renderObject => super.renderObject as RenderView;

  Element? _child;

  void _updateChild() {
    try {
      final Widget child = (widget as _RawViewInternal).builder(this, _effectivePipelineOwner);
      _child = updateChild(_child, child, null);
    } catch (e, stack) {
      final FlutterErrorDetails details = FlutterErrorDetails(
        exception: e,
        stack: stack,
        library: 'widgets library',
        context: ErrorDescription('building $this'),
        informationCollector:
            !kDebugMode
                ? null
                : () => <DiagnosticsNode>[DiagnosticsDebugCreator(DebugCreator(this))],
      );
      FlutterError.reportError(details);
      final Widget error = ErrorWidget.builder(details);
      _child = updateChild(null, error, slot);
    }
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot);
    assert(_effectivePipelineOwner.rootNode == null);
    _effectivePipelineOwner.rootNode = renderObject;
    _attachView();
    _updateChild();
    renderObject.prepareInitialFrame();
    if (_effectivePipelineOwner.semanticsOwner != null) {
      renderObject.scheduleInitialSemantics();
    }
  }

  PipelineOwner? _parentPipelineOwner; // Is null if view is currently not attached.

  void _attachView([PipelineOwner? parentPipelineOwner]) {
    assert(_parentPipelineOwner == null);
    parentPipelineOwner ??= View.pipelineOwnerOf(this);
    parentPipelineOwner.adoptChild(_effectivePipelineOwner);
    RendererBinding.instance.addRenderView(renderObject);
    _parentPipelineOwner = parentPipelineOwner;
  }

  void _detachView() {
    final PipelineOwner? parentPipelineOwner = _parentPipelineOwner;
    if (parentPipelineOwner != null) {
      RendererBinding.instance.removeRenderView(renderObject);
      parentPipelineOwner.dropChild(_effectivePipelineOwner);
      _parentPipelineOwner = null;
    }
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (_parentPipelineOwner == null) {
      return;
    }
    final PipelineOwner newParentPipelineOwner = View.pipelineOwnerOf(this);
    if (newParentPipelineOwner != _parentPipelineOwner) {
      _detachView();
      _attachView(newParentPipelineOwner);
    }
  }

  @override
  void performRebuild() {
    super.performRebuild();
    _updateChild();
  }

  @override
  void activate() {
    super.activate();
    assert(_effectivePipelineOwner.rootNode == null);
    _effectivePipelineOwner.rootNode = renderObject;
    _attachView();
  }

  @override
  void deactivate() {
    _detachView();
    assert(_effectivePipelineOwner.rootNode == renderObject);
    _effectivePipelineOwner.rootNode = null; // To satisfy the assert in the super class.
    super.deactivate();
  }

  @override
  void update(_RawViewInternal newWidget) {
    super.update(newWidget);
    _updateChild();
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
  void insertRenderObjectChild(RenderBox child, Object? slot) {
    assert(slot == null);
    assert(renderObject.debugValidateChild(child));
    renderObject.child = child;
  }

  @override
  void moveRenderObjectChild(RenderObject child, Object? oldSlot, Object? newSlot) {
    assert(false);
  }

  @override
  void removeRenderObjectChild(RenderObject child, Object? slot) {
    assert(slot == null);
    assert(renderObject.child == child);
    renderObject.child = null;
  }

  @override
  void unmount() {
    if (_effectivePipelineOwner != (widget as _RawViewInternal)._deprecatedPipelineOwner) {
      _effectivePipelineOwner.dispose();
    }
    super.unmount();
  }
}

class _ViewScope extends InheritedWidget {
  const _ViewScope({required this.view, required super.child});

  final FlutterView view;

  @override
  bool updateShouldNotify(_ViewScope oldWidget) => view != oldWidget.view;
}

class _PipelineOwnerScope extends InheritedWidget {
  const _PipelineOwnerScope({required this.pipelineOwner, required super.child});

  final PipelineOwner pipelineOwner;

  @override
  bool updateShouldNotify(_PipelineOwnerScope oldWidget) =>
      pipelineOwner != oldWidget.pipelineOwner;
}

class _MultiChildComponentWidget extends Widget {
  const _MultiChildComponentWidget({
    super.key,
    List<Widget> views = const <Widget>[],
    Widget? child,
  }) : _views = views,
       _child = child;

  // It is up to the subclasses to make the relevant properties public.
  final List<Widget> _views;
  final Widget? _child;

  @override
  Element createElement() => _MultiChildComponentElement(this);
}

/// A collection of sibling [View]s.
///
/// This widget can only be used in places were a [View] widget is allowed, i.e.
/// in a non-rendering zone of the widget tree. In practical terms, it can be
/// used at the root of the widget tree outside of any [View] widget, as a child
/// to a another [ViewCollection], or in the [ViewAnchor.view] slot of a
/// [ViewAnchor] widget. It is not required to be a direct child of those
/// widgets; other non-[RenderObjectWidget]s may appear in between the two (such
/// as an [InheritedWidget]).
///
/// Similarly, the [views] children of this widget must be [View]s, but they
/// may be wrapped in additional non-[RenderObjectWidget]s (e.g.
/// [InheritedWidget]s).
///
/// See also:
///
///  * [WidgetsBinding] for an explanation of rendering and non-rendering zones.
class ViewCollection extends _MultiChildComponentWidget {
  /// Creates a [ViewCollection] widget.
  const ViewCollection({super.key, required super.views});

  /// The [View] descendants of this widget.
  ///
  /// The [View]s may be wrapped in other non-[RenderObjectWidget]s (e.g.
  /// [InheritedWidget]s). However, no [RenderObjectWidget] is allowed to appear
  /// between the [ViewCollection] and the next [View] widget.
  List<Widget> get views => _views;
}

/// Decorates a [child] widget with a side [View].
///
/// This widget must have a [View] ancestor, into which the [child] widget
/// is rendered.
///
/// Typically, a [View] or [ViewCollection] widget is used in the [view] slot to
/// define the content of the side view(s). Those widgets may be wrapped in
/// other non-[RenderObjectWidget]s (e.g. [InheritedWidget]s). However, no
/// [RenderObjectWidget] is allowed to appear between the [ViewAnchor] and the
/// next [View] widget in the [view] slot. The widgets in the [view] slot have
/// access to all [InheritedWidget]s above the [ViewAnchor] in the tree.
///
/// In technical terms, the [ViewAnchor] can only be used in a rendering zone of
/// the widget tree and the [view] slot marks the start of a new non-rendering
/// zone (see [WidgetsBinding] for a definition of these zones). Typically,
/// it is occupied by a [View] widget, which will start a new rendering zone.
///
/// {@template flutter.widgets.ViewAnchor}
/// An example use case for this widget is a tooltip for a button. The tooltip
/// should be able to extend beyond the bounds of the main view. For this, the
/// tooltip can be implemented as a separate [View], which is anchored to the
/// button in the main view by wrapping that button with a [ViewAnchor]. In this
/// example, the [view] slot is configured with the tooltip [View] and the
/// [child] is the button widget rendered into the surrounding view.
/// {@endtemplate}
class ViewAnchor extends StatelessWidget {
  /// Creates a [ViewAnchor] widget.
  const ViewAnchor({super.key, this.view, required this.child});

  /// The widget that defines the view anchored to this widget.
  ///
  /// Typically, a [View] or [ViewCollection] widget is used, which may be
  /// wrapped in other non-[RenderObjectWidget]s (e.g. [InheritedWidget]s).
  ///
  /// {@macro flutter.widgets.ViewAnchor}
  final Widget? view;

  /// The widget below this widget in the tree.
  ///
  /// It is rendered into the surrounding view, not in the view defined by
  /// [view].
  ///
  /// {@macro flutter.widgets.ViewAnchor}
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return _MultiChildComponentWidget(
      views: <Widget>[if (view != null) LookupBoundary(child: view!)],
      child: child,
    );
  }
}

class _MultiChildComponentElement extends Element {
  _MultiChildComponentElement(super.widget);

  List<Element> _viewElements = <Element>[];
  final Set<Element> _forgottenViewElements = HashSet<Element>();
  Element? _childElement;

  bool _debugAssertChildren() {
    final _MultiChildComponentWidget typedWidget = widget as _MultiChildComponentWidget;
    // Each view widget must have a corresponding element.
    assert(_viewElements.length == typedWidget._views.length);
    // Iff there is a child widget, it must have a corresponding element.
    assert((_childElement == null) == (typedWidget._child == null));
    // The child element is not also a view element.
    assert(!_viewElements.contains(_childElement));
    return true;
  }

  @override
  void attachRenderObject(Object? newSlot) {
    super.attachRenderObject(newSlot);
    assert(_debugCheckMustAttachRenderObject(newSlot));
  }

  @override
  void mount(Element? parent, Object? newSlot) {
    super.mount(parent, newSlot);
    assert(_debugCheckMustAttachRenderObject(newSlot));
    assert(_viewElements.isEmpty);
    assert(_childElement == null);
    rebuild();
    assert(_debugAssertChildren());
  }

  @override
  void updateSlot(Object? newSlot) {
    super.updateSlot(newSlot);
    assert(_debugCheckMustAttachRenderObject(newSlot));
  }

  bool _debugCheckMustAttachRenderObject(Object? slot) {
    // Check only applies in the ViewCollection configuration.
    if (!kDebugMode || (widget as _MultiChildComponentWidget)._child != null) {
      return true;
    }
    bool hasAncestorRenderObjectElement = false;
    bool ancestorWantsRenderObject = true;
    visitAncestorElements((Element ancestor) {
      if (!ancestor.debugExpectsRenderObjectForSlot(slot)) {
        ancestorWantsRenderObject = false;
        return false;
      }
      if (ancestor is RenderObjectElement) {
        hasAncestorRenderObjectElement = true;
        return false;
      }
      return true;
    });
    if (hasAncestorRenderObjectElement && ancestorWantsRenderObject) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'The Element for ${toStringShort()} cannot be inserted into slot "$slot" of its ancestor. ',
            ),
            ErrorDescription(
              'The ownership chain for the Element in question was:\n  ${debugGetCreatorChain(10)}',
            ),
            ErrorDescription(
              'This Element allows the creation of multiple independent render trees, which cannot '
              'be attached to an ancestor in an existing render tree. However, an ancestor RenderObject '
              'is expecting that a child will be attached.',
            ),
            ErrorHint(
              'Try moving the subtree that contains the ${toStringShort()} widget into the '
              'view property of a ViewAnchor widget or to the root of the widget tree, where '
              'it is not expected to attach its RenderObject to its ancestor.',
            ),
          ]),
        ),
      );
    }
    return true;
  }

  @override
  void update(_MultiChildComponentWidget newWidget) {
    // Cannot switch from ViewAnchor config to ViewCollection config.
    assert((newWidget._child == null) == ((widget as _MultiChildComponentWidget)._child == null));
    super.update(newWidget);
    rebuild(force: true);
    assert(_debugAssertChildren());
  }

  static const Object _viewSlot = Object();

  @override
  bool debugExpectsRenderObjectForSlot(Object? slot) => slot != _viewSlot;

  @override
  void performRebuild() {
    final _MultiChildComponentWidget typedWidget = widget as _MultiChildComponentWidget;

    _childElement = updateChild(_childElement, typedWidget._child, slot);

    final List<Widget> views = typedWidget._views;
    _viewElements = updateChildren(
      _viewElements,
      views,
      forgottenChildren: _forgottenViewElements,
      slots: List<Object>.generate(views.length, (_) => _viewSlot),
    );
    _forgottenViewElements.clear();

    super.performRebuild(); // clears the dirty flag
    assert(_debugAssertChildren());
  }

  @override
  void forgetChild(Element child) {
    if (child == _childElement) {
      _childElement = null;
    } else {
      assert(_viewElements.contains(child));
      assert(!_forgottenViewElements.contains(child));
      _forgottenViewElements.add(child);
    }
    super.forgetChild(child);
  }

  @override
  void visitChildren(ElementVisitor visitor) {
    if (_childElement != null) {
      visitor(_childElement!);
    }
    for (final Element child in _viewElements) {
      if (!_forgottenViewElements.contains(child)) {
        visitor(child);
      }
    }
  }

  @override
  bool get debugDoingBuild => false; // This element does not have a concept of "building".

  @override
  Element? get renderObjectAttachingChild => _childElement;

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return <DiagnosticsNode>[
      if (_childElement != null) _childElement!.toDiagnosticsNode(),
      for (int i = 0; i < _viewElements.length; i++)
        _viewElements[i].toDiagnosticsNode(
          name: 'view ${i + 1}',
          style: DiagnosticsTreeStyle.offstage,
        ),
    ];
  }
}

// A special [GlobalKey] to support passing the deprecated
// [RendererBinding.renderView] and [RendererBinding.pipelineOwner] to the
// [_RawView]. Will be removed when those deprecated properties are removed.
@optionalTypeArgs
class _DeprecatedRawViewKey<T extends State<StatefulWidget>> extends GlobalKey<T> {
  const _DeprecatedRawViewKey(this.view, this.owner, this.renderView) : super.constructor();

  final FlutterView view;
  final PipelineOwner? owner;
  final RenderView? renderView;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _DeprecatedRawViewKey<T> &&
        identical(other.view, view) &&
        identical(other.owner, owner) &&
        identical(other.renderView, renderView);
  }

  @override
  int get hashCode => Object.hash(view, owner, renderView);

  @override
  String toString() => '[_DeprecatedRawViewKey ${describeIdentity(view)}]';
}
