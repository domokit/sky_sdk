// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/semantics.dart';
///
/// @docImport 'actions.dart';
/// @docImport 'container.dart';
/// @docImport 'editable_text.dart';
/// @docImport 'focus_traversal.dart';
/// @docImport 'overlay.dart';
library;

import 'package:flutter/foundation.dart';

import 'basic.dart';
import 'focus_manager.dart';
import 'framework.dart';
import 'inherited_notifier.dart';

/// A widget that manages a [FocusNode] to allow keyboard focus to be given
/// to this widget and its descendants.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=JCDfh5bs1xc}
///
/// When the focus is gained or lost, [onFocusChange] is called.
///
/// For keyboard events, [onKey] and [onKeyEvent] are called if
/// [FocusNode.hasFocus] is true for this widget's [focusNode], unless a focused
/// descendant's [onKey] or [onKeyEvent] callback returned
/// [KeyEventResult.handled] when called.
///
/// This widget does not provide any visual indication that the focus has
/// changed. Any desired visual changes should be made when [onFocusChange] is
/// called.
///
/// To access the [FocusNode] of the nearest ancestor [Focus] widget and
/// establish a relationship that will rebuild the widget when the focus
/// changes, use the [Focus.of] and [FocusScope.of] static methods.
///
/// To access the focused state of the nearest [Focus] widget, use
/// [FocusNode.hasFocus] from a build method, which also establishes a
/// relationship between the calling widget and the [Focus] widget that will
/// rebuild the calling widget when the focus changes.
///
/// Managing a [FocusNode] means managing its lifecycle, listening for changes
/// in focus, and re-parenting it when needed to keep the focus hierarchy in
/// sync with the widget hierarchy. This widget does all of those things for
/// you. See [FocusNode] for more information about the details of what node
/// management entails if you are not using a [Focus] widget and you need to do
/// it yourself.
///
/// If the [Focus] default constructor is used, then this widget will manage any
/// given [focusNode] by overwriting the appropriate values of the [focusNode]
/// with the values of [FocusNode.onKey], [FocusNode.onKeyEvent],
/// [FocusNode.skipTraversal], [FocusNode.canRequestFocus], and
/// [FocusNode.descendantsAreFocusable] whenever the [Focus] widget is updated.
///
/// If the [Focus.withExternalFocusNode] is used instead, then the values
/// returned by [onKey], [onKeyEvent], [skipTraversal], [canRequestFocus], and
/// [descendantsAreFocusable] will be the values in the external focus node, and
/// the external focus node's values will not be overwritten when the widget is
/// updated.
///
/// To collect a sub-tree of nodes into an exclusive group that restricts focus
/// traversal to the group, use a [FocusScope]. To collect a sub-tree of nodes
/// into a group that has a specific order to its traversal but allows the
/// traversal to escape the group, use a [FocusTraversalGroup].
///
/// To move the focus, use methods on [FocusNode] by getting the [FocusNode]
/// through the [of] method. For instance, to move the focus to the next node in
/// the focus traversal order, call `Focus.of(context).nextFocus()`. To unfocus
/// a widget, call `Focus.of(context).unfocus()`.
///
/// {@tool dartpad}
/// This example shows how to manage focus using the [Focus] and [FocusScope]
/// widgets. See [FocusNode] for a similar example that doesn't use [Focus] or
/// [FocusScope].
///
/// ** See code in examples/api/lib/widgets/focus_scope/focus.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to wrap another widget in a [Focus] widget to make it
/// focusable. It wraps a [Container], and changes the container's color when it
/// is set as the [FocusManager.primaryFocus].
///
/// If you also want to handle mouse hover and/or keyboard actions on a widget,
/// consider using a [FocusableActionDetector], which combines several different
/// widgets to provide those capabilities.
///
/// ** See code in examples/api/lib/widgets/focus_scope/focus.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to focus a newly-created widget immediately after it
/// is created.
///
/// The focus node will not actually be given the focus until after the frame in
/// which it has requested focus is drawn, so it is OK to call
/// [FocusNode.requestFocus] on a node which is not yet in the focus tree.
///
/// ** See code in examples/api/lib/widgets/focus_scope/focus.2.dart **
/// {@end-tool}
///
/// See also:
///
///  * [FocusNode], which represents a node in the focus hierarchy and
///    [FocusNode]'s API documentation includes a detailed explanation of its role
///    in the overall focus system.
///  * [FocusScope], a widget that manages a group of focusable widgets using a
///    [FocusScopeNode].
///  * [FocusScopeNode], a node that collects focus nodes into a group for
///    traversal.
///  * [FocusManager], a singleton that manages the primary focus and
///    distributes key events to focused nodes.
///  * [FocusTraversalPolicy], an object used to determine how to move the focus
///    to other nodes.
///  * [FocusTraversalGroup], a widget that groups together and imposes a
///    traversal policy on the [Focus] nodes below it in the widget hierarchy.
class Focus extends StatefulWidget {
  /// Creates a widget that manages a [FocusNode].
  const Focus({
    super.key,
    required this.child,
    this.focusNode,
    this.parentNode,
    this.autofocus = false,
    this.onFocusChange,
    FocusOnKeyEventCallback? onKeyEvent,
    @Deprecated(
      'Use onKeyEvent instead. '
      'This feature was deprecated after v3.18.0-2.0.pre.',
    )
    FocusOnKeyCallback? onKey,
    bool? canRequestFocus,
    bool? skipTraversal,
    bool? descendantsAreFocusable,
    bool? descendantsAreTraversable,
    this.includeSemantics = true,
    String? debugLabel,
  }) : _onKeyEvent = onKeyEvent,
       _onKey = onKey,
       _canRequestFocus = canRequestFocus,
       _skipTraversal = skipTraversal,
       _descendantsAreFocusable = descendantsAreFocusable,
       _descendantsAreTraversable = descendantsAreTraversable,
       _debugLabel = debugLabel;

  /// Creates a Focus widget that uses the given [focusNode] as the source of
  /// truth for attributes on the node, rather than the attributes of this widget.
  const factory Focus.withExternalFocusNode({
    Key? key,
    required Widget child,
    required FocusNode focusNode,
    FocusNode? parentNode,
    bool autofocus,
    ValueChanged<bool>? onFocusChange,
    bool includeSemantics,
  }) = _FocusWithExternalFocusNode;

  // Indicates whether the widget's focusNode attributes should have priority
  // when then widget is updated.
  bool get _usingExternalFocus => false;

  /// The optional parent node to use when reparenting the [focusNode] for this
  /// [Focus] widget.
  ///
  /// If [parentNode] is null, then [Focus.maybeOf] is used to find the parent
  /// in the widget tree, which is typically what is desired, since it is easier
  /// to reason about the focus tree if it mirrors the shape of the widget tree.
  ///
  /// Set this property if the focus tree needs to have a different shape than
  /// the widget tree. This is typically in cases where a dialog is in an
  /// [Overlay] (or another part of the widget tree), and focus should
  /// behave as if the widgets in the overlay are descendants of the given
  /// [parentNode] for purposes of focus.
  ///
  /// Defaults to null.
  final FocusNode? parentNode;

  /// The child widget of this [Focus].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  /// {@template flutter.widgets.Focus.focusNode}
  /// An optional focus node to use as the focus node for this widget.
  ///
  /// If one is not supplied, then one will be automatically allocated, owned,
  /// and managed by this widget. The widget will be focusable even if a
  /// [focusNode] is not supplied. If supplied, the given [focusNode] will be
  /// _hosted_ by this widget, but not owned. See [FocusNode] for more
  /// information on what being hosted and/or owned implies.
  ///
  /// Supplying a focus node is sometimes useful if an ancestor to this widget
  /// wants to control when this widget has the focus. The owner will be
  /// responsible for calling [FocusNode.dispose] on the focus node when it is
  /// done with it, but this widget will attach/detach and reparent the node
  /// when needed.
  /// {@endtemplate}
  ///
  /// A non-null [focusNode] must be supplied if using the
  /// [Focus.withExternalFocusNode] constructor.
  final FocusNode? focusNode;

  /// {@template flutter.widgets.Focus.autofocus}
  /// True if this widget will be selected as the initial focus when no other
  /// node in its scope is currently focused.
  ///
  /// Ideally, there is only one widget with autofocus set in each [FocusScope].
  /// If there is more than one widget with autofocus set, then the first one
  /// added to the tree will get focus.
  ///
  /// Defaults to false.
  /// {@endtemplate}
  final bool autofocus;

  /// Handler called when the focus changes.
  ///
  /// Called with true if this widget's node gains focus, and false if it loses
  /// focus.
  final ValueChanged<bool>? onFocusChange;

  /// A handler for keys that are pressed when this object or one of its
  /// children has focus.
  ///
  /// Key events are first given to the [FocusNode] that has primary focus, and
  /// if its [onKeyEvent] method returns [KeyEventResult.ignored], then they are
  /// given to each ancestor node up the focus hierarchy in turn. If an event
  /// reaches the root of the hierarchy, it is discarded.
  ///
  /// This is not the way to get text input in the manner of a text field: it
  /// leaves out support for input method editors, and doesn't support soft
  /// keyboards in general. For text input, consider [TextField],
  /// [EditableText], or [CupertinoTextField] instead, which do support these
  /// things.
  FocusOnKeyEventCallback? get onKeyEvent => _onKeyEvent ?? focusNode?.onKeyEvent;
  final FocusOnKeyEventCallback? _onKeyEvent;

  /// A handler for keys that are pressed when this object or one of its
  /// children has focus.
  ///
  /// This property is deprecated and will be removed. Use [onKeyEvent] instead.
  ///
  /// Key events are first given to the [FocusNode] that has primary focus, and
  /// if its [onKey] method return false, then they are given to each ancestor
  /// node up the focus hierarchy in turn. If an event reaches the root of the
  /// hierarchy, it is discarded.
  ///
  /// This is not the way to get text input in the manner of a text field: it
  /// leaves out support for input method editors, and doesn't support soft
  /// keyboards in general. For text input, consider [TextField],
  /// [EditableText], or [CupertinoTextField] instead, which do support these
  /// things.
  @Deprecated(
    'Use onKeyEvent instead. '
    'This feature was deprecated after v3.18.0-2.0.pre.',
  )
  FocusOnKeyCallback? get onKey => _onKey ?? focusNode?.onKey;
  final FocusOnKeyCallback? _onKey;

  /// {@template flutter.widgets.Focus.canRequestFocus}
  /// If true, this widget may request the primary focus.
  ///
  /// Defaults to true. Set to false if you want the [FocusNode] this widget
  /// manages to do nothing when [FocusNode.requestFocus] is called on it. Does
  /// not affect the children of this node, and [FocusNode.hasFocus] can still
  /// return true if this node is the ancestor of the primary focus.
  ///
  /// This is different than [Focus.skipTraversal] because [Focus.skipTraversal]
  /// still allows the widget to be focused, just not traversed to.
  ///
  /// Setting [FocusNode.canRequestFocus] to false implies that the widget will
  /// also be skipped for traversal purposes.
  ///
  /// See also:
  ///
  /// * [FocusTraversalGroup], a widget that sets the traversal policy for its
  ///   descendants.
  /// * [FocusTraversalPolicy], a class that can be extended to describe a
  ///   traversal policy.
  /// {@endtemplate}
  bool get canRequestFocus => _canRequestFocus ?? focusNode?.canRequestFocus ?? true;
  final bool? _canRequestFocus;

  /// Sets the [FocusNode.skipTraversal] flag on the focus node so that it won't
  /// be visited by the [FocusTraversalPolicy].
  ///
  /// This is sometimes useful if a [Focus] widget should receive key events as
  /// part of the focus chain, but shouldn't be accessible via focus traversal.
  ///
  /// This is different from [FocusNode.canRequestFocus] because it only implies
  /// that the widget can't be reached via traversal, not that it can't be
  /// focused. It may still be focused explicitly.
  bool get skipTraversal => _skipTraversal ?? focusNode?.skipTraversal ?? false;
  final bool? _skipTraversal;

  /// {@template flutter.widgets.Focus.descendantsAreFocusable}
  /// If false, will make this widget's descendants unfocusable.
  ///
  /// Defaults to true. Does not affect focusability of this node (just its
  /// descendants): for that, use [FocusNode.canRequestFocus].
  ///
  /// If any descendants are focused when this is set to false, they will be
  /// unfocused. When [descendantsAreFocusable] is set to true again, they will
  /// not be refocused, although they will be able to accept focus again.
  ///
  /// Does not affect the value of [FocusNode.canRequestFocus] on the
  /// descendants.
  ///
  /// If a descendant node loses focus when this value is changed, the focus
  /// will move to the scope enclosing this node.
  ///
  /// See also:
  ///
  /// * [ExcludeFocus], a widget that uses this property to conditionally
  ///   exclude focus for a subtree.
  /// * [descendantsAreTraversable], which makes this widget's descendants
  ///   untraversable.
  /// * [ExcludeFocusTraversal], a widget that conditionally excludes focus
  ///   traversal for a subtree.
  /// * [FocusTraversalGroup], a widget used to group together and configure the
  ///   focus traversal policy for a widget subtree that has a
  ///   `descendantsAreFocusable` parameter to conditionally block focus for a
  ///   subtree.
  /// {@endtemplate}
  bool get descendantsAreFocusable =>
      _descendantsAreFocusable ?? focusNode?.descendantsAreFocusable ?? true;
  final bool? _descendantsAreFocusable;

  /// {@template flutter.widgets.Focus.descendantsAreTraversable}
  /// If false, will make this widget's descendants untraversable.
  ///
  /// Defaults to true. Does not affect traversability of this node (just its
  /// descendants): for that, use [FocusNode.skipTraversal].
  ///
  /// Does not affect the value of [FocusNode.skipTraversal] on the
  /// descendants. Does not affect focusability of the descendants.
  ///
  /// See also:
  ///
  /// * [ExcludeFocusTraversal], a widget that uses this property to
  ///   conditionally exclude focus traversal for a subtree.
  /// * [descendantsAreFocusable], which makes this widget's descendants
  ///   unfocusable.
  /// * [ExcludeFocus], a widget that conditionally excludes focus for a subtree.
  /// * [FocusTraversalGroup], a widget used to group together and configure the
  ///   focus traversal policy for a widget subtree that has a
  ///   `descendantsAreFocusable` parameter to conditionally block focus for a
  ///   subtree.
  /// {@endtemplate}
  bool get descendantsAreTraversable =>
      _descendantsAreTraversable ?? focusNode?.descendantsAreTraversable ?? true;
  final bool? _descendantsAreTraversable;

  /// {@template flutter.widgets.Focus.includeSemantics}
  /// Include semantics information in this widget.
  ///
  /// If true, this widget will include a [Semantics] node that indicates the
  /// [SemanticsProperties.focusable] and [SemanticsProperties.focused]
  /// properties.
  ///
  /// It is not typical to set this to false, as that can affect the semantics
  /// information available to accessibility systems.
  ///
  /// Defaults to true.
  /// {@endtemplate}
  final bool includeSemantics;

  /// A debug label for this widget.
  ///
  /// Not used for anything except to be printed in the diagnostic output from
  /// [toString] or [toStringDeep].
  ///
  /// To get a string with the entire tree, call [debugDescribeFocusTree]. To
  /// print it to the console call [debugDumpFocusTree].
  ///
  /// Defaults to null.
  String? get debugLabel => _debugLabel ?? focusNode?.debugLabel;
  final String? _debugLabel;

  /// Returns the [focusNode] of the [Focus] that most tightly encloses the
  /// given [BuildContext].
  ///
  /// If no [Focus] node is found before reaching the nearest [FocusScope]
  /// widget, or there is no [Focus] widget in the context, then this method
  /// will throw an exception.
  ///
  /// {@macro flutter.widgets.focus_scope.Focus.maybeOf}
  ///
  /// See also:
  ///
  /// * [maybeOf], which is similar to this function, but will return null
  ///   instead of throwing if it doesn't find a [Focus] node.
  static FocusNode of(BuildContext context, {bool scopeOk = false, bool createDependency = true}) {
    final FocusNode? node = Focus.maybeOf(
      context,
      scopeOk: scopeOk,
      createDependency: createDependency,
    );
    assert(() {
      if (node == null) {
        throw FlutterError(
          'Focus.of() was called with a context that does not contain a Focus widget.\n'
          'No Focus widget ancestor could be found starting from the context that was passed to '
          'Focus.of(). This can happen because you are using a widget that looks for a Focus '
          'ancestor, and do not have a Focus widget descendant in the nearest FocusScope.\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    assert(() {
      if (!scopeOk && node is FocusScopeNode) {
        throw FlutterError(
          'Focus.of() was called with a context that does not contain a Focus between the given '
          'context and the nearest FocusScope widget.\n'
          'No Focus ancestor could be found starting from the context that was passed to '
          'Focus.of() to the point where it found the nearest FocusScope widget. This can happen '
          'because you are using a widget that looks for a Focus ancestor, and do not have a '
          'Focus widget ancestor in the current FocusScope.\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return node!;
  }

  /// Returns the [focusNode] of the [Focus] that most tightly encloses the
  /// given [BuildContext].
  ///
  /// If no [Focus] node is found before reaching the nearest [FocusScope]
  /// widget, or there is no [Focus] widget in scope, then this method will
  /// return null.
  ///
  /// {@template flutter.widgets.focus_scope.Focus.maybeOf}
  /// If `createDependency` is true (which is the default), calling this
  /// function creates a dependency that will rebuild the given context when the
  /// focus node gains or loses focus.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  /// * [of], which is similar to this function, but will throw an exception if
  ///   it doesn't find a [Focus] node, instead of returning null.
  static FocusNode? maybeOf(
    BuildContext context, {
    bool scopeOk = false,
    bool createDependency = true,
  }) {
    final _FocusInheritedScope? scope =
        createDependency
            ? context.dependOnInheritedWidgetOfExactType<_FocusInheritedScope>()
            : context.getInheritedWidgetOfExactType<_FocusInheritedScope>();

    return switch (scope?.notifier) {
      null => null,
      FocusScopeNode() when !scopeOk => null,
      final FocusNode node => node,
    };
  }

  /// Returns true if the nearest enclosing [Focus] widget's node is focused.
  ///
  /// A convenience method to allow build methods to write:
  /// `Focus.isAt(context)` to get whether or not the nearest [Focus] above them
  /// in the widget hierarchy currently has the input focus.
  ///
  /// Returns false if no [Focus] widget is found before reaching the nearest
  /// [FocusScope], or if the root of the focus tree is reached without finding
  /// a [Focus] widget.
  ///
  /// Calling this function creates a dependency that will rebuild the given
  /// context when the focus changes.
  static bool isAt(BuildContext context) => Focus.maybeOf(context)?.hasFocus ?? false;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('debugLabel', debugLabel, defaultValue: null));
    properties.add(
      FlagProperty('autofocus', value: autofocus, ifTrue: 'AUTOFOCUS', defaultValue: false),
    );
    properties.add(
      FlagProperty(
        'canRequestFocus',
        value: canRequestFocus,
        ifFalse: 'NOT FOCUSABLE',
        defaultValue: false,
      ),
    );
    properties.add(
      FlagProperty(
        'descendantsAreFocusable',
        value: descendantsAreFocusable,
        ifFalse: 'DESCENDANTS UNFOCUSABLE',
        defaultValue: true,
      ),
    );
    properties.add(
      FlagProperty(
        'descendantsAreTraversable',
        value: descendantsAreTraversable,
        ifFalse: 'DESCENDANTS UNTRAVERSABLE',
        defaultValue: true,
      ),
    );
    properties.add(DiagnosticsProperty<FocusNode>('focusNode', focusNode, defaultValue: null));
  }

  @override
  State<Focus> createState() => _FocusState();
}

// Implements the behavior differences when the Focus.withExternalFocusNode
// constructor is used.
class _FocusWithExternalFocusNode extends Focus {
  const _FocusWithExternalFocusNode({
    super.key,
    required super.child,
    required FocusNode super.focusNode,
    super.parentNode,
    super.autofocus,
    super.onFocusChange,
    super.includeSemantics,
  });

  @override
  bool get _usingExternalFocus => true;
  @override
  FocusOnKeyEventCallback? get onKeyEvent => focusNode!.onKeyEvent;
  @override
  FocusOnKeyCallback? get onKey => focusNode!.onKey;
  @override
  bool get canRequestFocus => focusNode!.canRequestFocus;
  @override
  bool get skipTraversal => focusNode!.skipTraversal;
  @override
  bool get descendantsAreFocusable => focusNode!.descendantsAreFocusable;
  @override
  bool? get _descendantsAreTraversable => focusNode!.descendantsAreTraversable;
  @override
  String? get debugLabel => focusNode!.debugLabel;
}

class _FocusState extends State<Focus> {
  FocusNode? _internalNode;
  FocusNode get focusNode => widget.focusNode ?? (_internalNode ??= _createNode());
  late bool _hadPrimaryFocus;
  late bool _couldRequestFocus;
  late bool _descendantsWereFocusable;
  late bool _descendantsWereTraversable;
  bool _didAutofocus = false;
  FocusAttachment? _focusAttachment;

  @override
  void initState() {
    super.initState();
    _initNode();
  }

  void _initNode() {
    if (!widget._usingExternalFocus) {
      focusNode.descendantsAreFocusable = widget.descendantsAreFocusable;
      focusNode.descendantsAreTraversable = widget.descendantsAreTraversable;
      focusNode.skipTraversal = widget.skipTraversal;
      if (widget._canRequestFocus != null) {
        focusNode.canRequestFocus = widget._canRequestFocus!;
      }
    }
    _couldRequestFocus = focusNode.canRequestFocus;
    _descendantsWereFocusable = focusNode.descendantsAreFocusable;
    _descendantsWereTraversable = focusNode.descendantsAreTraversable;
    _hadPrimaryFocus = focusNode.hasPrimaryFocus;
    _focusAttachment = focusNode.attach(
      context,
      onKeyEvent: widget.onKeyEvent,
      onKey: widget.onKey,
    );

    // Add listener even if the _internalNode existed before, since it should
    // not be listening now if we're re-using a previous one because it should
    // have already removed its listener.
    focusNode.addListener(_handleFocusChanged);
  }

  FocusNode _createNode() {
    return FocusNode(
      debugLabel: widget.debugLabel,
      canRequestFocus: widget.canRequestFocus,
      descendantsAreFocusable: widget.descendantsAreFocusable,
      descendantsAreTraversable: widget.descendantsAreTraversable,
      skipTraversal: widget.skipTraversal,
    );
  }

  @override
  void dispose() {
    // Regardless of the node owner, we need to remove it from the tree and stop
    // listening to it.
    focusNode.removeListener(_handleFocusChanged);
    _focusAttachment!.detach();

    // Don't manage the lifetime of external nodes given to the widget, just the
    // internal node.
    _internalNode?.dispose();
    super.dispose();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _focusAttachment?.reparent();
    _handleAutofocus();
  }

  void _handleAutofocus() {
    if (!_didAutofocus && widget.autofocus) {
      FocusScope.of(context).autofocus(focusNode);
      _didAutofocus = true;
    }
  }

  @override
  void deactivate() {
    super.deactivate();
    // The focus node's location in the tree is no longer valid here. But
    // we can't unfocus or remove the node from the tree because if the widget
    // is moved to a different part of the tree (via global key) it should
    // retain its focus state. That's why we temporarily park it on the root
    // focus node (via reparent) until it either gets moved to a different part
    // of the tree (via didChangeDependencies) or until it is disposed.
    _focusAttachment?.reparent();
    _didAutofocus = false;
  }

  @override
  void didUpdateWidget(Focus oldWidget) {
    super.didUpdateWidget(oldWidget);
    assert(() {
      // Only update the debug label in debug builds.
      if (oldWidget.focusNode == widget.focusNode &&
          !widget._usingExternalFocus &&
          oldWidget.debugLabel != widget.debugLabel) {
        focusNode.debugLabel = widget.debugLabel;
      }
      return true;
    }());

    if (oldWidget.focusNode == widget.focusNode) {
      if (!widget._usingExternalFocus) {
        if (widget.onKey != focusNode.onKey) {
          focusNode.onKey = widget.onKey;
        }
        if (widget.onKeyEvent != focusNode.onKeyEvent) {
          focusNode.onKeyEvent = widget.onKeyEvent;
        }
        focusNode.skipTraversal = widget.skipTraversal;
        if (widget._canRequestFocus != null) {
          focusNode.canRequestFocus = widget._canRequestFocus!;
        }
        focusNode.descendantsAreFocusable = widget.descendantsAreFocusable;
        focusNode.descendantsAreTraversable = widget.descendantsAreTraversable;
      }
    } else {
      _focusAttachment!.detach();
      oldWidget.focusNode?.removeListener(_handleFocusChanged);
      _initNode();
    }

    if (oldWidget.autofocus != widget.autofocus) {
      _handleAutofocus();
    }
  }

  void _handleFocusChanged() {
    final bool hasPrimaryFocus = focusNode.hasPrimaryFocus;
    final bool canRequestFocus = focusNode.canRequestFocus;
    final bool descendantsAreFocusable = focusNode.descendantsAreFocusable;
    final bool descendantsAreTraversable = focusNode.descendantsAreTraversable;
    widget.onFocusChange?.call(focusNode.hasFocus);
    // Check the cached states that matter here, and call setState if they have
    // changed.
    if (_hadPrimaryFocus != hasPrimaryFocus) {
      setState(() {
        _hadPrimaryFocus = hasPrimaryFocus;
      });
    }
    if (_couldRequestFocus != canRequestFocus) {
      setState(() {
        _couldRequestFocus = canRequestFocus;
      });
    }
    if (_descendantsWereFocusable != descendantsAreFocusable) {
      setState(() {
        _descendantsWereFocusable = descendantsAreFocusable;
      });
    }
    if (_descendantsWereTraversable != descendantsAreTraversable) {
      setState(() {
        _descendantsWereTraversable = descendantsAreTraversable;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    _focusAttachment!.reparent(parent: widget.parentNode);
    Widget child = widget.child;
    if (widget.includeSemantics) {
      child = Semantics(
        // Automatically request the focus for a focusable widget when it
        // receives an input focus action from the semantics. Nothing is needed
        // for losing the focus because if focus is lost, that means another
        // node will gain focus and take focus from this widget.
        // TODO(gspencergoog): Allow this to be set on iOS once the issue is
        // addressed: https://github.com/flutter/flutter/issues/150030
        onFocus:
            defaultTargetPlatform != TargetPlatform.iOS && _couldRequestFocus
                ? focusNode.requestFocus
                : null,
        focusable: _couldRequestFocus,
        focused: _hadPrimaryFocus,
        child: widget.child,
      );
    }
    return _FocusInheritedScope(node: focusNode, child: child);
  }
}

/// A [FocusScope] is similar to a [Focus], but also serves as a scope for its
/// descendants, restricting focus traversal to the scoped controls.
///
/// For example a new [FocusScope] is created automatically when a route is
/// pushed, keeping the focus traversal from moving to a control in a previous
/// route.
///
/// If you just want to group widgets together in a group so that they are
/// traversed in a particular order, but the focus can still leave the group,
/// use a [FocusTraversalGroup].
///
/// Like [Focus], [FocusScope] provides an [onFocusChange] as a way to be
/// notified when the focus is given to or removed from this widget.
///
/// The [onKey] argument allows specification of a key event handler that is
/// invoked when this node or one of its children has focus. Keys are handed to
/// the primary focused widget first, and then they propagate through the
/// ancestors of that node, stopping if one of them returns
/// [KeyEventResult.handled] from [onKey], indicating that it has handled the
/// event.
///
/// Managing a [FocusScopeNode] means managing its lifecycle, listening for
/// changes in focus, and re-parenting it when needed to keep the focus
/// hierarchy in sync with the widget hierarchy. This widget does all of those
/// things for you. See [FocusScopeNode] for more information about the details
/// of what node management entails if you are not using a [FocusScope] widget
/// and you need to do it yourself.
///
/// [FocusScopeNode]s remember the last [FocusNode] that was focused within
/// their descendants, and can move that focus to the next/previous node, or a
/// node in a particular direction when the [FocusNode.nextFocus],
/// [FocusNode.previousFocus], or [FocusNode.focusInDirection] are called on a
/// [FocusNode] or [FocusScopeNode].
///
/// To move the focus, use methods on [FocusNode] by getting the [FocusNode]
/// through the [of] method. For instance, to move the focus to the next node in
/// the focus traversal order, call `Focus.of(context).nextFocus()`. To unfocus
/// a widget, call `Focus.of(context).unfocus()`.
///
/// {@tool dartpad}
/// This example demonstrates using a [FocusScope] to restrict focus to a particular
/// portion of the app. In this case, restricting focus to the visible part of a
/// Stack.
///
/// ** See code in examples/api/lib/widgets/focus_scope/focus_scope.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [FocusScopeNode], which represents a scope node in the focus hierarchy.
///  * [FocusNode], which represents a node in the focus hierarchy and has an
///    explanation of the focus system.
///  * [Focus], a widget that manages a [FocusNode] and allows easy access to
///    managing focus without having to manage the node.
///  * [FocusManager], a singleton that manages the focus and distributes key
///    events to focused nodes.
///  * [FocusTraversalPolicy], an object used to determine how to move the focus
///    to other nodes.
///  * [FocusTraversalGroup], a widget used to configure the focus traversal
///    policy for a widget subtree.
class FocusScope extends Focus {
  /// Creates a widget that manages a [FocusScopeNode].
  const FocusScope({
    super.key,
    FocusScopeNode? node,
    super.parentNode,
    required super.child,
    super.autofocus,
    super.onFocusChange,
    super.canRequestFocus,
    super.skipTraversal,
    super.onKeyEvent,
    super.onKey,
    super.debugLabel,
    super.includeSemantics,
    super.descendantsAreFocusable,
    super.descendantsAreTraversable,
  }) : super(focusNode: node);

  /// Creates a FocusScope widget that uses the given [focusScopeNode] as the
  /// source of truth for attributes on the node, rather than the attributes of
  /// this widget.
  const factory FocusScope.withExternalFocusNode({
    Key? key,
    required Widget child,
    required FocusScopeNode focusScopeNode,
    FocusNode? parentNode,
    bool autofocus,
    bool includeSemantics,
    ValueChanged<bool>? onFocusChange,
  }) = _FocusScopeWithExternalFocusNode;

  /// Returns the [FocusNode.nearestScope] of the [Focus] or [FocusScope] that
  /// most tightly encloses the given [context].
  ///
  /// If this node doesn't have a [Focus] or [FocusScope] widget ancestor, then
  /// the [FocusManager.rootScope] is returned.
  ///
  /// {@macro flutter.widgets.focus_scope.Focus.maybeOf}
  static FocusScopeNode of(BuildContext context, {bool createDependency = true}) {
    return Focus.maybeOf(
          context,
          scopeOk: true,
          createDependency: createDependency,
        )?.nearestScope ??
        context.owner!.focusManager.rootScope;
  }

  @override
  State<Focus> createState() => _FocusScopeState();
}

// Implements the behavior differences when the FocusScope.withExternalFocusNode
// constructor is used.
class _FocusScopeWithExternalFocusNode extends FocusScope {
  const _FocusScopeWithExternalFocusNode({
    super.key,
    required super.child,
    required FocusScopeNode focusScopeNode,
    super.parentNode,
    super.autofocus,
    super.includeSemantics,
    super.onFocusChange,
  }) : super(node: focusScopeNode);

  @override
  bool get _usingExternalFocus => true;
  @override
  FocusOnKeyEventCallback? get onKeyEvent => focusNode!.onKeyEvent;
  @override
  FocusOnKeyCallback? get onKey => focusNode!.onKey;
  @override
  bool get canRequestFocus => focusNode!.canRequestFocus;
  @override
  bool get skipTraversal => focusNode!.skipTraversal;
  @override
  bool get descendantsAreFocusable => focusNode!.descendantsAreFocusable;
  @override
  bool get descendantsAreTraversable => focusNode!.descendantsAreTraversable;
  @override
  String? get debugLabel => focusNode!.debugLabel;
}

class _FocusScopeState extends _FocusState {
  @override
  FocusScopeNode _createNode() {
    return FocusScopeNode(
      debugLabel: widget.debugLabel,
      canRequestFocus: widget.canRequestFocus,
      skipTraversal: widget.skipTraversal,
    );
  }

  @override
  Widget build(BuildContext context) {
    _focusAttachment!.reparent(parent: widget.parentNode);
    Widget result = _FocusInheritedScope(node: focusNode, child: widget.child);
    if (widget.includeSemantics) {
      result = Semantics(explicitChildNodes: true, child: result);
    }
    return result;
  }
}

// The InheritedWidget for Focus and FocusScope.
class _FocusInheritedScope extends InheritedNotifier<FocusNode> {
  const _FocusInheritedScope({required FocusNode node, required super.child})
    : super(notifier: node);
}

/// A widget that controls whether or not the descendants of this widget are
/// focusable.
///
/// Does not affect the value of [Focus.canRequestFocus] on the descendants.
///
/// See also:
///
///  * [Focus], a widget for adding and managing a [FocusNode] in the widget tree.
///  * [FocusTraversalGroup], a widget that groups widgets for focus traversal,
///    and can also be used in the same way as this widget by setting its
///    `descendantsAreFocusable` attribute.
class ExcludeFocus extends StatelessWidget {
  /// Const constructor for [ExcludeFocus] widget.
  const ExcludeFocus({super.key, this.excluding = true, required this.child});

  /// If true, will make this widget's descendants unfocusable.
  ///
  /// Defaults to true.
  ///
  /// If any descendants are focused when this is set to true, they will be
  /// unfocused. When [excluding] is set to false again, they will not be
  /// refocused, although they will be able to accept focus again.
  ///
  /// Does not affect the value of [FocusNode.canRequestFocus] on the
  /// descendants.
  ///
  /// See also:
  ///
  /// * [Focus.descendantsAreFocusable], the attribute of a [Focus] widget that
  ///   controls this same property for focus widgets.
  /// * [FocusTraversalGroup], a widget used to group together and configure the
  ///   focus traversal policy for a widget subtree that has a
  ///   `descendantsAreFocusable` parameter to conditionally block focus for a
  ///   subtree.
  final bool excluding;

  /// The child widget of this [ExcludeFocus].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return Focus(
      canRequestFocus: false,
      skipTraversal: true,
      includeSemantics: false,
      descendantsAreFocusable: !excluding,
      child: child,
    );
  }
}
