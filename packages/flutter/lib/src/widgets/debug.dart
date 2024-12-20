// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/rendering.dart';
/// @docImport 'package:flutter/scheduler.dart';
///
/// @docImport 'binding.dart';
/// @docImport 'widget_inspector.dart';
library;

import 'dart:collection';
import 'dart:developer' show Timeline; // to disambiguate reference in dartdocs below

import 'package:flutter/foundation.dart';

import 'basic.dart';
import 'framework.dart';
import 'localizations.dart';
import 'lookup_boundary.dart';
import 'media_query.dart';
import 'overlay.dart';
import 'table.dart';

// Examples can assume:
// late BuildContext context;
// List<Widget> children = <Widget>[];
// List<Widget> items = <Widget>[];

// Any changes to this file should be reflected in the debugAssertAllWidgetVarsUnset()
// function below.

/// Log the dirty widgets that are built each frame.
///
/// Combined with [debugPrintBuildScope] or [debugPrintBeginFrameBanner], this
/// allows you to distinguish builds triggered by the initial mounting of a
/// widget tree (e.g. in a call to [runApp]) from the regular builds triggered
/// by the pipeline.
///
/// Combined with [debugPrintScheduleBuildForStacks], this lets you watch a
/// widget's dirty/clean lifecycle.
///
/// To get similar information but showing it on the timeline available from
/// Flutter DevTools rather than getting it in the console (where it can be
/// overwhelming), consider [debugProfileBuildsEnabled].
///
/// See also:
///
///  * [WidgetsBinding.drawFrame], which pumps the build and rendering pipeline
///    to generate a frame.
bool debugPrintRebuildDirtyWidgets = false;

/// Signature for [debugOnRebuildDirtyWidget] implementations.
typedef RebuildDirtyWidgetCallback = void Function(Element e, bool builtOnce);

/// Callback invoked for every dirty widget built each frame.
///
/// This callback is only invoked in debug builds.
///
/// See also:
///
///  * [debugPrintRebuildDirtyWidgets], which does something similar but logs
///    to the console instead of invoking a callback.
///  * [debugOnProfilePaint], which does something similar for [RenderObject]
///    painting.
///  * [WidgetInspectorService], which uses the [debugOnRebuildDirtyWidget]
///    callback to generate aggregate profile statistics describing which widget
///    rebuilds occurred when the
///    `ext.flutter.inspector.trackRebuildDirtyWidgets` service extension is
///    enabled.
RebuildDirtyWidgetCallback? debugOnRebuildDirtyWidget;

/// Log all calls to [BuildOwner.buildScope].
///
/// Combined with [debugPrintScheduleBuildForStacks], this allows you to track
/// when a [State.setState] call gets serviced.
///
/// Combined with [debugPrintRebuildDirtyWidgets] or
/// [debugPrintBeginFrameBanner], this allows you to distinguish builds
/// triggered by the initial mounting of a widget tree (e.g. in a call to
/// [runApp]) from the regular builds triggered by the pipeline.
///
/// See also:
///
///  * [WidgetsBinding.drawFrame], which pumps the build and rendering pipeline
///    to generate a frame.
bool debugPrintBuildScope = false;

/// Log the call stacks that mark widgets as needing to be rebuilt.
///
/// This is called whenever [BuildOwner.scheduleBuildFor] adds an element to the
/// dirty list. Typically this is as a result of [Element.markNeedsBuild] being
/// called, which itself is usually a result of [State.setState] being called.
///
/// To see when a widget is rebuilt, see [debugPrintRebuildDirtyWidgets].
///
/// To see when the dirty list is flushed, see [debugPrintBuildScope].
///
/// To see when a frame is scheduled, see [debugPrintScheduleFrameStacks].
bool debugPrintScheduleBuildForStacks = false;

/// Log when widgets with global keys are deactivated and log when they are
/// reactivated (retaken).
///
/// This can help track down framework bugs relating to the [GlobalKey] logic.
bool debugPrintGlobalKeyedWidgetLifecycle = false;

/// Adds [Timeline] events for every Widget built.
///
/// The timing information this flag exposes is not representative of the actual
/// cost of building, because the overhead of adding timeline events is
/// significant relative to the time each object takes to build. However, it can
/// expose unexpected widget behavior in the timeline.
///
/// In debug builds, additional information is included in the trace (such as
/// the properties of widgets being built). Collecting this data is
/// expensive and further makes these traces non-representative of actual
/// performance. This data is omitted in profile builds.
///
/// For more information about performance debugging in Flutter, see
/// <https://docs.flutter.dev/perf/ui-performance>.
///
/// See also:
///
///  * [debugPrintRebuildDirtyWidgets], which does something similar but
///    reporting the builds to the console.
///  * [debugProfileLayoutsEnabled], which does something similar for layout,
///    and [debugPrintLayouts], its console equivalent.
///  * [debugProfilePaintsEnabled], which does something similar for painting.
///  * [debugProfileBuildsEnabledUserWidgets], which adds events for user-created
///    [Widget] build times and incurs less overhead.
///  * [debugEnhanceBuildTimelineArguments], which enhances the trace with
///    debugging information related to [Widget] builds.
bool debugProfileBuildsEnabled = false;

/// Adds [Timeline] events for every user-created [Widget] built.
///
/// A user-created [Widget] is any [Widget] that is constructed in the root
/// library. Often [Widget]s contain child [Widget]s that are constructed in
/// libraries (for example, a [TextButton] having a [RichText] child). Timeline
/// events for those children will be omitted with this flag. This works for any
/// [Widget] not just ones declared in the root library.
///
/// See also:
///
///  * [debugProfileBuildsEnabled], which functions similarly but shows events
///    for every widget and has a higher overhead cost.
///  * [debugEnhanceBuildTimelineArguments], which enhances the trace with
///    debugging information related to [Widget] builds.
bool debugProfileBuildsEnabledUserWidgets = false;

/// Adds debugging information to [Timeline] events related to [Widget] builds.
///
/// This flag will only add [Timeline] event arguments for debug builds.
/// Additional arguments will be added for the "BUILD" [Timeline] event and for
/// all [Widget] build [Timeline] events, which are the [Timeline] events that
/// are added when either of [debugProfileBuildsEnabled] and
/// [debugProfileBuildsEnabledUserWidgets] are true. The debugging information
/// that will be added in trace arguments includes stats around [Widget] dirty
/// states and [Widget] diagnostic information (i.e. [Widget] properties).
///
/// See also:
///
///  * [debugProfileBuildsEnabled], which adds [Timeline] events for every
///    [Widget] built.
///  * [debugProfileBuildsEnabledUserWidgets], which adds [Timeline] events for
///    every user-created [Widget] built.
///  * [debugEnhanceLayoutTimelineArguments], which does something similar for
///    events related to [RenderObject] layouts.
///  * [debugEnhancePaintTimelineArguments], which does something similar for
///    events related to [RenderObject] paints.
bool debugEnhanceBuildTimelineArguments = false;

/// Show banners for deprecated widgets.
bool debugHighlightDeprecatedWidgets = false;

Key? _firstNonUniqueKey(Iterable<Widget> widgets) {
  final Set<Key> keySet = HashSet<Key>();
  for (final Widget widget in widgets) {
    if (widget.key == null) {
      continue;
    }
    if (!keySet.add(widget.key!)) {
      return widget.key;
    }
  }
  return null;
}

/// Asserts if the given child list contains any duplicate non-null keys.
///
/// To invoke this function, use the following pattern:
///
/// ```dart
/// class MyWidget extends StatelessWidget {
///   MyWidget({ super.key, required this.children }) {
///     assert(!debugChildrenHaveDuplicateKeys(this, children));
///   }
///
///   final List<Widget> children;
///
///   // ...
/// }
/// ```
///
/// If specified, the `message` overrides the default message.
///
/// For a version of this function that can be used in contexts where
/// the list of items does not have a particular parent, see
/// [debugItemsHaveDuplicateKeys].
///
/// Does nothing if asserts are disabled. Always returns false.
bool debugChildrenHaveDuplicateKeys(Widget parent, Iterable<Widget> children, {String? message}) {
  assert(() {
    final Key? nonUniqueKey = _firstNonUniqueKey(children);
    if (nonUniqueKey != null) {
      throw FlutterError(
        "${message ?? 'Duplicate keys found.\n'
                'If multiple keyed widgets exist as children of another widget, they must have unique keys.'}"
        '\n$parent has multiple children with key $nonUniqueKey.',
      );
    }
    return true;
  }());
  return false;
}

/// Asserts if the given list of items contains any duplicate non-null keys.
///
/// To invoke this function, use the following pattern:
///
/// ```dart
/// assert(!debugItemsHaveDuplicateKeys(items));
/// ```
///
/// For a version of this function specifically intended for parents
/// checking their children lists, see [debugChildrenHaveDuplicateKeys].
///
/// Does nothing if asserts are disabled. Always returns false.
bool debugItemsHaveDuplicateKeys(Iterable<Widget> items) {
  assert(() {
    final Key? nonUniqueKey = _firstNonUniqueKey(items);
    if (nonUniqueKey != null) {
      throw FlutterError('Duplicate key found: $nonUniqueKey.');
    }
    return true;
  }());
  return false;
}

/// Asserts that the given context has a [Table] ancestor.
///
/// Used by [TableRowInkWell] to make sure that it is only used in an appropriate context.
///
/// To invoke this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasTable(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// This method can be expensive (it walks the element tree).
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasTable(BuildContext context) {
  assert(() {
    if (context.widget is! Table && context.findAncestorWidgetOfExactType<Table>() == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('No Table widget found.'),
        ErrorDescription('${context.widget.runtimeType} widgets require a Table widget ancestor.'),
        context.describeWidget('The specific widget that could not find a Table ancestor was'),
        context.describeOwnershipChain('The ownership chain for the affected widget is'),
      ]);
    }
    return true;
  }());
  return true;
}

/// Asserts that the given context has a [MediaQuery] ancestor.
///
/// Used by various widgets to make sure that they are only used in an
/// appropriate context.
///
/// To invoke this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasMediaQuery(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasMediaQuery(BuildContext context) {
  assert(() {
    if (context.widget is! MediaQuery &&
        context.getElementForInheritedWidgetOfExactType<MediaQuery>() == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('No MediaQuery widget ancestor found.'),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require a MediaQuery widget ancestor.',
        ),
        context.describeWidget('The specific widget that could not find a MediaQuery ancestor was'),
        context.describeOwnershipChain('The ownership chain for the affected widget is'),
        ErrorHint(
          'No MediaQuery ancestor could be found starting from the context '
          'that was passed to MediaQuery.of(). This can happen because the '
          'context used is not a descendant of a View widget, which introduces '
          'a MediaQuery.',
        ),
      ]);
    }
    return true;
  }());
  return true;
}

/// Asserts that the given context has a [Directionality] ancestor.
///
/// Used by various widgets to make sure that they are only used in an
/// appropriate context.
///
/// To invoke this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasDirectionality(context));
/// ```
///
/// To improve the error messages you can add some extra color using the
/// named arguments.
///
///  * why: explain why the direction is needed, for example "to resolve
///    the 'alignment' argument". Should be an adverb phrase describing why.
///  * hint: explain why this might be happening, for example "The default
///    value of the 'alignment' argument of the $runtimeType widget is an
///    AlignmentDirectional value.". Should be a fully punctuated sentence.
///  * alternative: provide additional advice specific to the situation,
///    especially an alternative to providing a Directionality ancestor.
///    For example, "Alternatively, consider specifying the 'textDirection'
///    argument.". Should be a fully punctuated sentence.
///
/// Each one can be null, in which case it is skipped (this is the default).
/// If they are non-null, they are included in the order above, interspersed
/// with the more generic advice regarding [Directionality].
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasDirectionality(
  BuildContext context, {
  String? why,
  String? hint,
  String? alternative,
}) {
  assert(() {
    if (context.widget is! Directionality &&
        context.getElementForInheritedWidgetOfExactType<Directionality>() == null) {
      why = why == null ? '' : ' $why';
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('No Directionality widget found.'),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require a Directionality widget ancestor$why.\n',
        ),
        if (hint != null) ErrorHint(hint),
        context.describeWidget(
          'The specific widget that could not find a Directionality ancestor was',
        ),
        context.describeOwnershipChain('The ownership chain for the affected widget is'),
        ErrorHint(
          'Typically, the Directionality widget is introduced by the MaterialApp '
          'or WidgetsApp widget at the top of your application widget tree. It '
          'determines the ambient reading direction and is used, for example, to '
          'determine how to lay out text, how to interpret "start" and "end" '
          'values, and to resolve EdgeInsetsDirectional, '
          'AlignmentDirectional, and other *Directional objects.',
        ),
        if (alternative != null) ErrorHint(alternative),
      ]);
    }
    return true;
  }());
  return true;
}

/// Asserts that the `built` widget is not null.
///
/// Used when the given `widget` calls a builder function to check that the
/// function returned a non-null value, as typically required.
///
/// Does nothing when asserts are disabled.
void debugWidgetBuilderValue(Widget widget, Widget? built) {
  assert(() {
    if (built == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('A build function returned null.'),
        DiagnosticsProperty<Widget>(
          'The offending widget is',
          widget,
          style: DiagnosticsTreeStyle.errorProperty,
        ),
        ErrorDescription('Build functions must never return null.'),
        ErrorHint(
          'To return an empty space that causes the building widget to fill available room, return "Container()". '
          'To return an empty space that takes as little room as possible, return "Container(width: 0.0, height: 0.0)".',
        ),
      ]);
    }
    if (widget == built) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('A build function returned context.widget.'),
        DiagnosticsProperty<Widget>(
          'The offending widget is',
          widget,
          style: DiagnosticsTreeStyle.errorProperty,
        ),
        ErrorDescription(
          'Build functions must never return their BuildContext parameter\'s widget or a child that contains "context.widget". '
          'Doing so introduces a loop in the widget tree that can cause the app to crash.',
        ),
      ]);
    }
    return true;
  }());
}

/// Asserts that the given context has a [Localizations] ancestor that contains
/// a [WidgetsLocalizations] delegate.
///
/// To call this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasWidgetsLocalizations(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasWidgetsLocalizations(BuildContext context) {
  assert(() {
    if (Localizations.of<WidgetsLocalizations>(context, WidgetsLocalizations) == null) {
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('No WidgetsLocalizations found.'),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require WidgetsLocalizations '
          'to be provided by a Localizations widget ancestor.',
        ),
        ErrorDescription(
          'The widgets library uses Localizations to generate messages, '
          'labels, and abbreviations.',
        ),
        ErrorHint(
          'To introduce a WidgetsLocalizations, either use a '
          'WidgetsApp at the root of your application to include them '
          'automatically, or add a Localization widget with a '
          'WidgetsLocalizations delegate.',
        ),
        ...context.describeMissingAncestor(expectedAncestorType: WidgetsLocalizations),
      ]);
    }
    return true;
  }());
  return true;
}

/// Asserts that the given context has an [Overlay] ancestor.
///
/// To call this function, use the following pattern, typically in the
/// relevant Widget's build method:
///
/// ```dart
/// assert(debugCheckHasOverlay(context));
/// ```
///
/// Always place this before any early returns, so that the invariant is checked
/// in all cases. This prevents bugs from hiding until a particular codepath is
/// hit.
///
/// This method can be expensive (it walks the element tree).
///
/// Does nothing if asserts are disabled. Always returns true.
bool debugCheckHasOverlay(BuildContext context) {
  assert(() {
    if (LookupBoundary.findAncestorWidgetOfExactType<Overlay>(context) == null) {
      final bool hiddenByBoundary = LookupBoundary.debugIsHidingAncestorWidgetOfExactType<Overlay>(
        context,
      );
      throw FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary(
          'No Overlay widget found${hiddenByBoundary ? ' within the closest LookupBoundary' : ''}.',
        ),
        if (hiddenByBoundary)
          ErrorDescription(
            'There is an ancestor Overlay widget, but it is hidden by a LookupBoundary.',
          ),
        ErrorDescription(
          '${context.widget.runtimeType} widgets require an Overlay '
          'widget ancestor within the closest LookupBoundary.\n'
          'An overlay lets widgets float on top of other widget children.',
        ),
        ErrorHint(
          'To introduce an Overlay widget, you can either directly '
          'include one, or use a widget that contains an Overlay itself, '
          'such as a Navigator, WidgetApp, MaterialApp, or CupertinoApp.',
        ),
        ...context.describeMissingAncestor(expectedAncestorType: Overlay),
      ]);
    }
    return true;
  }());
  return true;
}

/// Returns true if none of the widget library debug variables have been changed.
///
/// This function is used by the test framework to ensure that debug variables
/// haven't been inadvertently changed.
///
/// See [the widgets library](widgets/widgets-library.html) for a complete list.
bool debugAssertAllWidgetVarsUnset(String reason) {
  assert(() {
    if (debugPrintRebuildDirtyWidgets ||
        debugPrintBuildScope ||
        debugPrintScheduleBuildForStacks ||
        debugPrintGlobalKeyedWidgetLifecycle ||
        debugProfileBuildsEnabled ||
        debugHighlightDeprecatedWidgets ||
        debugProfileBuildsEnabledUserWidgets) {
      throw FlutterError(reason);
    }
    return true;
  }());
  return true;
}
