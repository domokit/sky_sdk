// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/services.dart';
/// @docImport 'package:flutter/widgets.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';

export 'dart:ui' show TextDirection;

/// Determines the assertiveness level of the accessibility announcement.
///
/// It is used by [AnnounceSemanticsEvent] to determine the priority with which
/// assistive technology should treat announcements.
enum Assertiveness {
  /// The assistive technology will speak changes whenever the user is idle.
  polite,

  /// The assistive technology will interrupt any announcement that it is
  /// currently making to notify the user about the change.
  ///
  /// It should only be used for time-sensitive/critical notifications.
  assertive,
}

/// An event sent by the application to notify interested listeners that
/// something happened to the user interface (e.g. a view scrolled).
///
/// These events are usually interpreted by assistive technologies to give the
/// user additional clues about the current state of the UI.
abstract class SemanticsEvent {
  /// Initializes internal fields.
  ///
  /// [type] is a string that identifies this class of [SemanticsEvent]s.
  const SemanticsEvent(this.type);

  /// The type of this event.
  ///
  /// The type is used by the engine to translate this event into the
  /// appropriate native event (`UIAccessibility*Notification` on iOS and
  /// `AccessibilityEvent` on Android).
  final String type;

  /// Converts this event to a Map that can be encoded with
  /// [StandardMessageCodec].
  ///
  /// [nodeId] is the unique identifier of the semantics node associated with
  /// the event, or null if the event is not associated with a semantics node.
  Map<String, dynamic> toMap({int? nodeId}) {
    final Map<String, dynamic> event = <String, dynamic>{'type': type, 'data': getDataMap()};
    if (nodeId != null) {
      event['nodeId'] = nodeId;
    }

    return event;
  }

  /// Returns the event's data object.
  Map<String, dynamic> getDataMap();

  @override
  String toString() {
    final List<String> pairs = <String>[];
    final Map<String, dynamic> dataMap = getDataMap();
    final List<String> sortedKeys = dataMap.keys.toList()..sort();
    for (final String key in sortedKeys) {
      pairs.add('$key: ${dataMap[key]}');
    }
    return '${objectRuntimeType(this, 'SemanticsEvent')}(${pairs.join(', ')})';
  }
}

/// An event for a semantic announcement.
///
/// This should be used for announcement that are not seamlessly announced by
/// the system as a result of a UI state change.
///
/// For example a camera application can use this method to make accessibility
/// announcements regarding objects in the viewfinder.
///
/// When possible, prefer using mechanisms like [Semantics] to implicitly
/// trigger announcements over using this event.
class AnnounceSemanticsEvent extends SemanticsEvent {
  /// Constructs an event that triggers an announcement by the platform.
  const AnnounceSemanticsEvent(
    this.message,
    this.textDirection, {
    this.assertiveness = Assertiveness.polite,
  }) : super('announce');

  /// The message to announce.
  final String message;

  /// Text direction for [message].
  final TextDirection textDirection;

  /// Determines whether the announcement should interrupt any existing announcement,
  /// or queue after it.
  ///
  /// On the web this option uses the aria-live level to set the assertiveness
  /// of the announcement. On iOS, Android, Windows, Linux, macOS, and Fuchsia
  /// this option currently has no effect.
  final Assertiveness assertiveness;

  @override
  Map<String, dynamic> getDataMap() {
    return <String, dynamic>{
      'message': message,
      'textDirection': textDirection.index,
      if (assertiveness != Assertiveness.polite) 'assertiveness': assertiveness.index,
    };
  }
}

/// An event for a semantic announcement of a tooltip.
///
/// This is only used by Android to announce tooltip values.
class TooltipSemanticsEvent extends SemanticsEvent {
  /// Constructs an event that triggers a tooltip announcement by the platform.
  const TooltipSemanticsEvent(this.message) : super('tooltip');

  /// The text content of the tooltip.
  final String message;

  @override
  Map<String, dynamic> getDataMap() {
    return <String, dynamic>{'message': message};
  }
}

/// An event which triggers long press semantic feedback.
///
/// Currently only honored on Android. Triggers a long-press specific sound
/// when TalkBack is enabled.
class LongPressSemanticsEvent extends SemanticsEvent {
  /// Constructs an event that triggers a long-press semantic feedback by the platform.
  const LongPressSemanticsEvent() : super('longPress');

  @override
  Map<String, dynamic> getDataMap() => const <String, dynamic>{};
}

/// An event which triggers tap semantic feedback.
///
/// Currently only honored on Android. Triggers a tap specific sound when
/// TalkBack is enabled.
class TapSemanticEvent extends SemanticsEvent {
  /// Constructs an event that triggers a long-press semantic feedback by the platform.
  const TapSemanticEvent() : super('tap');

  @override
  Map<String, dynamic> getDataMap() => const <String, dynamic>{};
}

/// An event to move the accessibility focus.
///
/// Using this API is generally not recommended, as it may break a users' expectation of
/// how a11y focus works and therefore should be used very carefully.
///
/// One possible use case:
/// For example, the currently focused rendering object is replaced by another rendering
/// object. In general, such design should be avoided if possible. If not, one may want
/// to refocus the newly added rendering object.
///
/// One example that is not recommended:
/// When a new popup or dropdown opens, moving the focus in these cases may confuse users
/// and make it less accessible.
///
/// {@tool snippet}
///
/// The following code snippet shows how one can request focus on a
/// certain widget.
///
/// ```dart
/// class MyWidget extends StatefulWidget {
///   const MyWidget({super.key});
///
///   @override
///   State<MyWidget> createState() => _MyWidgetState();
/// }
///
/// class _MyWidgetState extends State<MyWidget> {
///   final GlobalKey mykey = GlobalKey();
///
///   @override
///   void initState() {
///     super.initState();
///     // Using addPostFrameCallback because changing focus need to wait for the widget to finish rendering.
///     WidgetsBinding.instance.addPostFrameCallback((_) {
///       mykey.currentContext?.findRenderObject()?.sendSemanticsEvent(const FocusSemanticEvent());
///     });
///   }
///
///   @override
///   Widget build(BuildContext context) {
///     return Scaffold(
///       appBar: AppBar(
///         title: const Text('example'),
///       ),
///       body: Column(
///         children: <Widget>[
///           const Text('Hello World'),
///           const SizedBox(height: 50),
///           Text('set focus here', key: mykey),
///         ],
///       ),
///     );
///   }
/// }
/// ```
/// {@end-tool}
///
/// This currently only supports Android and iOS.
class FocusSemanticEvent extends SemanticsEvent {
  /// Constructs an event that triggers a focus change by the platform.
  const FocusSemanticEvent() : super('focus');

  @override
  Map<String, dynamic> getDataMap() => const <String, dynamic>{};
}
