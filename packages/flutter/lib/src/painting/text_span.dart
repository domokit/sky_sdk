// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/rendering.dart';
/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:ui'
    as ui
    show Locale, LocaleStringAttribute, ParagraphBuilder, SpellOutStringAttribute, StringAttribute;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/services.dart';

import 'basic_types.dart';
import 'inline_span.dart';
import 'text_painter.dart';
import 'text_scaler.dart';

// Examples can assume:
// late TextSpan myTextSpan;

/// An immutable span of text.
///
/// A [TextSpan] object can be styled using its [style] property. The style will
/// be applied to the [text] and the [children].
///
/// A [TextSpan] object can just have plain text, or it can have children
/// [TextSpan] objects with their own styles that (possibly only partially)
/// override the [style] of this object. If a [TextSpan] has both [text] and
/// [children], then the [text] is treated as if it was an un-styled [TextSpan]
/// at the start of the [children] list. Leaving the [TextSpan.text] field null
/// results in the [TextSpan] acting as an empty node in the [InlineSpan] tree
/// with a list of children.
///
/// To paint a [TextSpan] on a [Canvas], use a [TextPainter]. To display a text
/// span in a widget, use a [RichText]. For text with a single style, consider
/// using the [Text] widget.
///
/// {@tool snippet}
///
/// The text "Hello world!", in black:
///
/// ```dart
/// const TextSpan(
///   text: 'Hello world!',
///   style: TextStyle(color: Colors.black),
/// )
/// ```
/// {@end-tool}
///
/// _There is some more detailed sample code in the documentation for the
/// [recognizer] property._
///
/// The [TextSpan.text] will be used as the semantics label unless overridden
/// by the [TextSpan.semanticsLabel] property. Any [PlaceholderSpan]s in the
/// [TextSpan.children] list will separate the text before and after it into two
/// semantics nodes.
///
/// See also:
///
///  * [WidgetSpan], a leaf node that represents an embedded inline widget in an
///    [InlineSpan] tree. Specify a widget within the [children] list by
///    wrapping the widget with a [WidgetSpan]. The widget will be laid out
///    inline within the paragraph.
///  * [Text], a widget for showing uniformly-styled text.
///  * [RichText], a widget for finer control of text rendering.
///  * [TextPainter], a class for painting [TextSpan] objects on a [Canvas].
@immutable
class TextSpan extends InlineSpan implements HitTestTarget, MouseTrackerAnnotation {
  /// Creates a [TextSpan] with the given values.
  ///
  /// For the object to be useful, at least one of [text] or
  /// [children] should be set.
  const TextSpan({
    this.text,
    this.children,
    super.style,
    this.recognizer,
    MouseCursor? mouseCursor,
    this.onEnter,
    this.onExit,
    this.semanticsLabel,
    this.semanticsIdentifier,
    this.locale,
    this.spellOut,
  }) : mouseCursor =
           mouseCursor ?? (recognizer == null ? MouseCursor.defer : SystemMouseCursors.click),
       assert(!(text == null && semanticsLabel != null));

  /// The text contained in this span.
  ///
  /// If both [text] and [children] are non-null, the text will precede the
  /// children.
  ///
  /// This getter does not include the contents of its children.
  final String? text;

  /// Additional spans to include as children.
  ///
  /// If both [text] and [children] are non-null, the text will precede the
  /// children.
  ///
  /// Modifying the list after the [TextSpan] has been created is not supported
  /// and may have unexpected results.
  ///
  /// The list must not contain any nulls.
  final List<InlineSpan>? children;

  /// A gesture recognizer that will receive events that hit this span.
  ///
  /// [InlineSpan] itself does not implement hit testing or event dispatch. The
  /// object that manages the [InlineSpan] painting is also responsible for
  /// dispatching events. In the rendering library, that is the
  /// [RenderParagraph] object, which corresponds to the [RichText] widget in
  /// the widgets layer; these objects do not bubble events in [InlineSpan]s,
  /// so a [recognizer] is only effective for events that directly hit the
  /// [text] of that [InlineSpan], not any of its [children].
  ///
  /// [InlineSpan] also does not manage the lifetime of the gesture recognizer.
  /// The code that owns the [GestureRecognizer] object must call
  /// [GestureRecognizer.dispose] when the [InlineSpan] object is no longer
  /// used.
  ///
  /// {@tool snippet}
  ///
  /// This example shows how to manage the lifetime of a gesture recognizer
  /// provided to an [InlineSpan] object. It defines a `BuzzingText` widget
  /// which uses the [HapticFeedback] class to vibrate the device when the user
  /// long-presses the "find the" span, which is underlined in wavy green. The
  /// hit-testing is handled by the [RichText] widget. It also changes the
  /// hovering mouse cursor to `precise`.
  ///
  /// ```dart
  /// class BuzzingText extends StatefulWidget {
  ///   const BuzzingText({super.key});
  ///
  ///   @override
  ///   State<BuzzingText> createState() => _BuzzingTextState();
  /// }
  ///
  /// class _BuzzingTextState extends State<BuzzingText> {
  ///   late LongPressGestureRecognizer _longPressRecognizer;
  ///
  ///   @override
  ///   void initState() {
  ///     super.initState();
  ///     _longPressRecognizer = LongPressGestureRecognizer()
  ///       ..onLongPress = _handlePress;
  ///   }
  ///
  ///   @override
  ///   void dispose() {
  ///     _longPressRecognizer.dispose();
  ///     super.dispose();
  ///   }
  ///
  ///   void _handlePress() {
  ///     HapticFeedback.vibrate();
  ///   }
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return Text.rich(
  ///       TextSpan(
  ///         text: 'Can you ',
  ///         style: const TextStyle(color: Colors.black),
  ///         children: <InlineSpan>[
  ///           TextSpan(
  ///             text: 'find the',
  ///             style: const TextStyle(
  ///               color: Colors.green,
  ///               decoration: TextDecoration.underline,
  ///               decorationStyle: TextDecorationStyle.wavy,
  ///             ),
  ///             recognizer: _longPressRecognizer,
  ///             mouseCursor: SystemMouseCursors.precise,
  ///           ),
  ///           const TextSpan(
  ///             text: ' secret?',
  ///           ),
  ///         ],
  ///       ),
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  final GestureRecognizer? recognizer;

  /// Mouse cursor when the mouse hovers over this span.
  ///
  /// The default value is [SystemMouseCursors.click] if [recognizer] is not
  /// null, or [MouseCursor.defer] otherwise.
  ///
  /// [TextSpan] itself does not implement hit testing or cursor changing.
  /// The object that manages the [TextSpan] painting is responsible
  /// to return the [TextSpan] in its hit test, as well as providing the
  /// correct mouse cursor when the [TextSpan]'s mouse cursor is
  /// [MouseCursor.defer].
  final MouseCursor mouseCursor;

  @override
  final PointerEnterEventListener? onEnter;

  @override
  final PointerExitEventListener? onExit;

  /// Returns the value of [mouseCursor].
  ///
  /// This field, required by [MouseTrackerAnnotation], is hidden publicly to
  /// avoid the confusion as a text cursor.
  @protected
  @override
  MouseCursor get cursor => mouseCursor;

  /// An alternative semantics label for this [TextSpan].
  ///
  /// If present, the semantics of this span will contain this value instead
  /// of the actual text.
  ///
  /// This is useful for replacing abbreviations or shorthands with the full
  /// text value:
  ///
  /// ```dart
  /// const TextSpan(text: r'$$', semanticsLabel: 'Double dollars')
  /// ```
  final String? semanticsLabel;

  /// A unique identifier for the semantics node for this [TextSpan].
  ///
  /// This is useful for cases where the text content of the [TextSpan] needs
  /// to be uniquely identified through the automation tools without having
  /// a dependency on the actual content of the text that can possibly be
  /// dynamic in nature.
  final String? semanticsIdentifier;

  /// The language of the text in this span and its span children.
  ///
  /// Setting the locale of this text span affects the way that assistive
  /// technologies, such as VoiceOver or TalkBack, pronounce the text.
  ///
  /// If this span contains other text span children, they also inherit the
  /// locale from this span unless explicitly set to different locales.
  final ui.Locale? locale;

  /// Whether the assistive technologies should spell out this text character
  /// by character.
  ///
  /// If the text is 'hello world', setting this to true causes the assistive
  /// technologies, such as VoiceOver or TalkBack, to pronounce
  /// 'h-e-l-l-o-space-w-o-r-l-d' instead of complete words. This is useful for
  /// texts, such as passwords or verification codes.
  ///
  /// If this span contains other text span children, they also inherit the
  /// property from this span unless explicitly set.
  ///
  /// If the property is not set, this text span inherits the spell out setting
  /// from its parent. If this text span does not have a parent or the parent
  /// does not have a spell out setting, this text span does not spell out the
  /// text by default.
  final bool? spellOut;

  @override
  bool get validForMouseTracker => true;

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    if (event is PointerDownEvent) {
      recognizer?.addPointer(event);
    }
  }

  /// Apply the [style], [text], and [children] of this object to the
  /// given [ParagraphBuilder], from which a [Paragraph] can be obtained.
  /// [Paragraph] objects can be drawn on [Canvas] objects.
  ///
  /// Rather than using this directly, it's simpler to use the
  /// [TextPainter] class to paint [TextSpan] objects onto [Canvas]
  /// objects.
  @override
  void build(
    ui.ParagraphBuilder builder, {
    TextScaler textScaler = TextScaler.noScaling,
    List<PlaceholderDimensions>? dimensions,
  }) {
    assert(debugAssertIsValid());
    final bool hasStyle = style != null;
    if (hasStyle) {
      builder.pushStyle(style!.getTextStyle(textScaler: textScaler));
    }
    if (text != null) {
      try {
        builder.addText(text!);
      } on ArgumentError catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'painting library',
            context: ErrorDescription('while building a TextSpan'),
            silent: true,
          ),
        );
        // Use a Unicode replacement character as a substitute for invalid text.
        builder.addText('\uFFFD');
      }
    }
    final List<InlineSpan>? children = this.children;
    if (children != null) {
      for (final InlineSpan child in children) {
        child.build(builder, textScaler: textScaler, dimensions: dimensions);
      }
    }
    if (hasStyle) {
      builder.pop();
    }
  }

  /// Walks this [TextSpan] and its descendants in pre-order and calls [visitor]
  /// for each span that has text.
  ///
  /// When `visitor` returns true, the walk will continue. When `visitor`
  /// returns false, then the walk will end.
  @override
  bool visitChildren(InlineSpanVisitor visitor) {
    if (text != null && !visitor(this)) {
      return false;
    }
    final List<InlineSpan>? children = this.children;
    if (children != null) {
      for (final InlineSpan child in children) {
        if (!child.visitChildren(visitor)) {
          return false;
        }
      }
    }
    return true;
  }

  @override
  bool visitDirectChildren(InlineSpanVisitor visitor) {
    final List<InlineSpan>? children = this.children;
    if (children != null) {
      for (final InlineSpan child in children) {
        if (!visitor(child)) {
          return false;
        }
      }
    }
    return true;
  }

  /// Returns the text span that contains the given position in the text.
  @override
  InlineSpan? getSpanForPositionVisitor(TextPosition position, Accumulator offset) {
    final String? text = this.text;
    if (text == null || text.isEmpty) {
      return null;
    }
    final TextAffinity affinity = position.affinity;
    final int targetOffset = position.offset;
    final int endOffset = offset.value + text.length;

    if (offset.value == targetOffset && affinity == TextAffinity.downstream ||
        offset.value < targetOffset && targetOffset < endOffset ||
        endOffset == targetOffset && affinity == TextAffinity.upstream) {
      return this;
    }
    offset.increment(text.length);
    return null;
  }

  @override
  void computeToPlainText(
    StringBuffer buffer, {
    bool includeSemanticsLabels = true,
    bool includePlaceholders = true,
  }) {
    assert(debugAssertIsValid());
    if (semanticsLabel != null && includeSemanticsLabels) {
      buffer.write(semanticsLabel);
    } else if (text != null) {
      buffer.write(text);
    }
    if (children != null) {
      for (final InlineSpan child in children!) {
        child.computeToPlainText(
          buffer,
          includeSemanticsLabels: includeSemanticsLabels,
          includePlaceholders: includePlaceholders,
        );
      }
    }
  }

  @override
  void computeSemanticsInformation(
    List<InlineSpanSemanticsInformation> collector, {
    ui.Locale? inheritedLocale,
    bool inheritedSpellOut = false,
  }) {
    assert(debugAssertIsValid());
    final ui.Locale? effectiveLocale = locale ?? inheritedLocale;
    final bool effectiveSpellOut = spellOut ?? inheritedSpellOut;

    if (text != null) {
      final int textLength = semanticsLabel?.length ?? text!.length;
      collector.add(
        InlineSpanSemanticsInformation(
          text!,
          stringAttributes: <ui.StringAttribute>[
            if (effectiveSpellOut && textLength > 0)
              ui.SpellOutStringAttribute(range: TextRange(start: 0, end: textLength)),
            if (effectiveLocale != null && textLength > 0)
              ui.LocaleStringAttribute(
                locale: effectiveLocale,
                range: TextRange(start: 0, end: textLength),
              ),
          ],
          semanticsLabel: semanticsLabel,
          semanticsIdentifier: semanticsIdentifier,
          recognizer: recognizer,
        ),
      );
    }
    final List<InlineSpan>? children = this.children;
    if (children != null) {
      for (final InlineSpan child in children) {
        if (child is TextSpan) {
          child.computeSemanticsInformation(
            collector,
            inheritedLocale: effectiveLocale,
            inheritedSpellOut: effectiveSpellOut,
          );
        } else {
          child.computeSemanticsInformation(collector);
        }
      }
    }
  }

  @override
  int? codeUnitAtVisitor(int index, Accumulator offset) {
    final String? text = this.text;
    if (text == null) {
      return null;
    }
    final int localOffset = index - offset.value;
    assert(localOffset >= 0);
    offset.increment(text.length);
    return localOffset < text.length ? text.codeUnitAt(localOffset) : null;
  }

  /// In debug mode, throws an exception if the object is not in a valid
  /// configuration. Otherwise, returns true.
  ///
  /// This is intended to be used as follows:
  ///
  /// ```dart
  /// assert(myTextSpan.debugAssertIsValid());
  /// ```
  @override
  bool debugAssertIsValid() {
    assert(() {
      if (children != null) {
        for (final InlineSpan child in children!) {
          assert(child.debugAssertIsValid());
        }
      }
      return true;
    }());
    return super.debugAssertIsValid();
  }

  @override
  RenderComparison compareTo(InlineSpan other) {
    if (identical(this, other)) {
      return RenderComparison.identical;
    }
    if (other.runtimeType != runtimeType) {
      return RenderComparison.layout;
    }
    final TextSpan textSpan = other as TextSpan;
    if (textSpan.text != text ||
        children?.length != textSpan.children?.length ||
        (style == null) != (textSpan.style == null)) {
      return RenderComparison.layout;
    }
    RenderComparison result =
        recognizer == textSpan.recognizer ? RenderComparison.identical : RenderComparison.metadata;
    if (style != null) {
      final RenderComparison candidate = style!.compareTo(textSpan.style!);
      if (candidate.index > result.index) {
        result = candidate;
      }
      if (result == RenderComparison.layout) {
        return result;
      }
    }
    if (children != null) {
      for (int index = 0; index < children!.length; index += 1) {
        final RenderComparison candidate = children![index].compareTo(textSpan.children![index]);
        if (candidate.index > result.index) {
          result = candidate;
        }
        if (result == RenderComparison.layout) {
          return result;
        }
      }
    }
    return result;
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    if (super != other) {
      return false;
    }
    return other is TextSpan &&
        other.text == text &&
        other.recognizer == recognizer &&
        other.semanticsLabel == semanticsLabel &&
        other.semanticsIdentifier == semanticsIdentifier &&
        onEnter == other.onEnter &&
        onExit == other.onExit &&
        mouseCursor == other.mouseCursor &&
        listEquals<InlineSpan>(other.children, children);
  }

  @override
  int get hashCode => Object.hash(
    super.hashCode,
    text,
    recognizer,
    semanticsLabel,
    semanticsIdentifier,
    onEnter,
    onExit,
    mouseCursor,
    children == null ? null : Object.hashAll(children!),
  );

  @override
  String toStringShort() => objectRuntimeType(this, 'TextSpan');

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);

    properties.add(StringProperty('text', text, showName: false, defaultValue: null));
    if (style == null && text == null && children == null) {
      properties.add(DiagnosticsNode.message('(empty)'));
    }

    properties.add(
      DiagnosticsProperty<GestureRecognizer>(
        'recognizer',
        recognizer,
        description: recognizer?.runtimeType.toString(),
        defaultValue: null,
      ),
    );

    properties.add(
      FlagsSummary<Function?>('callbacks', <String, Function?>{'enter': onEnter, 'exit': onExit}),
    );
    properties.add(
      DiagnosticsProperty<MouseCursor>('mouseCursor', cursor, defaultValue: MouseCursor.defer),
    );

    if (semanticsLabel != null) {
      properties.add(StringProperty('semanticsLabel', semanticsLabel));
    }

    if (semanticsIdentifier != null) {
      properties.add(StringProperty('semanticsIdentifier', semanticsIdentifier));
    }
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return children?.map<DiagnosticsNode>((InlineSpan child) {
          return child.toDiagnosticsNode();
        }).toList() ??
        const <DiagnosticsNode>[];
  }
}
