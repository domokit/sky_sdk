// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'widget_tester.dart';
library;

import 'dart:ui';

import 'package:flutter/material.dart' show Tooltip;
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';

import 'binding.dart';
import 'tree_traversal.dart';

/// Signature for [CommonFinders.byWidgetPredicate].
typedef WidgetPredicate = bool Function(Widget widget);

/// Signature for [CommonFinders.byElementPredicate].
typedef ElementPredicate = bool Function(Element element);

/// Signature for [CommonSemanticsFinders.byPredicate].
typedef SemanticsNodePredicate = bool Function(SemanticsNode node);

/// Signature for [FinderBase.describeMatch].
typedef DescribeMatchCallback = String Function(Plurality plurality);

/// The `CandidateType` of finders that search for and filter substrings,
/// within static text rendered by [RenderParagraph]s.
final class TextRangeContext {
  const TextRangeContext._(this.view, this.renderObject, this.textRange);

  /// The [View] containing the static text.
  ///
  /// This is used for hit-testing.
  final View view;

  /// The RenderObject that contains the static text.
  final RenderParagraph renderObject;

  /// The [TextRange] of the substring within [renderObject]'s text.
  final TextRange textRange;

  @override
  String toString() => 'TextRangeContext($view, $renderObject, $textRange)';
}

/// Some frequently used [Finder]s and [SemanticsFinder]s.
const CommonFinders find = CommonFinders._();

// Examples can assume:
// typedef Button = Placeholder;
// late WidgetTester tester;
// late String filePath;
// late Key backKey;

/// Provides lightweight syntax for getting frequently used [Finder]s and
/// [SemanticsFinder]s through [semantics].
///
/// This class is instantiated once, as [find].
class CommonFinders {
  const CommonFinders._();

  /// Some frequently used semantics finders.
  CommonSemanticsFinders get semantics => const CommonSemanticsFinders._();

  /// Some frequently used text range finders.
  CommonTextRangeFinders get textRange => const CommonTextRangeFinders._();

  /// Finds [Text], [EditableText], and optionally [RichText] widgets
  /// containing string equal to the `text` argument.
  ///
  /// If `findRichText` is false, all standalone [RichText] widgets are
  /// ignored and `text` is matched with [Text.data] or [Text.textSpan].
  /// If `findRichText` is true, [RichText] widgets (and therefore also
  /// [Text] and [Text.rich] widgets) are matched by comparing the
  /// [InlineSpan.toPlainText] with the given `text`.
  ///
  /// For [EditableText] widgets, the `text` is always compared to the current
  /// value of the [EditableText.controller].
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.text('Back'), findsOneWidget);
  /// ```
  ///
  /// This will match [Text], [Text.rich], and [EditableText] widgets that
  /// contain the "Back" string.
  ///
  /// ```dart
  /// expect(find.text('Close', findRichText: true), findsOneWidget);
  /// ```
  ///
  /// This will match [Text], [Text.rich], [EditableText], as well as standalone
  /// [RichText] widgets that contain the "Close" string.
  Finder text(String text, {bool findRichText = false, bool skipOffstage = true}) {
    return _TextWidgetFinder(text, findRichText: findRichText, skipOffstage: skipOffstage);
  }

  /// Finds [Text] and [EditableText], and optionally [RichText] widgets
  /// which contain the given `pattern` argument.
  ///
  /// If `findRichText` is false, all standalone [RichText] widgets are
  /// ignored and `pattern` is matched with [Text.data] or [Text.textSpan].
  /// If `findRichText` is true, [RichText] widgets (and therefore also
  /// [Text] and [Text.rich] widgets) are matched by comparing the
  /// [InlineSpan.toPlainText] with the given `pattern`.
  ///
  /// For [EditableText] widgets, the `pattern` is always compared to the current
  /// value of the [EditableText.controller].
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.textContaining('Back'), findsOneWidget);
  /// expect(find.textContaining(RegExp(r'(\w+)')), findsOneWidget);
  /// ```
  ///
  /// This will match [Text], [Text.rich], and [EditableText] widgets that
  /// contain the given pattern : 'Back' or RegExp(r'(\w+)').
  ///
  /// ```dart
  /// expect(find.textContaining('Close', findRichText: true), findsOneWidget);
  /// expect(find.textContaining(RegExp(r'(\w+)'), findRichText: true), findsOneWidget);
  /// ```
  ///
  /// This will match [Text], [Text.rich], [EditableText], as well as standalone
  /// [RichText] widgets that contain the given pattern : 'Close' or RegExp(r'(\w+)').
  Finder textContaining(Pattern pattern, {bool findRichText = false, bool skipOffstage = true}) {
    return _TextContainingWidgetFinder(
      pattern,
      findRichText: findRichText,
      skipOffstage: skipOffstage,
    );
  }

  /// Looks for widgets that contain a [Text] descendant with `text`
  /// in it.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// // Suppose there is a button with text 'Update' in it:
  /// const Button(
  ///   child: Text('Update')
  /// );
  ///
  /// // It can be found and tapped like this:
  /// tester.tap(find.widgetWithText(Button, 'Update'));
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder widgetWithText(Type widgetType, String text, {bool skipOffstage = true}) {
    return find.ancestor(
      of: find.text(text, skipOffstage: skipOffstage),
      matching: find.byType(widgetType, skipOffstage: skipOffstage),
    );
  }

  /// Finds [Image] and [FadeInImage] widgets containing `image` equal to the
  /// `image` argument.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.image(FileImage(File(filePath))), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder image(ImageProvider image, {bool skipOffstage = true}) =>
      _ImageWidgetFinder(image, skipOffstage: skipOffstage);

  /// Finds widgets by searching for one with the given `key`.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.byKey(backKey), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder byKey(Key key, {bool skipOffstage = true}) =>
      _KeyWidgetFinder(key, skipOffstage: skipOffstage);

  /// Finds widgets by searching for widgets implementing a particular type.
  ///
  /// This matcher accepts subtypes. For example a
  /// `bySubtype<StatefulWidget>()` will find any stateful widget.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.bySubtype<IconButton>(), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  ///
  /// See also:
  /// * [byType], which does not do subtype tests.
  Finder bySubtype<T extends Widget>({bool skipOffstage = true}) =>
      _SubtypeWidgetFinder<T>(skipOffstage: skipOffstage);

  /// Finds widgets by searching for widgets with a particular type.
  ///
  /// This does not do subclass tests, so for example
  /// `byType(StatefulWidget)` will never find anything since [StatefulWidget]
  /// is an abstract class.
  ///
  /// The `type` argument must be a subclass of [Widget].
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.byType(IconButton), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  ///
  /// See also:
  /// * [bySubtype], which allows subtype tests.
  Finder byType(Type type, {bool skipOffstage = true}) =>
      _TypeWidgetFinder(type, skipOffstage: skipOffstage);

  /// Finds [Icon] widgets containing icon data equal to the `icon`
  /// argument.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.byIcon(Icons.inbox), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder byIcon(IconData icon, {bool skipOffstage = true}) =>
      _IconWidgetFinder(icon, skipOffstage: skipOffstage);

  /// Looks for widgets that contain an [Icon] descendant displaying [IconData]
  /// `icon` in it.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// // Suppose there is a button with icon 'arrow_forward' in it:
  /// const Button(
  ///   child: Icon(Icons.arrow_forward)
  /// );
  ///
  /// // It can be found and tapped like this:
  /// tester.tap(find.widgetWithIcon(Button, Icons.arrow_forward));
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder widgetWithIcon(Type widgetType, IconData icon, {bool skipOffstage = true}) {
    return find.ancestor(of: find.byIcon(icon), matching: find.byType(widgetType));
  }

  /// Looks for widgets that contain an [Image] descendant displaying
  /// [ImageProvider] `image` in it.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// // Suppose there is a button with an image in it:
  /// Button(
  ///   child: Image.file(File(filePath))
  /// );
  ///
  /// // It can be found and tapped like this:
  /// tester.tap(find.widgetWithImage(Button, FileImage(File(filePath))));
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder widgetWithImage(Type widgetType, ImageProvider image, {bool skipOffstage = true}) {
    return find.ancestor(of: find.image(image), matching: find.byType(widgetType));
  }

  /// Finds widgets by searching for elements with a particular type.
  ///
  /// This does not do subclass tests, so for example
  /// `byElementType(VirtualViewportElement)` will never find anything
  /// since [RenderObjectElement] is an abstract class.
  ///
  /// The `type` argument must be a subclass of [Element].
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.byElementType(SingleChildRenderObjectElement), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder byElementType(Type type, {bool skipOffstage = true}) =>
      _ElementTypeWidgetFinder(type, skipOffstage: skipOffstage);

  /// Finds widgets whose current widget is the instance given by the `widget`
  /// argument.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// // Suppose there is a button created like this:
  /// Widget myButton = const Button(
  ///   child: Text('Update')
  /// );
  ///
  /// // It can be found and tapped like this:
  /// tester.tap(find.byWidget(myButton));
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder byWidget(Widget widget, {bool skipOffstage = true}) =>
      _ExactWidgetFinder(widget, skipOffstage: skipOffstage);

  /// Finds widgets using a widget `predicate`.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.byWidgetPredicate(
  ///   (Widget widget) => widget is Tooltip && widget.message == 'Back',
  ///   description: 'with tooltip "Back"',
  /// ), findsOneWidget);
  /// ```
  ///
  /// If `description` is provided, then this uses it as the description of the
  /// [Finder] and appears, for example, in the error message when the finder
  /// fails to locate the desired widget. Otherwise, the description prints the
  /// signature of the predicate function.
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder byWidgetPredicate(
    WidgetPredicate predicate, {
    String? description,
    bool skipOffstage = true,
  }) {
    return _WidgetPredicateWidgetFinder(
      predicate,
      description: description,
      skipOffstage: skipOffstage,
    );
  }

  /// Finds [Tooltip] widgets with the given `message`.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.byTooltip('Back'), findsOneWidget);
  /// expect(find.byTooltip(RegExp('Back.*')), findsNWidgets(2));
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder byTooltip(Pattern message, {bool skipOffstage = true}) {
    return byWidgetPredicate((Widget widget) {
      return widget is Tooltip &&
          (message is RegExp
              ? ((widget.message != null && message.hasMatch(widget.message!)) ||
                  (widget.richMessage != null &&
                      message.hasMatch(widget.richMessage!.toPlainText())))
              : ((widget.message ?? widget.richMessage?.toPlainText()) == message));
    }, skipOffstage: skipOffstage);
  }

  /// Finds widgets using an element `predicate`.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.byElementPredicate(
  ///   // Finds elements of type SingleChildRenderObjectElement, including
  ///   // those that are actually subclasses of that type.
  ///   // (contrast with byElementType, which only returns exact matches)
  ///   (Element element) => element is SingleChildRenderObjectElement,
  ///   description: '$SingleChildRenderObjectElement element',
  /// ), findsOneWidget);
  /// ```
  ///
  /// If `description` is provided, then this uses it as the description of the
  /// [Finder] and appears, for example, in the error message when the finder
  /// fails to locate the desired widget. Otherwise, the description prints the
  /// signature of the predicate function.
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder byElementPredicate(
    ElementPredicate predicate, {
    String? description,
    bool skipOffstage = true,
  }) {
    return _ElementPredicateWidgetFinder(
      predicate,
      description: description,
      skipOffstage: skipOffstage,
    );
  }

  /// Finds widgets that are descendants of the `of` parameter and that match
  /// the `matching` parameter.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.descendant(
  ///   of: find.widgetWithText(Row, 'label_1'),
  ///   matching: find.text('value_1'),
  /// ), findsOneWidget);
  /// ```
  ///
  /// If the `matchRoot` argument is true then the widget(s) specified by `of`
  /// will be matched along with the descendants.
  ///
  /// If the `skipOffstage` argument is true (the default), then nodes that are
  /// [Offstage] or that are from inactive [Route]s are skipped.
  Finder descendant({
    required FinderBase<Element> of,
    required FinderBase<Element> matching,
    bool matchRoot = false,
    bool skipOffstage = true,
  }) {
    return _DescendantWidgetFinder(of, matching, matchRoot: matchRoot, skipOffstage: skipOffstage);
  }

  /// Finds widgets that are ancestors of the `of` parameter and that match
  /// the `matching` parameter.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// // Test if a Text widget that contains 'faded' is the
  /// // descendant of an Opacity widget with opacity 0.5:
  /// expect(
  ///   tester.widget<Opacity>(
  ///     find.ancestor(
  ///       of: find.text('faded'),
  ///       matching: find.byType(Opacity),
  ///     )
  ///   ).opacity,
  ///   0.5
  /// );
  /// ```
  ///
  /// If the `matchRoot` argument is true then the widget(s) specified by `of`
  /// will be matched along with the ancestors.
  Finder ancestor({
    required FinderBase<Element> of,
    required FinderBase<Element> matching,
    bool matchRoot = false,
  }) {
    return _AncestorWidgetFinder(of, matching, matchLeaves: matchRoot);
  }

  /// Finds a standard "back" button.
  ///
  /// A common element on many user interfaces is the "back" button. This is
  /// the button which takes the user back to the previous page/screen/state.
  ///
  /// It is useful in tests to be able to find these buttons, both for tapping
  /// them or verifying their existence, but because different platforms and
  /// locales have different icons representing them with different labels and
  /// tooltips, it's not desirable to have to look them up by these attributes.
  ///
  /// This finder uses the [StandardComponentType] enum to look for buttons that
  /// have the key associated with [StandardComponentType.backButton]. If
  /// another widget is assigned that key, then it too will be considered an
  /// "official" back button in the widget tree, allowing this matcher to still
  /// find it even though it might use a different icon or tooltip.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.backButton(), findsOneWidget);
  /// ```
  ///
  /// See also:
  ///
  /// * [StandardComponentType], the enum that enumerates components that are
  ///   both common in user interfaces, but which also can vary slightly in
  ///   presentation across different platforms, locales, and devices.
  /// * [BackButton], the Flutter Material widget that represents the back
  ///   button.
  Finder backButton() {
    return byKey(StandardComponentType.backButton.key);
  }

  /// Finds a standard "close" button.
  ///
  /// A common element on many user interfaces is the "close" button. This is
  /// the button which closes or cancels whatever it is attached to.
  ///
  /// It is useful in tests to be able to find these buttons, both for tapping
  /// them or verifying their existence, but because different platforms and
  /// locales have different icons representing them with different labels and
  /// tooltips, it's not desirable to have to look them up by these attributes.
  ///
  /// This finder uses the [StandardComponentType] enum to look for buttons that
  /// have the key associated with [StandardComponentType.closeButton]. If
  /// another widget is assigned that key, then it too will be considered an
  /// "official" close button in the widget tree, allowing this matcher to still
  /// find it even though it might use a different icon or tooltip.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.closeButton(), findsOneWidget);
  /// ```
  ///
  /// See also:
  ///
  /// * [StandardComponentType], the enum that enumerates components that are
  ///   both common in user interfaces, but which also can vary slightly in
  ///   presentation across different platforms, locales, and devices.
  /// * [CloseButton], the Flutter Material widget that represents a close
  ///   button.
  Finder closeButton() {
    return byKey(StandardComponentType.closeButton.key);
  }

  /// Finds [Semantics] widgets matching the given `label`, either by
  /// [RegExp.hasMatch] or string equality.
  ///
  /// The framework may combine semantics labels in certain scenarios, such as
  /// when multiple [Text] widgets are in a [TextButton] widget. In such a
  /// case, it may be preferable to match by regular expression. Consumers of
  /// this API __must not__ introduce unsuitable content into the semantics tree
  /// for the purposes of testing; in particular, you should prefer matching by
  /// regular expression rather than by string if the framework has combined
  /// your semantics, and not try to force the framework to break up the
  /// semantics nodes. Breaking up the nodes would have an undesirable effect on
  /// screen readers and other accessibility services.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.bySemanticsLabel('Back'), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder bySemanticsLabel(Pattern label, {bool skipOffstage = true}) {
    return _bySemanticsProperty(
      label,
      (SemanticsNode? semantics) => semantics?.label,
      skipOffstage: skipOffstage,
    );
  }

  /// Finds [Semantics] widgets matching the given `identifier`, either by
  /// [RegExp.hasMatch] or string equality.
  ///
  /// This allows matching against the identifier of a [Semantics] widget, which
  /// is a unique identifier for the widget in the semantics tree. This is
  /// exposed to offer a unified way widget tests and e2e tests can match
  /// against a [Semantics] widget.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// expect(find.bySemanticsIdentifier('Back'), findsOneWidget);
  /// ```
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// nodes that are [Offstage] or that are from inactive [Route]s.
  Finder bySemanticsIdentifier(Pattern identifier, {bool skipOffstage = true}) {
    return _bySemanticsProperty(
      identifier,
      (SemanticsNode? semantics) => semantics?.identifier,
      skipOffstage: skipOffstage,
    );
  }

  Finder _bySemanticsProperty(
    Pattern pattern,
    String? Function(SemanticsNode?) propertyGetter, {
    bool skipOffstage = true,
  }) {
    if (!SemanticsBinding.instance.semanticsEnabled) {
      throw StateError(
        'Semantics are not enabled. '
        'Make sure to call tester.ensureSemantics() before using '
        'this finder, and call dispose on its return value after.',
      );
    }
    return byElementPredicate((Element element) {
      // Multiple elements can have the same renderObject - we want the "owner"
      // of the renderObject, i.e. the RenderObjectElement.
      if (element is! RenderObjectElement) {
        return false;
      }
      final String? propertyValue = propertyGetter(element.renderObject.debugSemantics);
      if (propertyValue == null) {
        return false;
      }
      return pattern is RegExp ? pattern.hasMatch(propertyValue) : pattern == propertyValue;
    }, skipOffstage: skipOffstage);
  }
}

/// Provides lightweight syntax for getting frequently used semantics finders.
///
/// This class is instantiated once, as [CommonFinders.semantics], under [find].
class CommonSemanticsFinders {
  const CommonSemanticsFinders._();

  /// Finds an ancestor of `of` that matches `matching`.
  ///
  /// If `matchRoot` is true, then the results of `of` are included in the
  /// search and results.
  FinderBase<SemanticsNode> ancestor({
    required FinderBase<SemanticsNode> of,
    required FinderBase<SemanticsNode> matching,
    bool matchRoot = false,
  }) {
    return _AncestorSemanticsFinder(of, matching, matchRoot);
  }

  /// Finds a descendant of `of` that matches `matching`.
  ///
  /// If `matchRoot` is true, then the results of `of` are included in the
  /// search and results.
  FinderBase<SemanticsNode> descendant({
    required FinderBase<SemanticsNode> of,
    required FinderBase<SemanticsNode> matching,
    bool matchRoot = false,
  }) {
    return _DescendantSemanticsFinder(of, matching, matchRoot: matchRoot);
  }

  /// Finds any [SemanticsNode]s matching the given `predicate`.
  ///
  /// If `describeMatch` is provided, it will be used to describe the
  /// [FinderBase] and [FinderResult]s.
  /// {@macro flutter_test.finders.FinderBase.describeMatch}
  ///
  /// {@template flutter_test.finders.CommonSemanticsFinders.viewParameter}
  /// The `view` provided will be used to determine the semantics tree where
  /// the search will be evaluated. If not provided, the search will be
  /// evaluated against the semantics tree of [WidgetTester.view].
  /// {@endtemplate}
  SemanticsFinder byPredicate(
    SemanticsNodePredicate predicate, {
    DescribeMatchCallback? describeMatch,
    FlutterView? view,
  }) {
    return _PredicateSemanticsFinder(predicate, describeMatch, view);
  }

  /// Finds any [SemanticsNode]s that has a [SemanticsNode.label] that matches
  /// the given `label`.
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder byLabel(Pattern label, {FlutterView? view}) {
    return byPredicate(
      (SemanticsNode node) => _matchesPattern(node.label, label),
      describeMatch:
          (Plurality plurality) =>
              '${switch (plurality) {
                Plurality.one => 'SemanticsNode',
                Plurality.zero || Plurality.many => 'SemanticsNodes',
              }} with label "$label"',
      view: view,
    );
  }

  /// Finds any [SemanticsNode]s that has a [SemanticsNode.value] that matches
  /// the given `value`.
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder byValue(Pattern value, {FlutterView? view}) {
    return byPredicate(
      (SemanticsNode node) => _matchesPattern(node.value, value),
      describeMatch:
          (Plurality plurality) =>
              '${switch (plurality) {
                Plurality.one => 'SemanticsNode',
                Plurality.zero || Plurality.many => 'SemanticsNodes',
              }} with value "$value"',
      view: view,
    );
  }

  /// Finds any [SemanticsNode]s that has a [SemanticsNode.hint] that matches
  /// the given `hint`.
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder byHint(Pattern hint, {FlutterView? view}) {
    return byPredicate(
      (SemanticsNode node) => _matchesPattern(node.hint, hint),
      describeMatch:
          (Plurality plurality) =>
              '${switch (plurality) {
                Plurality.one => 'SemanticsNode',
                Plurality.zero || Plurality.many => 'SemanticsNodes',
              }} with hint "$hint"',
      view: view,
    );
  }

  /// Finds any [SemanticsNode]s that has the given [SemanticsAction].
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder byAction(SemanticsAction action, {FlutterView? view}) {
    return byPredicate(
      (SemanticsNode node) => node.getSemanticsData().hasAction(action),
      describeMatch:
          (Plurality plurality) =>
              '${switch (plurality) {
                Plurality.one => 'SemanticsNode',
                Plurality.zero || Plurality.many => 'SemanticsNodes',
              }} with action "$action"',
      view: view,
    );
  }

  /// Finds any [SemanticsNode]s that has at least one of the given
  /// [SemanticsAction]s.
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder byAnyAction(List<SemanticsAction> actions, {FlutterView? view}) {
    final int actionsInt = actions.fold(
      0,
      (int value, SemanticsAction action) => value | action.index,
    );
    return byPredicate(
      (SemanticsNode node) => node.getSemanticsData().actions & actionsInt != 0,
      describeMatch:
          (Plurality plurality) =>
              '${switch (plurality) {
                Plurality.one => 'SemanticsNode',
                Plurality.zero || Plurality.many => 'SemanticsNodes',
              }} with any of the following actions: $actions',
      view: view,
    );
  }

  /// Finds any [SemanticsNode]s that has the given [SemanticsFlag].
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder byFlag(SemanticsFlag flag, {FlutterView? view}) {
    return byPredicate(
      (SemanticsNode node) => node.hasFlag(flag),
      describeMatch:
          (Plurality plurality) =>
              '${switch (plurality) {
                Plurality.one => 'SemanticsNode',
                Plurality.zero || Plurality.many => 'SemanticsNodes',
              }} with flag "$flag"',
      view: view,
    );
  }

  /// Finds any [SemanticsNode]s that has at least one of the given
  /// [SemanticsFlag]s.
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder byAnyFlag(List<SemanticsFlag> flags, {FlutterView? view}) {
    final int flagsInt = flags.fold(0, (int value, SemanticsFlag flag) => value | flag.index);
    return byPredicate(
      (SemanticsNode node) => node.getSemanticsData().flags & flagsInt != 0,
      describeMatch:
          (Plurality plurality) =>
              '${switch (plurality) {
                Plurality.one => 'SemanticsNode',
                Plurality.zero || Plurality.many => 'SemanticsNodes',
              }} with any of the following flags: $flags',
      view: view,
    );
  }

  /// Finds any [SemanticsNode]s that can scroll in at least one direction.
  ///
  /// If `axis` is provided, then the search will be limited to scrollable nodes
  /// that can scroll in the given axis. If `axis` is not provided, then both
  /// horizontal and vertical scrollable nodes will be found.
  ///
  /// {@macro flutter_test.finders.CommonSemanticsFinders.viewParameter}
  SemanticsFinder scrollable({Axis? axis, FlutterView? view}) {
    return byAnyAction(<SemanticsAction>[
      if (axis == null || axis == Axis.vertical) ...<SemanticsAction>[
        SemanticsAction.scrollUp,
        SemanticsAction.scrollDown,
      ],
      if (axis == null || axis == Axis.horizontal) ...<SemanticsAction>[
        SemanticsAction.scrollLeft,
        SemanticsAction.scrollRight,
      ],
    ]);
  }

  bool _matchesPattern(String target, Pattern pattern) {
    if (pattern is RegExp) {
      return pattern.hasMatch(target);
    } else {
      return pattern == target;
    }
  }
}

/// Provides lightweight syntax for getting frequently used text range finders.
///
/// This class is instantiated once, as [CommonFinders.textRange], under [find].
final class CommonTextRangeFinders {
  const CommonTextRangeFinders._();

  /// Finds all non-overlapping occurrences of the given `substring` in the
  /// static text widgets and returns the [TextRange]s.
  ///
  /// If the `skipOffstage` argument is true (the default), then this skips
  /// static text inside widgets that are [Offstage], or that are from inactive
  /// [Route]s.
  ///
  /// If the `descendentOf` argument is non-null, this method only searches in
  /// the descendants of that parameter for the given substring.
  ///
  /// This finder uses the [Pattern.allMatches] method to match the substring in
  /// the text. After finding a matching substring in the text, the method
  /// continues the search from the end of the match, thus skipping overlapping
  /// occurrences of the substring.
  FinderBase<TextRangeContext> ofSubstring(
    String substring, {
    bool skipOffstage = true,
    FinderBase<Element>? descendentOf,
  }) {
    final _TextContainingWidgetFinder textWidgetFinder = _TextContainingWidgetFinder(
      substring,
      skipOffstage: skipOffstage,
      findRichText: true,
    );
    final Finder elementFinder =
        descendentOf == null
            ? textWidgetFinder
            : _DescendantWidgetFinder(
              descendentOf,
              textWidgetFinder,
              matchRoot: true,
              skipOffstage: skipOffstage,
            );
    return _StaticTextRangeFinder(elementFinder, substring);
  }
}

/// Describes how a string of text should be pluralized.
enum Plurality {
  /// Text should be pluralized to describe zero items.
  zero,

  /// Text should be pluralized to describe a single item.
  one,

  /// Text should be pluralized to describe more than one item.
  many;

  static Plurality _fromNum(num source) {
    assert(source >= 0, 'A Plurality can only be created with a positive number.');
    return switch (source) {
      0 => Plurality.zero,
      1 => Plurality.one,
      _ => Plurality.many,
    };
  }
}

/// Encapsulates the logic for searching a list of candidates and filtering the
/// candidates to only those that meet the requirements defined by the finder.
///
/// Implementations will need to implement [allCandidates] to define the total
/// possible search space and [findInCandidates] to define the requirements of
/// the finder.
///
/// This library contains [Finder] and [SemanticsFinder] for searching
/// Flutter's element and semantics trees respectively.
///
/// If the search can be represented as a predicate, then consider using
/// [MatchFinderMixin] along with the [Finder] or [SemanticsFinder] base class.
///
/// If the search further filters the results from another finder, consider using
/// [ChainedFinderMixin] along with the [Finder] or [SemanticsFinder] base class.
abstract class FinderBase<CandidateType> {
  bool _cached = false;

  /// The results of the latest [evaluate] or [tryEvaluate] call.
  ///
  /// Unlike [evaluate] and [tryEvaluate], [found] will not re-execute the
  /// search for this finder. Either [evaluate] or [tryEvaluate] must be called
  /// before accessing [found].
  FinderResult<CandidateType> get found {
    assert(
      _found != null,
      'No results have been found yet. '
      'Either `evaluate` or `tryEvaluate` must be called before accessing `found`',
    );
    return _found!;
  }

  FinderResult<CandidateType>? _found;

  /// Whether or not this finder has any results in [found].
  bool get hasFound => _found != null;

  /// Describes zero, one, or more candidates that match the requirements of a
  /// finder.
  ///
  /// {@template flutter_test.finders.FinderBase.describeMatch}
  /// The description returned should be a brief English phrase describing a
  /// matching candidate with the proper plural form. As an example for a string
  /// finder that is looking for strings starting with "hello":
  ///
  /// ```dart
  /// String describeMatch(Plurality plurality) {
  ///   return switch (plurality) {
  ///     Plurality.zero || Plurality.many => 'strings starting with "hello"',
  ///     Plurality.one => 'string starting with "hello"',
  ///   };
  /// }
  /// ```
  /// {@endtemplate}
  ///
  /// This will be used both to describe a finder and the results of searching
  /// with that finder.
  ///
  /// See also:
  ///
  ///   * [FinderBase.toString] where this is used to fully describe the finder
  ///   * [FinderResult.toString] where this is used to provide context to the
  ///     results of a search
  String describeMatch(Plurality plurality);

  /// Returns all of the items that will be considered by this finder.
  @protected
  Iterable<CandidateType> get allCandidates;

  /// Returns a variant of this finder that only matches the first item
  /// found by this finder.
  FinderBase<CandidateType> get first => _FirstFinder<CandidateType>(this);

  /// Returns a variant of this finder that only matches the last item
  /// found by this finder.
  FinderBase<CandidateType> get last => _LastFinder<CandidateType>(this);

  /// Returns a variant of this finder that only matches the item at the
  /// given index found by this finder.
  FinderBase<CandidateType> at(int index) => _IndexFinder<CandidateType>(this, index);

  /// Returns all the items in the given list that match this
  /// finder's requirements.
  ///
  /// This is overridden to define the requirements of the finder when
  /// implementing finders that directly extend [FinderBase]. If a finder can
  /// be efficiently described just in terms of a predicate function, consider
  /// mixing in [MatchFinderMixin] and implementing [MatchFinderMixin.matches]
  /// instead.
  @protected
  Iterable<CandidateType> findInCandidates(Iterable<CandidateType> candidates);

  /// Searches a set of candidates for those that meet the requirements set by
  /// this finder and returns the result of that search.
  ///
  /// See also:
  ///
  ///   * [found] which will return the latest results without re-executing the
  ///     search.
  ///   * [tryEvaluate] which will indicate whether any results were found rather
  ///     than directly returning results.
  FinderResult<CandidateType> evaluate() {
    if (!_cached || _found == null) {
      _found = FinderResult<CandidateType>(describeMatch, findInCandidates(allCandidates));
    }
    return found;
  }

  /// Searches a set of candidates for those that meet the requirements set by
  /// this finder and returns whether the search found any matching candidates.
  ///
  /// This is useful in cases where an action needs to be repeated while or
  /// until a finder has results. The results from the search can be accessed
  /// using the [found] property without re-executing the search.
  ///
  /// ## Sample code
  ///
  /// ```dart
  /// testWidgets('Top text loads first', (WidgetTester tester) async {
  ///   // Assume a widget is pumped with a top and bottom loading area, with
  ///   // the texts "Top loaded" and "Bottom loaded" when loading is complete.
  ///   // await tester.pumpWidget(...)
  ///
  ///   // Wait until at least one loaded widget is available
  ///   Finder loadedFinder = find.textContaining('loaded');
  ///   while (!loadedFinder.tryEvaluate()) {
  ///     await tester.pump(const Duration(milliseconds: 100));
  ///   }
  ///
  ///   expect(loadedFinder.found, hasLength(1));
  ///   expect(tester.widget<Text>(loadedFinder).data, contains('Top'));
  /// });
  /// ```
  bool tryEvaluate() {
    evaluate();
    return found.isNotEmpty;
  }

  /// Runs the given callback using cached results.
  ///
  /// While in this callback, this [FinderBase] will cache the results from the
  /// next call to [evaluate] or [tryEvaluate] and then no longer evaluate new results
  /// until the callback completes. After the first call, all calls to [evaluate],
  /// [tryEvaluate] or [found] will return the same results without evaluating.
  void runCached(VoidCallback run) {
    reset();
    _cached = true;
    try {
      run();
    } finally {
      reset();
      _cached = false;
    }
  }

  /// Resets all state of this [FinderBase].
  ///
  /// Generally used between tests to reset the state of [found] if a finder is
  /// used across multiple tests.
  void reset() {
    _found = null;
  }

  /// A string representation of this finder or its results.
  ///
  /// By default, this describes the results of the search in order to play
  /// nicely with [expect] and its output when a failure occurs. If you wish
  /// to get a string representation of the finder itself, pass [describeSelf]
  /// as `true`.
  @override
  String toString({bool describeSelf = false}) {
    if (describeSelf) {
      return 'A finder that searches for ${describeMatch(Plurality.many)}.';
    } else {
      if (!hasFound) {
        evaluate();
      }
      return found.toString();
    }
  }
}

/// The results of searching with a [FinderBase].
class FinderResult<CandidateType> extends Iterable<CandidateType> {
  /// Creates a new [FinderResult] that describes the `values` using the given
  /// `describeMatch` callback.
  ///
  /// {@macro flutter_test.finders.FinderBase.describeMatch}
  FinderResult(DescribeMatchCallback describeMatch, Iterable<CandidateType> values)
    : _describeMatch = describeMatch,
      _values = values;

  final DescribeMatchCallback _describeMatch;
  final Iterable<CandidateType> _values;

  @override
  Iterator<CandidateType> get iterator => _values.iterator;

  @override
  String toString() {
    final List<CandidateType> valuesList = _values.toList();
    // This will put each value on its own line with a comma and indentation
    final String valuesString = valuesList.fold(
      '',
      (String current, CandidateType candidate) => '$current\n  $candidate,',
    );
    return 'Found ${valuesList.length} ${_describeMatch(Plurality._fromNum(valuesList.length))}: ['
        '${valuesString.isNotEmpty ? '$valuesString\n' : ''}'
        ']';
  }
}

/// Provides backwards compatibility with the original [Finder] API.
mixin _LegacyFinderMixin on FinderBase<Element> {
  Iterable<Element>? _precacheResults;

  /// Describes what the finder is looking for. The description should be
  /// a brief English noun phrase describing the finder's requirements.
  @Deprecated(
    'Use FinderBase.describeMatch instead. '
    'FinderBase.describeMatch allows for more readable descriptions and removes ambiguity about pluralization. '
    'This feature was deprecated after v3.13.0-0.2.pre.',
  )
  String get description;

  /// Returns all the elements in the given list that match this
  /// finder's pattern.
  ///
  /// When implementing Finders that inherit directly from
  /// [Finder], [findInCandidates] is the main method to override. This method
  /// is maintained for backwards compatibility and will be removed in a future
  /// version of Flutter. If the finder can efficiently be described just in
  /// terms of a predicate function, consider mixing in [MatchFinderMixin]
  /// instead.
  @Deprecated(
    'Override FinderBase.findInCandidates instead. '
    'Using the FinderBase API allows for more consistent caching behavior and cleaner options for interacting with the widget tree. '
    'This feature was deprecated after v3.13.0-0.2.pre.',
  )
  Iterable<Element> apply(Iterable<Element> candidates) {
    return findInCandidates(candidates);
  }

  /// Attempts to evaluate the finder. Returns whether any elements in the tree
  /// matched the finder. If any did, then the result is cached and can be obtained
  /// from [evaluate].
  ///
  /// If this returns true, you must call [evaluate] before you call [precache] again.
  @Deprecated(
    'Use FinderBase.tryFind or FinderBase.runCached instead. '
    'Using the FinderBase API allows for more consistent caching behavior and cleaner options for interacting with the widget tree. '
    'This feature was deprecated after v3.13.0-0.2.pre.',
  )
  bool precache() {
    assert(_precacheResults == null);
    if (tryEvaluate()) {
      return true;
    }
    _precacheResults = null;
    return false;
  }

  @override
  Iterable<Element> findInCandidates(Iterable<Element> candidates) {
    return apply(candidates);
  }
}

/// A base class for creating finders that search the [Element] tree for
/// [Widget]s.
///
/// The [findInCandidates] method must be overridden and will be enforced at
/// compilation after [apply] is removed.
abstract class Finder extends FinderBase<Element> with _LegacyFinderMixin {
  /// Creates a new [Finder] with the given `skipOffstage` value.
  Finder({this.skipOffstage = true});

  /// Whether this finder skips nodes that are offstage.
  ///
  /// If this is true, then the elements are walked using
  /// [Element.debugVisitOnstageChildren]. This skips offstage children of
  /// [Offstage] widgets, as well as children of inactive [Route]s.
  final bool skipOffstage;

  @override
  Finder get first => _FirstWidgetFinder(this);

  @override
  Finder get last => _LastWidgetFinder(this);

  @override
  Finder at(int index) => _IndexWidgetFinder(this, index);

  @override
  Iterable<Element> get allCandidates {
    return collectAllElementsFrom(WidgetsBinding.instance.rootElement!, skipOffstage: skipOffstage);
  }

  @override
  String describeMatch(Plurality plurality) {
    return switch (plurality) {
      Plurality.zero || Plurality.many => 'widgets with $description',
      Plurality.one => 'widget with $description',
    };
  }

  /// Returns a variant of this finder that only matches elements reachable by
  /// a hit test.
  ///
  /// The `at` parameter specifies the location relative to the size of the
  /// target element where the hit test is performed.
  Finder hitTestable({Alignment at = Alignment.center}) => _HitTestableWidgetFinder(this, at);
}

/// A base class for creating finders that search the semantics tree.
abstract class SemanticsFinder extends FinderBase<SemanticsNode> {
  /// Creates a new [SemanticsFinder] that will search within the given [view] or
  /// within all views if [view] is null.
  SemanticsFinder(this.view);

  /// The [FlutterView] whose semantics tree this finder will search.
  ///
  /// If null, the finder will search within all views.
  final FlutterView? view;

  /// Returns the root [SemanticsNode]s of all the semantics trees that this
  /// finder will search.
  Iterable<SemanticsNode> get roots {
    if (view == null) {
      return _allRoots;
    }
    final RenderView renderView = TestWidgetsFlutterBinding.instance.renderViews.firstWhere(
      (RenderView r) => r.flutterView == view,
    );
    return <SemanticsNode>[renderView.owner!.semanticsOwner!.rootSemanticsNode!];
  }

  @override
  Iterable<SemanticsNode> get allCandidates {
    return roots.expand((SemanticsNode root) => collectAllSemanticsNodesFrom(root));
  }

  static Iterable<SemanticsNode> get _allRoots {
    final List<SemanticsNode> roots = <SemanticsNode>[];
    void collectSemanticsRoots(PipelineOwner owner) {
      final SemanticsNode? root = owner.semanticsOwner?.rootSemanticsNode;
      if (root != null) {
        roots.add(root);
      }
      owner.visitChildren(collectSemanticsRoots);
    }

    collectSemanticsRoots(TestWidgetsFlutterBinding.instance.rootPipelineOwner);
    return roots;
  }
}

/// A base class for creating finders that search for static text rendered by a
/// [RenderParagraph].
class _StaticTextRangeFinder extends FinderBase<TextRangeContext> {
  /// Creates a new [_StaticTextRangeFinder] that searches for the given
  /// `pattern` in the [Element]s found by `_parent`.
  _StaticTextRangeFinder(this._parent, this.pattern);

  final FinderBase<Element> _parent;
  final Pattern pattern;

  Iterable<TextRangeContext> _flatMap(Element from) {
    final RenderObject? renderObject = from.renderObject;
    // This is currently only exposed on text matchers. Only consider RenderBoxes.
    if (renderObject is! RenderBox) {
      return const Iterable<TextRangeContext>.empty();
    }

    final View view = from.findAncestorWidgetOfExactType<View>()!;
    final List<RenderParagraph> paragraphs = <RenderParagraph>[];

    void visitor(RenderObject child) {
      switch (child) {
        case RenderParagraph():
          paragraphs.add(child);
        // No need to continue, we are piggybacking off of a text matcher, so
        // inline text widgets will be reported separately.
        case RenderBox():
          child.visitChildren(visitor);
        case _:
      }
    }

    visitor(renderObject);
    Iterable<TextRangeContext> searchInParagraph(RenderParagraph paragraph) {
      final String text = paragraph.text.toPlainText(includeSemanticsLabels: false);
      return pattern
          .allMatches(text)
          .map(
            (Match match) =>
                TextRangeContext._(view, paragraph, TextRange(start: match.start, end: match.end)),
          );
    }

    return paragraphs.expand(searchInParagraph);
  }

  @override
  Iterable<TextRangeContext> findInCandidates(Iterable<TextRangeContext> candidates) => candidates;

  @override
  Iterable<TextRangeContext> get allCandidates => _parent.evaluate().expand(_flatMap);

  @override
  String describeMatch(Plurality plurality) {
    return switch (plurality) {
      Plurality.zero ||
      Plurality.many => 'non-overlapping TextRanges that match the Pattern "$pattern"',
      Plurality.one => 'non-overlapping TextRange that matches the Pattern "$pattern"',
    };
  }
}

/// A mixin that applies additional filtering to the results of a parent [Finder].
mixin ChainedFinderMixin<CandidateType> on FinderBase<CandidateType> {
  /// Another finder whose results will be further filtered.
  FinderBase<CandidateType> get parent;

  /// Return another [Iterable] when given an [Iterable] of candidates from a
  /// parent [FinderBase].
  ///
  /// This is the main method to implement when mixing in [ChainedFinderMixin].
  Iterable<CandidateType> filter(Iterable<CandidateType> parentCandidates);

  @override
  Iterable<CandidateType> findInCandidates(Iterable<CandidateType> candidates) {
    return filter(parent.findInCandidates(candidates));
  }

  @override
  Iterable<CandidateType> get allCandidates => parent.allCandidates;
}

/// Applies additional filtering against a [parent] widget finder.
abstract class ChainedFinder extends Finder with ChainedFinderMixin<Element> {
  /// Create a Finder chained against the candidates of another `parent` [Finder].
  ChainedFinder(this.parent);

  @override
  final FinderBase<Element> parent;
}

mixin _FirstFinderMixin<CandidateType> on ChainedFinderMixin<CandidateType> {
  @override
  String describeMatch(Plurality plurality) {
    return '${parent.describeMatch(plurality)} (ignoring all but first)';
  }

  @override
  Iterable<CandidateType> filter(Iterable<CandidateType> parentCandidates) sync* {
    yield parentCandidates.first;
  }
}

class _FirstFinder<CandidateType> extends FinderBase<CandidateType>
    with ChainedFinderMixin<CandidateType>, _FirstFinderMixin<CandidateType> {
  _FirstFinder(this.parent);

  @override
  final FinderBase<CandidateType> parent;
}

class _FirstWidgetFinder extends ChainedFinder with _FirstFinderMixin<Element> {
  _FirstWidgetFinder(super.parent);

  @override
  String get description => describeMatch(Plurality.many);
}

mixin _LastFinderMixin<CandidateType> on ChainedFinderMixin<CandidateType> {
  @override
  String describeMatch(Plurality plurality) {
    return '${parent.describeMatch(plurality)} (ignoring all but first)';
  }

  @override
  Iterable<CandidateType> filter(Iterable<CandidateType> parentCandidates) sync* {
    yield parentCandidates.last;
  }
}

class _LastFinder<CandidateType> extends FinderBase<CandidateType>
    with ChainedFinderMixin<CandidateType>, _LastFinderMixin<CandidateType> {
  _LastFinder(this.parent);

  @override
  final FinderBase<CandidateType> parent;
}

class _LastWidgetFinder extends ChainedFinder with _LastFinderMixin<Element> {
  _LastWidgetFinder(super.parent);

  @override
  String get description => describeMatch(Plurality.many);
}

mixin _IndexFinderMixin<CandidateType> on ChainedFinderMixin<CandidateType> {
  int get index;

  @override
  String describeMatch(Plurality plurality) {
    return '${parent.describeMatch(plurality)} (ignoring all but index $index)';
  }

  @override
  Iterable<CandidateType> filter(Iterable<CandidateType> parentCandidates) sync* {
    yield parentCandidates.elementAt(index);
  }
}

class _IndexFinder<CandidateType> extends FinderBase<CandidateType>
    with ChainedFinderMixin<CandidateType>, _IndexFinderMixin<CandidateType> {
  _IndexFinder(this.parent, this.index);

  @override
  final int index;

  @override
  final FinderBase<CandidateType> parent;
}

class _IndexWidgetFinder extends ChainedFinder with _IndexFinderMixin<Element> {
  _IndexWidgetFinder(super.parent, this.index);

  @override
  final int index;

  @override
  String get description => describeMatch(Plurality.many);
}

class _HitTestableWidgetFinder extends ChainedFinder {
  _HitTestableWidgetFinder(super.parent, this.alignment);

  final Alignment alignment;

  @override
  String describeMatch(Plurality plurality) {
    return '${parent.describeMatch(plurality)} (considering only hit-testable ones)';
  }

  @override
  String get description => describeMatch(Plurality.many);

  @override
  Iterable<Element> filter(Iterable<Element> parentCandidates) sync* {
    for (final Element candidate in parentCandidates) {
      final int viewId = candidate.findAncestorWidgetOfExactType<View>()!.view.viewId;
      final RenderBox box = candidate.renderObject! as RenderBox;
      final Offset absoluteOffset = box.localToGlobal(alignment.alongSize(box.size));
      final HitTestResult hitResult = HitTestResult();
      WidgetsBinding.instance.hitTestInView(hitResult, absoluteOffset, viewId);
      for (final HitTestEntry entry in hitResult.path) {
        if (entry.target == candidate.renderObject) {
          yield candidate;
          break;
        }
      }
    }
  }
}

/// A mixin for creating finders that search candidates for those that match
/// a given pattern.
mixin MatchFinderMixin<CandidateType> on FinderBase<CandidateType> {
  /// Returns true if the given element matches the pattern.
  ///
  /// When implementing a MatchFinder, this is the main method to override.
  bool matches(CandidateType candidate);

  @override
  Iterable<CandidateType> findInCandidates(Iterable<CandidateType> candidates) {
    return candidates.where(matches);
  }
}

/// Searches candidates for any that match a particular pattern.
abstract class MatchFinder extends Finder with MatchFinderMixin<Element> {
  /// Initializes a predicate-based Finder. Used by subclasses to initialize the
  /// `skipOffstage` property.
  MatchFinder({super.skipOffstage});
}

abstract class _MatchTextFinder extends MatchFinder {
  _MatchTextFinder({this.findRichText = false, super.skipOffstage});

  /// Whether standalone [RichText] widgets should be found or not.
  ///
  /// Defaults to `false`.
  ///
  /// If disabled, only [Text] widgets will be matched. [RichText] widgets
  /// *without* a [Text] ancestor will be ignored.
  /// If enabled, only [RichText] widgets will be matched. This *implicitly*
  /// matches [Text] widgets as well since they always insert a [RichText]
  /// child.
  ///
  /// In either case, [EditableText] widgets will also be matched.
  final bool findRichText;

  bool matchesText(String textToMatch);

  @override
  bool matches(Element candidate) {
    final Widget widget = candidate.widget;
    if (widget is EditableText) {
      return _matchesEditableText(widget);
    }

    if (!findRichText) {
      return _matchesNonRichText(widget);
    }
    // It would be sufficient to always use _matchesRichText if we wanted to
    // match both standalone RichText widgets as well as Text widgets. However,
    // the find.text() finder used to always ignore standalone RichText widgets,
    // which is why we need the _matchesNonRichText method in order to not be
    // backwards-compatible and not break existing tests.
    return _matchesRichText(widget);
  }

  bool _matchesRichText(Widget widget) {
    if (widget is RichText) {
      return matchesText(widget.text.toPlainText());
    }
    return false;
  }

  bool _matchesNonRichText(Widget widget) {
    if (widget is Text) {
      if (widget.data != null) {
        return matchesText(widget.data!);
      }
      assert(widget.textSpan != null);
      return matchesText(widget.textSpan!.toPlainText());
    }
    return false;
  }

  bool _matchesEditableText(EditableText widget) {
    return matchesText(widget.controller.text);
  }
}

class _TextWidgetFinder extends _MatchTextFinder {
  _TextWidgetFinder(this.text, {super.findRichText, super.skipOffstage});

  final String text;

  @override
  String get description => 'text "$text"';

  @override
  bool matchesText(String textToMatch) {
    return textToMatch == text;
  }
}

class _TextContainingWidgetFinder extends _MatchTextFinder {
  _TextContainingWidgetFinder(this.pattern, {super.findRichText, super.skipOffstage});

  final Pattern pattern;

  @override
  String get description => 'text containing $pattern';

  @override
  bool matchesText(String textToMatch) {
    return textToMatch.contains(pattern);
  }
}

class _KeyWidgetFinder extends MatchFinder {
  _KeyWidgetFinder(this.key, {super.skipOffstage});

  final Key key;

  @override
  String get description => 'key $key';

  @override
  bool matches(Element candidate) {
    return candidate.widget.key == key;
  }
}

class _SubtypeWidgetFinder<T extends Widget> extends MatchFinder {
  _SubtypeWidgetFinder({super.skipOffstage});

  @override
  String get description => 'is "$T"';

  @override
  bool matches(Element candidate) {
    return candidate.widget is T;
  }
}

class _TypeWidgetFinder extends MatchFinder {
  _TypeWidgetFinder(this.widgetType, {super.skipOffstage});

  final Type widgetType;

  @override
  String get description => 'type "$widgetType"';

  @override
  bool matches(Element candidate) {
    return candidate.widget.runtimeType == widgetType;
  }
}

class _ImageWidgetFinder extends MatchFinder {
  _ImageWidgetFinder(this.image, {super.skipOffstage});

  final ImageProvider image;

  @override
  String get description => 'image "$image"';

  @override
  bool matches(Element candidate) {
    final Widget widget = candidate.widget;
    if (widget is Image) {
      return widget.image == image;
    } else if (widget is FadeInImage) {
      return widget.image == image;
    }
    return false;
  }
}

class _IconWidgetFinder extends MatchFinder {
  _IconWidgetFinder(this.icon, {super.skipOffstage});

  final IconData icon;

  @override
  String get description => 'icon "$icon"';

  @override
  bool matches(Element candidate) {
    final Widget widget = candidate.widget;
    return widget is Icon && widget.icon == icon;
  }
}

class _ElementTypeWidgetFinder extends MatchFinder {
  _ElementTypeWidgetFinder(this.elementType, {super.skipOffstage});

  final Type elementType;

  @override
  String get description => 'type "$elementType"';

  @override
  bool matches(Element candidate) {
    return candidate.runtimeType == elementType;
  }
}

class _ExactWidgetFinder extends MatchFinder {
  _ExactWidgetFinder(this.widget, {super.skipOffstage});

  final Widget widget;

  @override
  String get description => 'the given widget ($widget)';

  @override
  bool matches(Element candidate) {
    return candidate.widget == widget;
  }
}

class _WidgetPredicateWidgetFinder extends MatchFinder {
  _WidgetPredicateWidgetFinder(this.predicate, {String? description, super.skipOffstage})
    : _description = description;

  final WidgetPredicate predicate;
  final String? _description;

  @override
  String get description => _description ?? 'widget matching predicate';

  @override
  bool matches(Element candidate) {
    return predicate(candidate.widget);
  }
}

class _ElementPredicateWidgetFinder extends MatchFinder {
  _ElementPredicateWidgetFinder(this.predicate, {String? description, super.skipOffstage})
    : _description = description;

  final ElementPredicate predicate;
  final String? _description;

  @override
  String get description => _description ?? 'element matching predicate';

  @override
  bool matches(Element candidate) {
    return predicate(candidate);
  }
}

class _PredicateSemanticsFinder extends SemanticsFinder with MatchFinderMixin<SemanticsNode> {
  _PredicateSemanticsFinder(this.predicate, DescribeMatchCallback? describeMatch, super.view)
    : _describeMatch = describeMatch;

  final SemanticsNodePredicate predicate;
  final DescribeMatchCallback? _describeMatch;

  @override
  String describeMatch(Plurality plurality) {
    return _describeMatch?.call(plurality) ?? 'matching semantics predicate';
  }

  @override
  bool matches(SemanticsNode candidate) {
    return predicate(candidate);
  }
}

mixin _DescendantFinderMixin<CandidateType> on FinderBase<CandidateType> {
  FinderBase<CandidateType> get ancestor;
  FinderBase<CandidateType> get descendant;
  bool get matchRoot;

  @override
  String describeMatch(Plurality plurality) {
    return '${descendant.describeMatch(plurality)} descending from '
        '${ancestor.describeMatch(plurality)}'
        '${matchRoot ? ' inclusive' : ''}';
  }

  @override
  Iterable<CandidateType> findInCandidates(Iterable<CandidateType> candidates) {
    final Iterable<CandidateType> descendants = descendant.evaluate();
    return candidates.where((CandidateType candidate) => descendants.contains(candidate));
  }

  @override
  Iterable<CandidateType> get allCandidates {
    final Iterable<CandidateType> ancestors = ancestor.evaluate();
    final List<CandidateType> candidates =
        ancestors
            .expand<CandidateType>((CandidateType ancestor) => _collectDescendants(ancestor))
            .toSet()
            .toList();
    if (matchRoot) {
      candidates.insertAll(0, ancestors);
    }
    return candidates;
  }

  Iterable<CandidateType> _collectDescendants(CandidateType root);
}

class _DescendantWidgetFinder extends Finder with _DescendantFinderMixin<Element> {
  _DescendantWidgetFinder(
    this.ancestor,
    this.descendant, {
    this.matchRoot = false,
    super.skipOffstage,
  });

  @override
  final FinderBase<Element> ancestor;
  @override
  final FinderBase<Element> descendant;
  @override
  final bool matchRoot;

  @override
  String get description => describeMatch(Plurality.many);

  @override
  Iterable<Element> _collectDescendants(Element root) {
    return collectAllElementsFrom(root, skipOffstage: skipOffstage);
  }
}

class _DescendantSemanticsFinder extends FinderBase<SemanticsNode>
    with _DescendantFinderMixin<SemanticsNode> {
  _DescendantSemanticsFinder(this.ancestor, this.descendant, {this.matchRoot = false});

  @override
  final FinderBase<SemanticsNode> ancestor;

  @override
  final FinderBase<SemanticsNode> descendant;

  @override
  final bool matchRoot;

  @override
  Iterable<SemanticsNode> _collectDescendants(SemanticsNode root) {
    return collectAllSemanticsNodesFrom(root);
  }
}

mixin _AncestorFinderMixin<CandidateType> on FinderBase<CandidateType> {
  FinderBase<CandidateType> get ancestor;
  FinderBase<CandidateType> get descendant;
  bool get matchLeaves;

  @override
  String describeMatch(Plurality plurality) {
    return '${ancestor.describeMatch(plurality)} that are ancestors of '
        '${descendant.describeMatch(plurality)}'
        '${matchLeaves ? ' inclusive' : ''}';
  }

  @override
  Iterable<CandidateType> findInCandidates(Iterable<CandidateType> candidates) {
    final Iterable<CandidateType> ancestors = ancestor.evaluate();
    return candidates.where((CandidateType element) => ancestors.contains(element));
  }

  @override
  Iterable<CandidateType> get allCandidates {
    final List<CandidateType> candidates = <CandidateType>[];
    for (final CandidateType leaf in descendant.evaluate()) {
      if (matchLeaves) {
        candidates.add(leaf);
      }
      candidates.addAll(_collectAncestors(leaf));
    }
    return candidates;
  }

  Iterable<CandidateType> _collectAncestors(CandidateType child);
}

class _AncestorWidgetFinder extends Finder with _AncestorFinderMixin<Element> {
  _AncestorWidgetFinder(this.descendant, this.ancestor, {this.matchLeaves = false})
    : super(skipOffstage: false);

  @override
  final FinderBase<Element> ancestor;
  @override
  final FinderBase<Element> descendant;
  @override
  final bool matchLeaves;

  @override
  String get description => describeMatch(Plurality.many);

  @override
  Iterable<Element> _collectAncestors(Element child) {
    final List<Element> ancestors = <Element>[];
    child.visitAncestorElements((Element element) {
      ancestors.add(element);
      return true;
    });
    return ancestors;
  }
}

class _AncestorSemanticsFinder extends FinderBase<SemanticsNode>
    with _AncestorFinderMixin<SemanticsNode> {
  _AncestorSemanticsFinder(this.descendant, this.ancestor, this.matchLeaves);

  @override
  final FinderBase<SemanticsNode> ancestor;

  @override
  final FinderBase<SemanticsNode> descendant;

  @override
  final bool matchLeaves;

  @override
  Iterable<SemanticsNode> _collectAncestors(SemanticsNode child) {
    final List<SemanticsNode> ancestors = <SemanticsNode>[];
    while (child.parent != null) {
      ancestors.add(child.parent!);
      child = child.parent!;
    }
    return ancestors;
  }
}
