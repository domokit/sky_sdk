// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:developer';
///
/// @docImport 'package:flutter/rendering.dart';
/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:collection';
import 'dart:math' as math;
import 'dart:ui' show clampDouble;

import 'package:meta/meta.dart';

import 'assertions.dart';
import 'constants.dart';
import 'debug.dart';
import 'object.dart';

// Examples can assume:
// late int rows, columns;
// late String _name;
// late bool inherit;
// abstract class ExampleSuperclass with Diagnosticable { }
// late String message;
// late double stepWidth;
// late double scale;
// late double hitTestExtent;
// late double paintExtent;
// late double maxWidth;
// late double progress;
// late int maxLines;
// late Duration duration;
// late int depth;
// late bool primary;
// late bool isCurrent;
// late bool keepAlive;
// late bool obscureText;
// late TextAlign textAlign;
// late ImageRepeat repeat;
// late Widget widget;
// late List<BoxShadow> boxShadow;
// late Size size;
// late bool hasSize;
// late Matrix4 transform;
// late Color color;
// late Map<Listenable, VoidCallback>? handles;
// late DiagnosticsTreeStyle style;
// late IconData icon;
// late double devicePixelRatio;

/// The various priority levels used to filter which diagnostics are shown and
/// omitted.
///
/// Trees of Flutter diagnostics can be very large so filtering the diagnostics
/// shown matters. Typically filtering to only show diagnostics with at least
/// level [debug] is appropriate.
///
/// In release mode, this level may not have any effect, as diagnostics in
/// release mode are compacted or truncated to reduce binary size.
enum DiagnosticLevel {
  /// Diagnostics that should not be shown.
  ///
  /// If a user chooses to display [hidden] diagnostics, they should not expect
  /// the diagnostics to be formatted consistently with other diagnostics and
  /// they should expect them to sometimes be misleading. For example,
  /// [FlagProperty] and [ObjectFlagProperty] have uglier formatting when the
  /// property `value` does not match a value with a custom flag
  /// description. An example of a misleading diagnostic is a diagnostic for
  /// a property that has no effect because some other property of the object is
  /// set in a way that causes the hidden property to have no effect.
  hidden,

  /// A diagnostic that is likely to be low value but where the diagnostic
  /// display is just as high quality as a diagnostic with a higher level.
  ///
  /// Use this level for diagnostic properties that match their default value
  /// and other cases where showing a diagnostic would not add much value such
  /// as an [IterableProperty] where the value is empty.
  fine,

  /// Diagnostics that should only be shown when performing fine grained
  /// debugging of an object.
  ///
  /// Unlike a [fine] diagnostic, these diagnostics provide important
  /// information about the object that is likely to be needed to debug. Used by
  /// properties that are important but where the property value is too verbose
  /// (e.g. 300+ characters long) to show with a higher diagnostic level.
  debug,

  /// Interesting diagnostics that should be typically shown.
  info,

  /// Very important diagnostics that indicate problematic property values.
  ///
  /// For example, use if you would write the property description
  /// message in ALL CAPS.
  warning,

  /// Diagnostics that provide a hint about best practices.
  ///
  /// For example, a diagnostic describing best practices for fixing an error.
  hint,

  /// Diagnostics that summarize other diagnostics present.
  ///
  /// For example, use this level for a short one or two line summary
  /// describing other diagnostics present.
  summary,

  /// Diagnostics that indicate errors or unexpected conditions.
  ///
  /// For example, use for property values where computing the value throws an
  /// exception.
  error,

  /// Special level indicating that no diagnostics should be shown.
  ///
  /// Do not specify this level for diagnostics. This level is only used to
  /// filter which diagnostics are shown.
  off,
}

/// Styles for displaying a node in a [DiagnosticsNode] tree.
///
/// In release mode, these styles may be ignored, as diagnostics are compacted
/// or truncated to save on binary size.
///
/// See also:
///
///  * [DiagnosticsNode.toStringDeep], which dumps text art trees for these
///    styles.
enum DiagnosticsTreeStyle {
  /// A style that does not display the tree, for release mode.
  none,

  /// Sparse style for displaying trees.
  ///
  /// See also:
  ///
  ///  * [RenderObject], which uses this style.
  sparse,

  /// Connects a node to its parent with a dashed line.
  ///
  /// See also:
  ///
  ///  * [RenderSliverMultiBoxAdaptor], which uses this style to distinguish
  ///    offstage children from onstage children.
  offstage,

  /// Slightly more compact version of the [sparse] style.
  ///
  /// See also:
  ///
  ///  * [Element], which uses this style.
  dense,

  /// Style that enables transitioning from nodes of one style to children of
  /// another.
  ///
  /// See also:
  ///
  ///  * [RenderParagraph], which uses this style to display a [TextSpan] child
  ///    in a way that is compatible with the [DiagnosticsTreeStyle.sparse]
  ///    style of the [RenderObject] tree.
  transition,

  /// Style for displaying content describing an error.
  ///
  /// See also:
  ///
  ///  * [FlutterError], which uses this style for the root node in a tree
  ///    describing an error.
  error,

  /// Render the tree just using whitespace without connecting parents to
  /// children using lines.
  ///
  /// See also:
  ///
  ///  * [SliverGeometry], which uses this style.
  whitespace,

  /// Render the tree without indenting children at all.
  ///
  /// See also:
  ///
  ///  * [DiagnosticsStackTrace], which uses this style.
  flat,

  /// Render the tree on a single line without showing children.
  singleLine,

  /// Render the tree using a style appropriate for properties that are part
  /// of an error message.
  ///
  /// The name is placed on one line with the value and properties placed on
  /// the following line.
  ///
  /// See also:
  ///
  ///  * [singleLine], which displays the same information but keeps the
  ///    property and value on the same line.
  errorProperty,

  /// Render only the immediate properties of a node instead of the full tree.
  ///
  /// See also:
  ///
  ///  * [DebugOverflowIndicatorMixin], which uses this style to display just
  ///    the immediate children of a node.
  shallow,

  /// Render only the children of a node truncating before the tree becomes too
  /// large.
  truncateChildren,
}

/// Configuration specifying how a particular [DiagnosticsTreeStyle] should be
/// rendered as text art.
///
/// In release mode, these configurations may be ignored, as diagnostics are
/// compacted or truncated to save on binary size.
///
/// See also:
///
///  * [sparseTextConfiguration], which is a typical style.
///  * [transitionTextConfiguration], which is an example of a complex tree style.
///  * [DiagnosticsNode.toStringDeep], for code using [TextTreeConfiguration]
///    to render text art for arbitrary trees of [DiagnosticsNode] objects.
class TextTreeConfiguration {
  /// Create a configuration object describing how to render a tree as text.
  TextTreeConfiguration({
    required this.prefixLineOne,
    required this.prefixOtherLines,
    required this.prefixLastChildLineOne,
    required this.prefixOtherLinesRootNode,
    required this.linkCharacter,
    required this.propertyPrefixIfChildren,
    required this.propertyPrefixNoChildren,
    this.lineBreak = '\n',
    this.lineBreakProperties = true,
    this.afterName = ':',
    this.afterDescriptionIfBody = '',
    this.afterDescription = '',
    this.beforeProperties = '',
    this.afterProperties = '',
    this.mandatoryAfterProperties = '',
    this.propertySeparator = '',
    this.bodyIndent = '',
    this.footer = '',
    this.showChildren = true,
    this.addBlankLineIfNoChildren = true,
    this.isNameOnOwnLine = false,
    this.isBlankLineBetweenPropertiesAndChildren = true,
    this.beforeName = '',
    this.suffixLineOne = '',
    this.mandatoryFooter = '',
  }) : childLinkSpace = ' ' * linkCharacter.length;

  /// Prefix to add to the first line to display a child with this style.
  final String prefixLineOne;

  /// Suffix to add to end of the first line to make its length match the footer.
  final String suffixLineOne;

  /// Prefix to add to other lines to display a child with this style.
  ///
  /// [prefixOtherLines] should typically be one character shorter than
  /// [prefixLineOne] is.
  final String prefixOtherLines;

  /// Prefix to add to the first line to display the last child of a node with
  /// this style.
  final String prefixLastChildLineOne;

  /// Additional prefix to add to other lines of a node if this is the root node
  /// of the tree.
  final String prefixOtherLinesRootNode;

  /// Prefix to add before each property if the node as children.
  ///
  /// Plays a similar role to [linkCharacter] except that some configurations
  /// intentionally use a different line style than the [linkCharacter].
  final String propertyPrefixIfChildren;

  /// Prefix to add before each property if the node does not have children.
  ///
  /// This string is typically a whitespace string the same length as
  /// [propertyPrefixIfChildren] but can have a different length.
  final String propertyPrefixNoChildren;

  /// Character to use to draw line linking parent to child.
  ///
  /// The first child does not require a line but all subsequent children do
  /// with the line drawn immediately before the left edge of the previous
  /// sibling.
  final String linkCharacter;

  /// Whitespace to draw instead of the childLink character if this node is the
  /// last child of its parent so no link line is required.
  final String childLinkSpace;

  /// Character(s) to use to separate lines.
  ///
  /// Typically leave set at the default value of '\n' unless this style needs
  /// to treat lines differently as is the case for
  /// [singleLineTextConfiguration].
  final String lineBreak;

  /// Whether to place line breaks between properties or to leave all
  /// properties on one line.
  final bool lineBreakProperties;

  /// Text added immediately before the name of the node.
  ///
  /// See [errorTextConfiguration] for an example of using this to achieve a
  /// custom line art style.
  final String beforeName;

  /// Text added immediately after the name of the node.
  ///
  /// See [transitionTextConfiguration] for an example of using a value other
  /// than ':' to achieve a custom line art style.
  final String afterName;

  /// Text to add immediately after the description line of a node with
  /// properties and/or children if the node has a body.
  final String afterDescriptionIfBody;

  /// Text to add immediately after the description line of a node with
  /// properties and/or children.
  final String afterDescription;

  /// Optional string to add before the properties of a node.
  ///
  /// Only displayed if the node has properties.
  /// See [singleLineTextConfiguration] for an example of using this field
  /// to enclose the property list with parenthesis.
  final String beforeProperties;

  /// Optional string to add after the properties of a node.
  ///
  /// See documentation for [beforeProperties].
  final String afterProperties;

  /// Mandatory string to add after the properties of a node regardless of
  /// whether the node has any properties.
  final String mandatoryAfterProperties;

  /// Property separator to add between properties.
  ///
  /// See [singleLineTextConfiguration] for an example of using this field
  /// to render properties as a comma separated list.
  final String propertySeparator;

  /// Prefix to add to all lines of the body of the tree node.
  ///
  /// The body is all content in the node other than the name and description.
  final String bodyIndent;

  /// Whether the children of a node should be shown.
  ///
  /// See [singleLineTextConfiguration] for an example of using this field to
  /// hide all children of a node.
  final bool showChildren;

  /// Whether to add a blank line at the end of the output for a node if it has
  /// no children.
  ///
  /// See [denseTextConfiguration] for an example of setting this to false.
  final bool addBlankLineIfNoChildren;

  /// Whether the name should be displayed on the same line as the description.
  final bool isNameOnOwnLine;

  /// Footer to add as its own line at the end of a non-root node.
  ///
  /// See [transitionTextConfiguration] for an example of using footer to draw a box
  /// around the node. [footer] is indented the same amount as [prefixOtherLines].
  final String footer;

  /// Footer to add even for root nodes.
  final String mandatoryFooter;

  /// Add a blank line between properties and children if both are present.
  final bool isBlankLineBetweenPropertiesAndChildren;
}

/// Default text tree configuration.
///
/// Example:
///
///     <root_name>: <root_description>
///      │ <property1>
///      │ <property2>
///      │ ...
///      │ <propertyN>
///      ├─<child_name>: <child_description>
///      │ │ <property1>
///      │ │ <property2>
///      │ │ ...
///      │ │ <propertyN>
///      │ │
///      │ └─<child_name>: <child_description>
///      │     <property1>
///      │     <property2>
///      │     ...
///      │     <propertyN>
///      │
///      └─<child_name>: <child_description>'
///        <property1>
///        <property2>
///        ...
///        <propertyN>
///
/// See also:
///
///  * [DiagnosticsTreeStyle.sparse], uses this style for ASCII art display.
final TextTreeConfiguration sparseTextConfiguration = TextTreeConfiguration(
  prefixLineOne: '├─',
  prefixOtherLines: ' ',
  prefixLastChildLineOne: '└─',
  linkCharacter: '│',
  propertyPrefixIfChildren: '│ ',
  propertyPrefixNoChildren: '  ',
  prefixOtherLinesRootNode: ' ',
);

/// Identical to [sparseTextConfiguration] except that the lines connecting
/// parent to children are dashed.
///
/// Example:
///
///     <root_name>: <root_description>
///      │ <property1>
///      │ <property2>
///      │ ...
///      │ <propertyN>
///      ├─<normal_child_name>: <child_description>
///      ╎ │ <property1>
///      ╎ │ <property2>
///      ╎ │ ...
///      ╎ │ <propertyN>
///      ╎ │
///      ╎ └─<child_name>: <child_description>
///      ╎     <property1>
///      ╎     <property2>
///      ╎     ...
///      ╎     <propertyN>
///      ╎
///      ╎╌<dashed_child_name>: <child_description>
///      ╎ │ <property1>
///      ╎ │ <property2>
///      ╎ │ ...
///      ╎ │ <propertyN>
///      ╎ │
///      ╎ └─<child_name>: <child_description>
///      ╎     <property1>
///      ╎     <property2>
///      ╎     ...
///      ╎     <propertyN>
///      ╎
///      └╌<dashed_child_name>: <child_description>'
///        <property1>
///        <property2>
///        ...
///        <propertyN>
///
/// See also:
///
///  * [DiagnosticsTreeStyle.offstage], uses this style for ASCII art display.
final TextTreeConfiguration dashedTextConfiguration = TextTreeConfiguration(
  prefixLineOne: '╎╌',
  prefixLastChildLineOne: '└╌',
  prefixOtherLines: ' ',
  linkCharacter: '╎',
  // Intentionally not set as a dashed line as that would make the properties
  // look like they were disabled.
  propertyPrefixIfChildren: '│ ',
  propertyPrefixNoChildren: '  ',
  prefixOtherLinesRootNode: ' ',
);

/// Dense text tree configuration that minimizes horizontal whitespace.
///
/// Example:
///
///     <root_name>: <root_description>(<property1>; <property2> <propertyN>)
///     ├<child_name>: <child_description>(<property1>, <property2>, <propertyN>)
///     └<child_name>: <child_description>(<property1>, <property2>, <propertyN>)
///
/// See also:
///
///  * [DiagnosticsTreeStyle.dense], uses this style for ASCII art display.
final TextTreeConfiguration denseTextConfiguration = TextTreeConfiguration(
  propertySeparator: ', ',
  beforeProperties: '(',
  afterProperties: ')',
  lineBreakProperties: false,
  prefixLineOne: '├',
  prefixOtherLines: '',
  prefixLastChildLineOne: '└',
  linkCharacter: '│',
  propertyPrefixIfChildren: '│',
  propertyPrefixNoChildren: ' ',
  prefixOtherLinesRootNode: '',
  addBlankLineIfNoChildren: false,
  isBlankLineBetweenPropertiesAndChildren: false,
);

/// Configuration that draws a box around a leaf node.
///
/// Used by leaf nodes such as [TextSpan] to draw a clear border around the
/// contents of a node.
///
/// Example:
///
///     <parent_node>
///     ╞═╦══ <name> ═══
///     │ ║  <description>:
///     │ ║    <body>
///     │ ║    ...
///     │ ╚═══════════
///     ╘═╦══ <name> ═══
///       ║  <description>:
///       ║    <body>
///       ║    ...
///       ╚═══════════
///
/// See also:
///
///  * [DiagnosticsTreeStyle.transition], uses this style for ASCII art display.
final TextTreeConfiguration transitionTextConfiguration = TextTreeConfiguration(
  prefixLineOne: '╞═╦══ ',
  prefixLastChildLineOne: '╘═╦══ ',
  prefixOtherLines: ' ║ ',
  footer: ' ╚═══════════',
  linkCharacter: '│',
  // Subtree boundaries are clear due to the border around the node so omit the
  // property prefix.
  propertyPrefixIfChildren: '',
  propertyPrefixNoChildren: '',
  prefixOtherLinesRootNode: '',
  afterName: ' ═══',
  // Add a colon after the description if the node has a body to make the
  // connection between the description and the body clearer.
  afterDescriptionIfBody: ':',
  // Members are indented an extra two spaces to disambiguate as the description
  // is placed within the box instead of along side the name as is the case for
  // other styles.
  bodyIndent: '  ',
  isNameOnOwnLine: true,
  // No need to add a blank line as the footer makes the boundary of this
  // subtree unambiguous.
  addBlankLineIfNoChildren: false,
  isBlankLineBetweenPropertiesAndChildren: false,
);

/// Configuration that draws a box around a node ignoring the connection to the
/// parents.
///
/// If nested in a tree, this node is best displayed in the property box rather
/// than as a traditional child.
///
/// Used to draw a decorative box around detailed descriptions of an exception.
///
/// Example:
///
///     ══╡ <name>: <description> ╞═════════════════════════════════════
///     <body>
///     ...
///     ├─<normal_child_name>: <child_description>
///     ╎ │ <property1>
///     ╎ │ <property2>
///     ╎ │ ...
///     ╎ │ <propertyN>
///     ╎ │
///     ╎ └─<child_name>: <child_description>
///     ╎     <property1>
///     ╎     <property2>
///     ╎     ...
///     ╎     <propertyN>
///     ╎
///     ╎╌<dashed_child_name>: <child_description>
///     ╎ │ <property1>
///     ╎ │ <property2>
///     ╎ │ ...
///     ╎ │ <propertyN>
///     ╎ │
///     ╎ └─<child_name>: <child_description>
///     ╎     <property1>
///     ╎     <property2>
///     ╎     ...
///     ╎     <propertyN>
///     ╎
///     └╌<dashed_child_name>: <child_description>'
///     ════════════════════════════════════════════════════════════════
///
/// See also:
///
///  * [DiagnosticsTreeStyle.error], uses this style for ASCII art display.
final TextTreeConfiguration errorTextConfiguration = TextTreeConfiguration(
  prefixLineOne: '╞═╦',
  prefixLastChildLineOne: '╘═╦',
  prefixOtherLines: ' ║ ',
  footer: ' ╚═══════════',
  linkCharacter: '│',
  // Subtree boundaries are clear due to the border around the node so omit the
  // property prefix.
  propertyPrefixIfChildren: '',
  propertyPrefixNoChildren: '',
  prefixOtherLinesRootNode: '',
  beforeName: '══╡ ',
  suffixLineOne: ' ╞══',
  mandatoryFooter: '═════',
  // No need to add a blank line as the footer makes the boundary of this
  // subtree unambiguous.
  addBlankLineIfNoChildren: false,
  isBlankLineBetweenPropertiesAndChildren: false,
);

/// Whitespace only configuration where children are consistently indented
/// two spaces.
///
/// Use this style for displaying properties with structured values or for
/// displaying children within a [transitionTextConfiguration] as using a style that
/// draws line art would be visually distracting for those cases.
///
/// Example:
///
///     <parent_node>
///       <name>: <description>:
///         <properties>
///         <children>
///       <name>: <description>:
///         <properties>
///         <children>
///
/// See also:
///
///  * [DiagnosticsTreeStyle.whitespace], uses this style for ASCII art display.
final TextTreeConfiguration whitespaceTextConfiguration = TextTreeConfiguration(
  prefixLineOne: '',
  prefixLastChildLineOne: '',
  prefixOtherLines: ' ',
  prefixOtherLinesRootNode: '  ',
  propertyPrefixIfChildren: '',
  propertyPrefixNoChildren: '',
  linkCharacter: ' ',
  addBlankLineIfNoChildren: false,
  // Add a colon after the description and before the properties to link the
  // properties to the description line.
  afterDescriptionIfBody: ':',
  isBlankLineBetweenPropertiesAndChildren: false,
);

/// Whitespace only configuration where children are not indented.
///
/// Use this style when indentation is not needed to disambiguate parents from
/// children as in the case of a [DiagnosticsStackTrace].
///
/// Example:
///
///     <parent_node>
///     <name>: <description>:
///     <properties>
///     <children>
///     <name>: <description>:
///     <properties>
///     <children>
///
/// See also:
///
///  * [DiagnosticsTreeStyle.flat], uses this style for ASCII art display.
final TextTreeConfiguration flatTextConfiguration = TextTreeConfiguration(
  prefixLineOne: '',
  prefixLastChildLineOne: '',
  prefixOtherLines: '',
  prefixOtherLinesRootNode: '',
  propertyPrefixIfChildren: '',
  propertyPrefixNoChildren: '',
  linkCharacter: '',
  addBlankLineIfNoChildren: false,
  // Add a colon after the description and before the properties to link the
  // properties to the description line.
  afterDescriptionIfBody: ':',
  isBlankLineBetweenPropertiesAndChildren: false,
);

/// Render a node as a single line omitting children.
///
/// Example:
/// `<name>: <description>(<property1>, <property2>, ..., <propertyN>)`
///
/// See also:
///
///  * [DiagnosticsTreeStyle.singleLine], uses this style for ASCII art display.
final TextTreeConfiguration singleLineTextConfiguration = TextTreeConfiguration(
  propertySeparator: ', ',
  beforeProperties: '(',
  afterProperties: ')',
  prefixLineOne: '',
  prefixOtherLines: '',
  prefixLastChildLineOne: '',
  lineBreak: '',
  lineBreakProperties: false,
  addBlankLineIfNoChildren: false,
  showChildren: false,
  propertyPrefixIfChildren: '  ',
  propertyPrefixNoChildren: '  ',
  linkCharacter: '',
  prefixOtherLinesRootNode: '',
);

/// Render the name on a line followed by the body and properties on the next
/// line omitting the children.
///
/// Example:
///
///     <name>:
///       <description>(<property1>, <property2>, ..., <propertyN>)
///
/// See also:
///
///  * [DiagnosticsTreeStyle.errorProperty], uses this style for ASCII art
///    display.
final TextTreeConfiguration errorPropertyTextConfiguration = TextTreeConfiguration(
  propertySeparator: ', ',
  beforeProperties: '(',
  afterProperties: ')',
  prefixLineOne: '',
  prefixOtherLines: '',
  prefixLastChildLineOne: '',
  lineBreakProperties: false,
  addBlankLineIfNoChildren: false,
  showChildren: false,
  propertyPrefixIfChildren: '  ',
  propertyPrefixNoChildren: '  ',
  linkCharacter: '',
  prefixOtherLinesRootNode: '',
  isNameOnOwnLine: true,
);

/// Render a node on multiple lines omitting children.
///
/// Example:
/// `<name>: <description>
///   <property1>
///   <property2>
///   <propertyN>`
///
/// See also:
///
///  * [DiagnosticsTreeStyle.shallow]
final TextTreeConfiguration shallowTextConfiguration = TextTreeConfiguration(
  prefixLineOne: '',
  prefixLastChildLineOne: '',
  prefixOtherLines: ' ',
  prefixOtherLinesRootNode: '  ',
  propertyPrefixIfChildren: '',
  propertyPrefixNoChildren: '',
  linkCharacter: ' ',
  addBlankLineIfNoChildren: false,
  // Add a colon after the description and before the properties to link the
  // properties to the description line.
  afterDescriptionIfBody: ':',
  isBlankLineBetweenPropertiesAndChildren: false,
  showChildren: false,
);

enum _WordWrapParseMode { inSpace, inWord, atBreak }

/// Builder that builds a String with specified prefixes for the first and
/// subsequent lines.
///
/// Allows for the incremental building of strings using `write*()` methods.
/// The strings are concatenated into a single string with the first line
/// prefixed by [prefixLineOne] and subsequent lines prefixed by
/// [prefixOtherLines].
class _PrefixedStringBuilder {
  _PrefixedStringBuilder({
    required this.prefixLineOne,
    required String? prefixOtherLines,
    this.wrapWidth,
  }) : _prefixOtherLines = prefixOtherLines;

  /// Prefix to add to the first line.
  final String prefixLineOne;

  /// Prefix to add to subsequent lines.
  ///
  /// The prefix can be modified while the string is being built in which case
  /// subsequent lines will be added with the modified prefix.
  String? get prefixOtherLines => _nextPrefixOtherLines ?? _prefixOtherLines;
  String? _prefixOtherLines;
  set prefixOtherLines(String? prefix) {
    _prefixOtherLines = prefix;
    _nextPrefixOtherLines = null;
  }

  String? _nextPrefixOtherLines;
  void incrementPrefixOtherLines(String suffix, {required bool updateCurrentLine}) {
    if (_currentLine.isEmpty || updateCurrentLine) {
      _prefixOtherLines = prefixOtherLines! + suffix;
      _nextPrefixOtherLines = null;
    } else {
      _nextPrefixOtherLines = prefixOtherLines! + suffix;
    }
  }

  final int? wrapWidth;

  /// Buffer containing lines that have already been completely laid out.
  final StringBuffer _buffer = StringBuffer();

  /// Buffer containing the current line that has not yet been wrapped.
  final StringBuffer _currentLine = StringBuffer();

  /// List of pairs of integers indicating the start and end of each block of
  /// text within _currentLine that can be wrapped.
  final List<int> _wrappableRanges = <int>[];

  /// Whether the string being built already has more than 1 line.
  bool get requiresMultipleLines =>
      _numLines > 1 ||
      (_numLines == 1 && _currentLine.isNotEmpty) ||
      (_currentLine.length + _getCurrentPrefix(true)!.length > wrapWidth!);

  bool get isCurrentLineEmpty => _currentLine.isEmpty;

  int _numLines = 0;

  void _finalizeLine(bool addTrailingLineBreak) {
    final bool firstLine = _buffer.isEmpty;
    final String text = _currentLine.toString();
    _currentLine.clear();

    if (_wrappableRanges.isEmpty) {
      // Fast path. There were no wrappable spans of text.
      _writeLine(text, includeLineBreak: addTrailingLineBreak, firstLine: firstLine);
      return;
    }
    final Iterable<String> lines = _wordWrapLine(
      text,
      _wrappableRanges,
      wrapWidth!,
      startOffset: firstLine ? prefixLineOne.length : _prefixOtherLines!.length,
      otherLineOffset: _prefixOtherLines!.length,
    );
    int i = 0;
    final int length = lines.length;
    for (final String line in lines) {
      i++;
      _writeLine(line, includeLineBreak: addTrailingLineBreak || i < length, firstLine: firstLine);
    }
    _wrappableRanges.clear();
  }

  /// Wraps the given string at the given width.
  ///
  /// Wrapping occurs at space characters (U+0020).
  ///
  /// This is not suitable for use with arbitrary Unicode text. For example, it
  /// doesn't implement UAX #14, can't handle ideographic text, doesn't hyphenate,
  /// and so forth. It is only intended for formatting error messages.
  ///
  /// This method wraps a sequence of text where only some spans of text can be
  /// used as wrap boundaries.
  static Iterable<String> _wordWrapLine(
    String message,
    List<int> wrapRanges,
    int width, {
    int startOffset = 0,
    int otherLineOffset = 0,
  }) {
    if (message.length + startOffset < width) {
      // Nothing to do. The line doesn't wrap.
      return <String>[message];
    }
    final List<String> wrappedLine = <String>[];
    int startForLengthCalculations = -startOffset;
    bool addPrefix = false;
    int index = 0;
    _WordWrapParseMode mode = _WordWrapParseMode.inSpace;
    late int lastWordStart;
    int? lastWordEnd;
    int start = 0;

    int currentChunk = 0;

    // This helper is called with increasing indexes.
    bool noWrap(int index) {
      while (true) {
        if (currentChunk >= wrapRanges.length) {
          return true;
        }

        if (index < wrapRanges[currentChunk + 1]) {
          break; // Found nearest chunk.
        }
        currentChunk += 2;
      }
      return index < wrapRanges[currentChunk];
    }

    while (true) {
      switch (mode) {
        case _WordWrapParseMode
            .inSpace: // at start of break point (or start of line); can't break until next break
          while ((index < message.length) && (message[index] == ' ')) {
            index += 1;
          }
          lastWordStart = index;
          mode = _WordWrapParseMode.inWord;
        case _WordWrapParseMode.inWord: // looking for a good break point. Treat all text
          while ((index < message.length) && (message[index] != ' ' || noWrap(index))) {
            index += 1;
          }
          mode = _WordWrapParseMode.atBreak;
        case _WordWrapParseMode.atBreak: // at start of break point
          if ((index - startForLengthCalculations > width) || (index == message.length)) {
            // we are over the width line, so break
            if ((index - startForLengthCalculations <= width) || (lastWordEnd == null)) {
              // we should use this point, because either it doesn't actually go over the
              // end (last line), or it does, but there was no earlier break point
              lastWordEnd = index;
            }
            final String line = message.substring(start, lastWordEnd);
            wrappedLine.add(line);
            addPrefix = true;
            if (lastWordEnd >= message.length) {
              return wrappedLine;
            }
            // just yielded a line
            if (lastWordEnd == index) {
              // we broke at current position
              // eat all the wrappable spaces, then set our start point
              // Even if some of the spaces are not wrappable that is ok.
              while ((index < message.length) && (message[index] == ' ')) {
                index += 1;
              }
              start = index;
              mode = _WordWrapParseMode.inWord;
            } else {
              // we broke at the previous break point, and we're at the start of a new one
              assert(lastWordStart > lastWordEnd);
              start = lastWordStart;
              mode = _WordWrapParseMode.atBreak;
            }
            startForLengthCalculations = start - otherLineOffset;
            assert(addPrefix);
            lastWordEnd = null;
          } else {
            // save this break point, we're not yet over the line width
            lastWordEnd = index;
            // skip to the end of this break point
            mode = _WordWrapParseMode.inSpace;
          }
      }
    }
  }

  /// Write text ensuring the specified prefixes for the first and subsequent
  /// lines.
  ///
  /// If [allowWrap] is true, the text may be wrapped to stay within the
  /// allow `wrapWidth`.
  void write(String s, {bool allowWrap = false}) {
    if (s.isEmpty) {
      return;
    }

    final List<String> lines = s.split('\n');
    for (int i = 0; i < lines.length; i += 1) {
      if (i > 0) {
        _finalizeLine(true);
        _updatePrefix();
      }
      final String line = lines[i];
      if (line.isNotEmpty) {
        if (allowWrap && wrapWidth != null) {
          final int wrapStart = _currentLine.length;
          final int wrapEnd = wrapStart + line.length;
          if (_wrappableRanges.lastOrNull == wrapStart) {
            // Extend last range.
            _wrappableRanges.last = wrapEnd;
          } else {
            _wrappableRanges
              ..add(wrapStart)
              ..add(wrapEnd);
          }
        }
        _currentLine.write(line);
      }
    }
  }

  void _updatePrefix() {
    if (_nextPrefixOtherLines != null) {
      _prefixOtherLines = _nextPrefixOtherLines;
      _nextPrefixOtherLines = null;
    }
  }

  void _writeLine(String line, {required bool includeLineBreak, required bool firstLine}) {
    line = '${_getCurrentPrefix(firstLine)}$line';
    _buffer.write(line.trimRight());
    if (includeLineBreak) {
      _buffer.write('\n');
    }
    _numLines++;
  }

  String? _getCurrentPrefix(bool firstLine) {
    return _buffer.isEmpty ? prefixLineOne : _prefixOtherLines;
  }

  /// Write lines assuming the lines obey the specified prefixes. Ensures that
  /// a newline is added if one is not present.
  void writeRawLines(String lines) {
    if (lines.isEmpty) {
      return;
    }

    if (_currentLine.isNotEmpty) {
      _finalizeLine(true);
    }
    assert(_currentLine.isEmpty);

    _buffer.write(lines);
    if (!lines.endsWith('\n')) {
      _buffer.write('\n');
    }
    _numLines++;
    _updatePrefix();
  }

  /// Finishes the current line with a stretched version of text.
  void writeStretched(String text, int targetLineLength) {
    write(text);
    final int currentLineLength = _currentLine.length + _getCurrentPrefix(_buffer.isEmpty)!.length;
    assert(_currentLine.length > 0);
    final int targetLength = targetLineLength - currentLineLength;
    if (targetLength > 0) {
      assert(text.isNotEmpty);
      final String lastChar = text[text.length - 1];
      assert(lastChar != '\n');
      _currentLine.write(lastChar * targetLength);
    }
    // Mark the entire line as not wrappable.
    _wrappableRanges.clear();
  }

  String build() {
    if (_currentLine.isNotEmpty) {
      _finalizeLine(false);
    }

    return _buffer.toString();
  }
}

class _NoDefaultValue {
  const _NoDefaultValue();
}

/// Marker object indicating that a [DiagnosticsNode] has no default value.
const Object kNoDefaultValue = _NoDefaultValue();

bool _isSingleLine(DiagnosticsTreeStyle? style) {
  return style == DiagnosticsTreeStyle.singleLine;
}

/// Renderer that creates ASCII art representations of trees of
/// [DiagnosticsNode] objects.
///
/// See also:
///
///  * [DiagnosticsNode.toStringDeep], which uses a [TextTreeRenderer] to return a
///    string representation of this node and its descendants.
class TextTreeRenderer {
  /// Creates a [TextTreeRenderer] object with the given arguments specifying
  /// how the tree is rendered.
  ///
  /// Lines are wrapped to at the maximum of [wrapWidth] and the current indent
  /// plus [wrapWidthProperties] characters. This ensures that wrapping does not
  /// become too excessive when displaying very deep trees and that wrapping
  /// only occurs at the overall [wrapWidth] when the tree is not very indented.
  /// If [maxDescendentsTruncatableNode] is specified, [DiagnosticsNode] objects
  /// with `allowTruncate` set to `true` are truncated after including
  /// [maxDescendentsTruncatableNode] descendants of the node to be truncated.
  TextTreeRenderer({
    DiagnosticLevel minLevel = DiagnosticLevel.debug,
    int wrapWidth = 100,
    int wrapWidthProperties = 65,
    int maxDescendentsTruncatableNode = -1,
  }) : _minLevel = minLevel,
       _wrapWidth = wrapWidth,
       _wrapWidthProperties = wrapWidthProperties,
       _maxDescendentsTruncatableNode = maxDescendentsTruncatableNode;

  final int _wrapWidth;
  final int _wrapWidthProperties;
  final DiagnosticLevel _minLevel;
  final int _maxDescendentsTruncatableNode;

  /// Text configuration to use to connect this node to a `child`.
  ///
  /// The singleLine styles are special cased because the connection from the
  /// parent to the child should be consistent with the parent's style as the
  /// single line style does not provide any meaningful style for how children
  /// should be connected to their parents.
  TextTreeConfiguration? _childTextConfiguration(
    DiagnosticsNode child,
    TextTreeConfiguration textStyle,
  ) {
    final DiagnosticsTreeStyle? childStyle = child.style;
    return (_isSingleLine(childStyle) || childStyle == DiagnosticsTreeStyle.errorProperty)
        ? textStyle
        : child.textTreeConfiguration;
  }

  /// Renders a [node] to a String.
  String render(
    DiagnosticsNode node, {
    String prefixLineOne = '',
    String? prefixOtherLines,
    TextTreeConfiguration? parentConfiguration,
  }) {
    if (kReleaseMode) {
      return '';
    } else {
      return _debugRender(
        node,
        prefixLineOne: prefixLineOne,
        prefixOtherLines: prefixOtherLines,
        parentConfiguration: parentConfiguration,
      );
    }
  }

  String _debugRender(
    DiagnosticsNode node, {
    String prefixLineOne = '',
    String? prefixOtherLines,
    TextTreeConfiguration? parentConfiguration,
  }) {
    final bool isSingleLine =
        _isSingleLine(node.style) && parentConfiguration?.lineBreakProperties != true;
    prefixOtherLines ??= prefixLineOne;
    if (node.linePrefix != null) {
      prefixLineOne += node.linePrefix!;
      prefixOtherLines += node.linePrefix!;
    }

    final TextTreeConfiguration config = node.textTreeConfiguration!;
    if (prefixOtherLines.isEmpty) {
      prefixOtherLines += config.prefixOtherLinesRootNode;
    }

    if (node.style == DiagnosticsTreeStyle.truncateChildren) {
      // This style is different enough that it isn't worthwhile to reuse the
      // existing logic.
      final List<String> descendants = <String>[];
      const int maxDepth = 5;
      int depth = 0;
      const int maxLines = 25;
      int lines = 0;
      void visitor(DiagnosticsNode node) {
        for (final DiagnosticsNode child in node.getChildren()) {
          if (lines < maxLines) {
            depth += 1;
            descendants.add('$prefixOtherLines${"  " * depth}$child');
            if (depth < maxDepth) {
              visitor(child);
            }
            depth -= 1;
          } else if (lines == maxLines) {
            descendants.add(
              '$prefixOtherLines  ...(descendants list truncated after $lines lines)',
            );
          }
          lines += 1;
        }
      }

      visitor(node);
      final StringBuffer information = StringBuffer(prefixLineOne);
      if (lines > 1) {
        information.writeln(
          'This ${node.name} had the following descendants (showing up to depth $maxDepth):',
        );
      } else if (descendants.length == 1) {
        information.writeln('This ${node.name} had the following child:');
      } else {
        information.writeln('This ${node.name} has no descendants.');
      }
      information.writeAll(descendants, '\n');
      return information.toString();
    }
    final _PrefixedStringBuilder builder = _PrefixedStringBuilder(
      prefixLineOne: prefixLineOne,
      prefixOtherLines: prefixOtherLines,
      wrapWidth: math.max(_wrapWidth, prefixOtherLines.length + _wrapWidthProperties),
    );

    List<DiagnosticsNode> children = node.getChildren();

    String description = node.toDescription(parentConfiguration: parentConfiguration);
    if (config.beforeName.isNotEmpty) {
      builder.write(config.beforeName);
    }
    final bool wrapName = !isSingleLine && node.allowNameWrap;
    final bool wrapDescription = !isSingleLine && node.allowWrap;
    final bool uppercaseTitle = node.style == DiagnosticsTreeStyle.error;
    String? name = node.name;
    if (uppercaseTitle) {
      name = name?.toUpperCase();
    }
    if (description.isEmpty) {
      if (node.showName && name != null) {
        builder.write(name, allowWrap: wrapName);
      }
    } else {
      bool includeName = false;
      if (name != null && name.isNotEmpty && node.showName) {
        includeName = true;
        builder.write(name, allowWrap: wrapName);
        if (node.showSeparator) {
          builder.write(config.afterName, allowWrap: wrapName);
        }

        builder.write(
          config.isNameOnOwnLine || description.contains('\n') ? '\n' : ' ',
          allowWrap: wrapName,
        );
      }
      if (!isSingleLine && builder.requiresMultipleLines && !builder.isCurrentLineEmpty) {
        // Make sure there is a break between the current line and next one if
        // there is not one already.
        builder.write('\n');
      }
      if (includeName) {
        builder.incrementPrefixOtherLines(
          children.isEmpty ? config.propertyPrefixNoChildren : config.propertyPrefixIfChildren,
          updateCurrentLine: true,
        );
      }

      if (uppercaseTitle) {
        description = description.toUpperCase();
      }
      builder.write(description.trimRight(), allowWrap: wrapDescription);

      if (!includeName) {
        builder.incrementPrefixOtherLines(
          children.isEmpty ? config.propertyPrefixNoChildren : config.propertyPrefixIfChildren,
          updateCurrentLine: false,
        );
      }
    }
    if (config.suffixLineOne.isNotEmpty) {
      builder.writeStretched(config.suffixLineOne, builder.wrapWidth!);
    }

    final Iterable<DiagnosticsNode> propertiesIterable = node.getProperties().where(
      (DiagnosticsNode n) => !n.isFiltered(_minLevel),
    );
    List<DiagnosticsNode> properties;
    if (_maxDescendentsTruncatableNode >= 0 && node.allowTruncate) {
      if (propertiesIterable.length < _maxDescendentsTruncatableNode) {
        properties = propertiesIterable.take(_maxDescendentsTruncatableNode).toList();
        properties.add(DiagnosticsNode.message('...'));
      } else {
        properties = propertiesIterable.toList();
      }
      if (_maxDescendentsTruncatableNode < children.length) {
        children = children.take(_maxDescendentsTruncatableNode).toList();
        children.add(DiagnosticsNode.message('...'));
      }
    } else {
      properties = propertiesIterable.toList();
    }

    // If the node does not show a separator and there is no description then
    // we should not place a separator between the name and the value.
    // Essentially in this case the properties are treated a bit like a value.
    if ((properties.isNotEmpty || children.isNotEmpty || node.emptyBodyDescription != null) &&
        (node.showSeparator || description.isNotEmpty)) {
      builder.write(config.afterDescriptionIfBody);
    }

    if (config.lineBreakProperties) {
      builder.write(config.lineBreak);
    }

    if (properties.isNotEmpty) {
      builder.write(config.beforeProperties);
    }

    builder.incrementPrefixOtherLines(config.bodyIndent, updateCurrentLine: false);

    if (node.emptyBodyDescription != null &&
        properties.isEmpty &&
        children.isEmpty &&
        prefixLineOne.isNotEmpty) {
      builder.write(node.emptyBodyDescription!);
      if (config.lineBreakProperties) {
        builder.write(config.lineBreak);
      }
    }

    for (int i = 0; i < properties.length; ++i) {
      final DiagnosticsNode property = properties[i];
      if (i > 0) {
        builder.write(config.propertySeparator);
      }

      final TextTreeConfiguration propertyStyle = property.textTreeConfiguration!;
      if (_isSingleLine(property.style)) {
        // We have to treat single line properties slightly differently to deal
        // with cases where a single line properties output may not have single
        // linebreak.
        final String propertyRender = render(
          property,
          prefixLineOne: propertyStyle.prefixLineOne,
          prefixOtherLines: '${propertyStyle.childLinkSpace}${propertyStyle.prefixOtherLines}',
          parentConfiguration: config,
        );
        final List<String> propertyLines = propertyRender.split('\n');
        if (propertyLines.length == 1 && !config.lineBreakProperties) {
          builder.write(propertyLines.first);
        } else {
          builder.write(propertyRender);
          if (!propertyRender.endsWith('\n')) {
            builder.write('\n');
          }
        }
      } else {
        final String propertyRender = render(
          property,
          prefixLineOne: '${builder.prefixOtherLines}${propertyStyle.prefixLineOne}',
          prefixOtherLines:
              '${builder.prefixOtherLines}${propertyStyle.childLinkSpace}${propertyStyle.prefixOtherLines}',
          parentConfiguration: config,
        );
        builder.writeRawLines(propertyRender);
      }
    }
    if (properties.isNotEmpty) {
      builder.write(config.afterProperties);
    }

    builder.write(config.mandatoryAfterProperties);

    if (!config.lineBreakProperties) {
      builder.write(config.lineBreak);
    }

    final String prefixChildren = config.bodyIndent;
    final String prefixChildrenRaw = '$prefixOtherLines$prefixChildren';
    if (children.isEmpty &&
        config.addBlankLineIfNoChildren &&
        builder.requiresMultipleLines &&
        builder.prefixOtherLines!.trimRight().isNotEmpty) {
      builder.write(config.lineBreak);
    }

    if (children.isNotEmpty && config.showChildren) {
      if (config.isBlankLineBetweenPropertiesAndChildren &&
          properties.isNotEmpty &&
          children.first.textTreeConfiguration!.isBlankLineBetweenPropertiesAndChildren) {
        builder.write(config.lineBreak);
      }

      builder.prefixOtherLines = prefixOtherLines;

      for (int i = 0; i < children.length; i++) {
        final DiagnosticsNode child = children[i];
        final TextTreeConfiguration childConfig = _childTextConfiguration(child, config)!;
        if (i == children.length - 1) {
          final String lastChildPrefixLineOne =
              '$prefixChildrenRaw${childConfig.prefixLastChildLineOne}';
          final String childPrefixOtherLines =
              '$prefixChildrenRaw${childConfig.childLinkSpace}${childConfig.prefixOtherLines}';
          builder.writeRawLines(
            render(
              child,
              prefixLineOne: lastChildPrefixLineOne,
              prefixOtherLines: childPrefixOtherLines,
              parentConfiguration: config,
            ),
          );
          if (childConfig.footer.isNotEmpty) {
            builder.prefixOtherLines = prefixChildrenRaw;
            builder.write('${childConfig.childLinkSpace}${childConfig.footer}');
            if (childConfig.mandatoryFooter.isNotEmpty) {
              builder.writeStretched(
                childConfig.mandatoryFooter,
                math.max(builder.wrapWidth!, _wrapWidthProperties + childPrefixOtherLines.length),
              );
            }
            builder.write(config.lineBreak);
          }
        } else {
          final TextTreeConfiguration nextChildStyle =
              _childTextConfiguration(children[i + 1], config)!;
          final String childPrefixLineOne = '$prefixChildrenRaw${childConfig.prefixLineOne}';
          final String childPrefixOtherLines =
              '$prefixChildrenRaw${nextChildStyle.linkCharacter}${childConfig.prefixOtherLines}';
          builder.writeRawLines(
            render(
              child,
              prefixLineOne: childPrefixLineOne,
              prefixOtherLines: childPrefixOtherLines,
              parentConfiguration: config,
            ),
          );
          if (childConfig.footer.isNotEmpty) {
            builder.prefixOtherLines = prefixChildrenRaw;
            builder.write('${childConfig.linkCharacter}${childConfig.footer}');
            if (childConfig.mandatoryFooter.isNotEmpty) {
              builder.writeStretched(
                childConfig.mandatoryFooter,
                math.max(builder.wrapWidth!, _wrapWidthProperties + childPrefixOtherLines.length),
              );
            }
            builder.write(config.lineBreak);
          }
        }
      }
    }
    if (parentConfiguration == null && config.mandatoryFooter.isNotEmpty) {
      builder.writeStretched(config.mandatoryFooter, builder.wrapWidth!);
      builder.write(config.lineBreak);
    }
    return builder.build();
  }
}

/// The JSON representation of a [DiagnosticsNode].
typedef _JsonDiagnosticsNode = Map<String, Object?>;

/// Stack containing [DiagnosticNode]s to convert to JSON and the callback to
/// call with the JSON.
///
/// Using a stack is required to process the widget tree iteratively instead of
/// recursively.
typedef _NodesToJsonifyStack = ListQueue<(DiagnosticsNode, void Function(_JsonDiagnosticsNode))>;

/// Defines diagnostics data for a [value].
///
/// For debug and profile modes, [DiagnosticsNode] provides a high quality
/// multiline string dump via [toStringDeep]. The core members are the [name],
/// [toDescription], [getProperties], [value], and [getChildren]. All other
/// members exist typically to provide hints for how [toStringDeep] and
/// debugging tools should format output.
///
/// In release mode, far less information is retained and some information may
/// not print at all.
abstract class DiagnosticsNode {
  /// Initializes the object.
  ///
  /// The [style], [showName], and [showSeparator] arguments must not
  /// be null.
  DiagnosticsNode({
    required this.name,
    this.style,
    this.showName = true,
    this.showSeparator = true,
    this.linePrefix,
  }) : assert(
         // A name ending with ':' indicates that the user forgot that the ':' will
         // be automatically added for them when generating descriptions of the
         // property.
         name == null || !name.endsWith(':'),
         'Names of diagnostic nodes must not end with colons.\n'
         'name:\n'
         '  "$name"',
       );

  /// Diagnostics containing just a string `message` and not a concrete name or
  /// value.
  ///
  /// See also:
  ///
  ///  * [MessageProperty], which is better suited to messages that are to be
  ///    formatted like a property with a separate name and message.
  factory DiagnosticsNode.message(
    String message, {
    DiagnosticsTreeStyle style = DiagnosticsTreeStyle.singleLine,
    DiagnosticLevel level = DiagnosticLevel.info,
    bool allowWrap = true,
  }) {
    return DiagnosticsProperty<void>(
      '',
      null,
      description: message,
      style: style,
      showName: false,
      allowWrap: allowWrap,
      level: level,
    );
  }

  /// Label describing the [DiagnosticsNode], typically shown before a separator
  /// (see [showSeparator]).
  ///
  /// The name will be omitted if the [showName] property is false.
  final String? name;

  /// Returns a description with a short summary of the node itself not
  /// including children or properties.
  ///
  /// `parentConfiguration` specifies how the parent is rendered as text art.
  /// For example, if the parent does not line break between properties, the
  /// description of a property should also be a single line if possible.
  String toDescription({TextTreeConfiguration? parentConfiguration});

  /// Whether to show a separator between [name] and description.
  ///
  /// If false, name and description should be shown with no separation.
  /// `:` is typically used as a separator when displaying as text.
  final bool showSeparator;

  /// Whether the diagnostic should be filtered due to its [level] being lower
  /// than `minLevel`.
  ///
  /// If `minLevel` is [DiagnosticLevel.hidden] no diagnostics will be filtered.
  /// If `minLevel` is [DiagnosticLevel.off] all diagnostics will be filtered.
  bool isFiltered(DiagnosticLevel minLevel) => kReleaseMode || level.index < minLevel.index;

  /// Priority level of the diagnostic used to control which diagnostics should
  /// be shown and filtered.
  ///
  /// Typically this only makes sense to set to a different value than
  /// [DiagnosticLevel.info] for diagnostics representing properties. Some
  /// subclasses have a [level] argument to their constructor which influences
  /// the value returned here but other factors also influence it. For example,
  /// whether an exception is thrown computing a property value
  /// [DiagnosticLevel.error] is returned.
  DiagnosticLevel get level => kReleaseMode ? DiagnosticLevel.hidden : DiagnosticLevel.info;

  /// Whether the name of the property should be shown when showing the default
  /// view of the tree.
  ///
  /// This could be set to false (hiding the name) if the value's description
  /// will make the name self-evident.
  final bool showName;

  /// Prefix to include at the start of each line.
  final String? linePrefix;

  /// Description to show if the node has no displayed properties or children.
  String? get emptyBodyDescription => null;

  /// The actual object this is diagnostics data for.
  Object? get value;

  /// Hint for how the node should be displayed.
  final DiagnosticsTreeStyle? style;

  /// Whether to wrap text on onto multiple lines or not.
  bool get allowWrap => false;

  /// Whether to wrap the name onto multiple lines or not.
  bool get allowNameWrap => false;

  /// Whether to allow truncation when displaying the node and its children.
  bool get allowTruncate => false;

  /// Properties of this [DiagnosticsNode].
  ///
  /// Properties and children are kept distinct even though they are both
  /// [List<DiagnosticsNode>] because they should be grouped differently.
  List<DiagnosticsNode> getProperties();

  /// Children of this [DiagnosticsNode].
  ///
  /// See also:
  ///
  ///  * [getProperties], which returns the properties of the [DiagnosticsNode]
  ///    object.
  List<DiagnosticsNode> getChildren();

  String get _separator => showSeparator ? ':' : '';

  /// Converts the properties ([getProperties]) of this node to a form useful
  /// for [Timeline] event arguments (as in [Timeline.startSync]).
  ///
  /// Children ([getChildren]) are omitted.
  ///
  /// This method is only valid in debug builds. In profile builds, this method
  /// throws an exception. In release builds it returns null.
  ///
  /// See also:
  ///
  ///  * [toJsonMap], which converts this node to a structured form intended for
  ///    data exchange (e.g. with an IDE).
  Map<String, String>? toTimelineArguments() {
    if (!kReleaseMode) {
      // We don't throw in release builds, to avoid hurting users. We also don't do anything useful.
      if (kProfileMode) {
        throw FlutterError(
          // Parts of this string are searched for verbatim by a test in dev/bots/test.dart.
          '$DiagnosticsNode.toTimelineArguments used in non-debug build.\n'
          'The $DiagnosticsNode.toTimelineArguments API is expensive and causes timeline traces '
          'to be non-representative. As such, it should not be used in profile builds. However, '
          'this application is compiled in profile mode and yet still invoked the method.',
        );
      }
      final Map<String, String> result = <String, String>{};
      for (final DiagnosticsNode property in getProperties()) {
        if (property.name != null) {
          result[property.name!] = property.toDescription(
            parentConfiguration: singleLineTextConfiguration,
          );
        }
      }
      return result;
    }
    return null;
  }

  /// Serialize the node to a JSON map according to the configuration provided
  /// in the [DiagnosticsSerializationDelegate].
  ///
  /// Subclasses should override if they have additional properties that are
  /// useful for the GUI tools that consume this JSON.
  ///
  /// See also:
  ///
  ///  * [WidgetInspectorService], which forms the bridge between JSON returned
  ///    by this method and interactive tree views in the Flutter IntelliJ
  ///    plugin.
  @mustCallSuper
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    Map<String, Object?> result = <String, Object?>{};
    assert(() {
      final bool hasChildren = getChildren().isNotEmpty;
      result = <String, Object?>{
        'description': toDescription(),
        'type': runtimeType.toString(),
        if (name != null) 'name': name,
        if (!showSeparator) 'showSeparator': showSeparator,
        if (level != DiagnosticLevel.info) 'level': level.name,
        if (!showName) 'showName': showName,
        if (emptyBodyDescription != null) 'emptyBodyDescription': emptyBodyDescription,
        if (style != DiagnosticsTreeStyle.sparse) 'style': style!.name,
        if (allowTruncate) 'allowTruncate': allowTruncate,
        if (hasChildren) 'hasChildren': hasChildren,
        if (linePrefix?.isNotEmpty ?? false) 'linePrefix': linePrefix,
        if (!allowWrap) 'allowWrap': allowWrap,
        if (allowNameWrap) 'allowNameWrap': allowNameWrap,
        ...delegate.additionalNodeProperties(this),
        if (delegate.includeProperties)
          'properties': toJsonList(
            delegate.filterProperties(getProperties(), this),
            this,
            delegate,
          ),
        if (delegate.subtreeDepth > 0)
          'children': toJsonList(delegate.filterChildren(getChildren(), this), this, delegate),
      };
      return true;
    }());
    return result;
  }

  /// Iteratively serialize the node to a JSON map according to the
  /// configuration provided in the [DiagnosticsSerializationDelegate].
  ///
  /// This is only used when [WidgetInspectorServiceExtensions.getRootWidgetTree]
  /// is called with fullDetails=false. To get the full widget details, including
  /// any details provided by subclasses, [toJsonMap] should be used instead.
  ///
  /// See https://github.com/flutter/devtools/issues/8553 for details about this
  /// iterative approach.
  Map<String, Object?> toJsonMapIterative(DiagnosticsSerializationDelegate delegate) {
    final _NodesToJsonifyStack childrenToJsonify =
        ListQueue<(DiagnosticsNode, void Function(_JsonDiagnosticsNode))>();
    _JsonDiagnosticsNode result = <String, Object?>{};
    assert(() {
      result = _toJson(delegate, childrenToJsonify: childrenToJsonify);
      _jsonifyNextNodesInStack(childrenToJsonify, delegate: delegate);
      return true;
    }());
    return result;
  }

  /// Serializes a [List] of [DiagnosticsNode]s to a JSON list according to
  /// the configuration provided by the [DiagnosticsSerializationDelegate].
  ///
  /// The provided `nodes` may be properties or children of the `parent`
  /// [DiagnosticsNode].
  static List<Map<String, Object?>> toJsonList(
    List<DiagnosticsNode>? nodes,
    DiagnosticsNode? parent,
    DiagnosticsSerializationDelegate delegate,
  ) {
    bool truncated = false;
    if (nodes == null) {
      return const <Map<String, Object?>>[];
    }
    final int originalNodeCount = nodes.length;
    nodes = delegate.truncateNodesList(nodes, parent);
    if (nodes.length != originalNodeCount) {
      nodes.add(DiagnosticsNode.message('...'));
      truncated = true;
    }
    final List<_JsonDiagnosticsNode> json =
        nodes.map<_JsonDiagnosticsNode>((DiagnosticsNode node) {
          return node.toJsonMap(delegate.delegateForNode(node));
        }).toList();
    if (truncated) {
      json.last['truncated'] = true;
    }
    return json;
  }

  /// Returns a string representation of this diagnostic that is compatible with
  /// the style of the parent if the node is not the root.
  ///
  /// `parentConfiguration` specifies how the parent is rendered as text art.
  /// For example, if the parent places all properties on one line, the
  /// [toString] for each property should avoid line breaks if possible.
  ///
  /// `minLevel` specifies the minimum [DiagnosticLevel] for properties included
  /// in the output.
  ///
  /// In release mode, far less information is retained and some information may
  /// not print at all.
  @override
  String toString({
    TextTreeConfiguration? parentConfiguration,
    DiagnosticLevel minLevel = DiagnosticLevel.info,
  }) {
    String result = super.toString();
    assert(style != null);
    assert(() {
      if (_isSingleLine(style)) {
        result = toStringDeep(parentConfiguration: parentConfiguration, minLevel: minLevel);
      } else {
        final String description = toDescription(parentConfiguration: parentConfiguration);

        if (name == null || name!.isEmpty || !showName) {
          result = description;
        } else {
          result =
              description.contains('\n')
                  ? '$name$_separator\n$description'
                  : '$name$_separator $description';
        }
      }
      return true;
    }());
    return result;
  }

  /// Returns a configuration specifying how this object should be rendered
  /// as text art.
  @protected
  TextTreeConfiguration? get textTreeConfiguration {
    assert(style != null);
    return switch (style!) {
      DiagnosticsTreeStyle.none => null,
      DiagnosticsTreeStyle.dense => denseTextConfiguration,
      DiagnosticsTreeStyle.sparse => sparseTextConfiguration,
      DiagnosticsTreeStyle.offstage => dashedTextConfiguration,
      DiagnosticsTreeStyle.whitespace => whitespaceTextConfiguration,
      DiagnosticsTreeStyle.transition => transitionTextConfiguration,
      DiagnosticsTreeStyle.singleLine => singleLineTextConfiguration,
      DiagnosticsTreeStyle.errorProperty => errorPropertyTextConfiguration,
      DiagnosticsTreeStyle.shallow => shallowTextConfiguration,
      DiagnosticsTreeStyle.error => errorTextConfiguration,
      DiagnosticsTreeStyle.flat => flatTextConfiguration,

      // Truncate children doesn't really need its own text style as the
      // rendering is quite custom.
      DiagnosticsTreeStyle.truncateChildren => whitespaceTextConfiguration,
    };
  }

  /// Returns a string representation of this node and its descendants.
  ///
  /// `prefixLineOne` will be added to the front of the first line of the
  /// output. `prefixOtherLines` will be added to the front of each other line.
  /// If `prefixOtherLines` is null, the `prefixLineOne` is used for every line.
  /// By default, there is no prefix.
  ///
  /// `minLevel` specifies the minimum [DiagnosticLevel] for properties included
  /// in the output.
  ///
  /// `wrapWidth` specifies the column number where word wrapping will be
  /// applied.
  ///
  /// The [toStringDeep] method takes other arguments, but those are intended
  /// for internal use when recursing to the descendants, and so can be ignored.
  ///
  /// In release mode, far less information is retained and some information may
  /// not print at all.
  ///
  /// See also:
  ///
  ///  * [toString], for a brief description of the [value] but not its
  ///    children.
  String toStringDeep({
    String prefixLineOne = '',
    String? prefixOtherLines,
    TextTreeConfiguration? parentConfiguration,
    DiagnosticLevel minLevel = DiagnosticLevel.debug,
    int wrapWidth = 65,
  }) {
    String result = '';
    assert(() {
      result = TextTreeRenderer(minLevel: minLevel, wrapWidth: wrapWidth).render(
        this,
        prefixLineOne: prefixLineOne,
        prefixOtherLines: prefixOtherLines,
        parentConfiguration: parentConfiguration,
      );
      return true;
    }());
    return result;
  }

  void _jsonifyNextNodesInStack(
    _NodesToJsonifyStack toJsonify, {
    required DiagnosticsSerializationDelegate delegate,
  }) {
    while (toJsonify.isNotEmpty) {
      final (DiagnosticsNode nextNode, void Function(_JsonDiagnosticsNode) callback) =
          toJsonify.removeFirst();
      final _JsonDiagnosticsNode nodeAsJson = nextNode._toJson(
        delegate,
        childrenToJsonify: toJsonify,
      );
      callback(nodeAsJson);
    }
  }

  Map<String, Object?> _toJson(
    DiagnosticsSerializationDelegate delegate, {
    required _NodesToJsonifyStack childrenToJsonify,
  }) {
    final List<_JsonDiagnosticsNode> childrenJsonList = <_JsonDiagnosticsNode>[];
    final bool includeChildren = getChildren().isNotEmpty && delegate.subtreeDepth > 0;

    // Collect the children nodes to convert to JSON later.
    bool truncated = false;
    if (includeChildren) {
      List<DiagnosticsNode> childrenNodes = delegate.filterChildren(getChildren(), this);
      final int originalNodeCount = childrenNodes.length;
      childrenNodes = delegate.truncateNodesList(childrenNodes, this);
      if (childrenNodes.length != originalNodeCount) {
        childrenNodes.add(DiagnosticsNode.message('...'));
        truncated = true;
      }
      for (final DiagnosticsNode child in childrenNodes) {
        childrenToJsonify.add((
          child,
          (_JsonDiagnosticsNode jsonChild) {
            childrenJsonList.add(jsonChild);
          },
        ));
      }
    }

    final String description = toDescription();
    final String widgetRuntimeType =
        description == '[root]' ? 'RootWidget' : description.split('-').first;
    final bool shouldIndent =
        style != DiagnosticsTreeStyle.flat && style != DiagnosticsTreeStyle.error;

    return <String, Object?>{
      'description': description,
      'shouldIndent': shouldIndent,
      // TODO(elliette): This can be removed to reduce the JSON response even
      // further once DevTools computes the widget runtime type from the
      // description instead, see:
      // https://github.com/flutter/devtools/issues/8556
      'widgetRuntimeType': widgetRuntimeType,
      if (truncated) 'truncated': truncated,
      ...delegate.additionalNodeProperties(this, fullDetails: false),
      if (includeChildren) 'children': childrenJsonList,
    };
  }
}

/// Debugging message displayed like a property.
///
/// {@tool snippet}
///
/// The following two properties are better expressed using this
/// [MessageProperty] class, rather than [StringProperty], as the intent is to
/// show a message with property style display rather than to describe the value
/// of an actual property of the object:
///
/// ```dart
/// MessageProperty table = MessageProperty('table size', '$columns\u00D7$rows');
/// MessageProperty usefulness = MessageProperty('usefulness ratio', 'no metrics collected yet (never painted)');
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// On the other hand, [StringProperty] is better suited when the property has a
/// concrete value that is a string:
///
/// ```dart
/// StringProperty name = StringProperty('name', _name);
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [DiagnosticsNode.message], which serves the same role for messages
///    without a clear property name.
///  * [StringProperty], which is a better fit for properties with string values.
class MessageProperty extends DiagnosticsProperty<void> {
  /// Create a diagnostics property that displays a message.
  ///
  /// Messages have no concrete [value] (so [value] will return null). The
  /// message is stored as the description.
  MessageProperty(
    String name,
    String message, {
    DiagnosticsTreeStyle style = DiagnosticsTreeStyle.singleLine,
    DiagnosticLevel level = DiagnosticLevel.info,
  }) : super(name, null, description: message, style: style, level: level);
}

/// Property which encloses its string [value] in quotes.
///
/// See also:
///
///  * [MessageProperty], which is a better fit for showing a message
///    instead of describing a property with a string value.
class StringProperty extends DiagnosticsProperty<String> {
  /// Create a diagnostics property for strings.
  StringProperty(
    String super.name,
    super.value, {
    super.description,
    super.tooltip,
    super.showName,
    super.defaultValue,
    this.quoted = true,
    super.ifEmpty,
    super.style,
    super.level,
  });

  /// Whether the value is enclosed in double quotes.
  final bool quoted;

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final Map<String, Object?> json = super.toJsonMap(delegate);
    json['quoted'] = quoted;
    return json;
  }

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    String? text = _description ?? value;
    if (parentConfiguration != null && !parentConfiguration.lineBreakProperties && text != null) {
      // Escape linebreaks in multiline strings to avoid confusing output when
      // the parent of this node is trying to display all properties on the same
      // line.
      text = text.replaceAll('\n', r'\n');
    }

    if (quoted && text != null) {
      // An empty value would not appear empty after being surrounded with
      // quotes so we have to handle this case separately.
      if (ifEmpty != null && text.isEmpty) {
        return ifEmpty!;
      }
      return '"$text"';
    }
    return text.toString();
  }
}

abstract class _NumProperty<T extends num> extends DiagnosticsProperty<T> {
  _NumProperty(
    String super.name,
    super.value, {
    super.ifNull,
    this.unit,
    super.showName,
    super.defaultValue,
    super.tooltip,
    super.style,
    super.level,
  });

  _NumProperty.lazy(
    String super.name,
    super.computeValue, {
    super.ifNull,
    this.unit,
    super.showName,
    super.defaultValue,
    super.tooltip,
    super.style,
    super.level,
  }) : super.lazy();

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final Map<String, Object?> json = super.toJsonMap(delegate);
    if (unit != null) {
      json['unit'] = unit;
    }

    json['numberToString'] = numberToString();
    return json;
  }

  /// Optional unit the [value] is measured in.
  ///
  /// Unit must be acceptable to display immediately after a number with no
  /// spaces. For example: 'physical pixels per logical pixel' should be a
  /// [tooltip] not a [unit].
  final String? unit;

  /// String describing just the numeric [value] without a unit suffix.
  String numberToString();

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    if (value == null) {
      return value.toString();
    }

    return unit != null ? '${numberToString()}$unit' : numberToString();
  }
}

/// Property describing a [double] [value] with an optional [unit] of measurement.
///
/// Numeric formatting is optimized for debug message readability.
class DoubleProperty extends _NumProperty<double> {
  /// If specified, [unit] describes the unit for the [value] (e.g. px).
  DoubleProperty(
    super.name,
    super.value, {
    super.ifNull,
    super.unit,
    super.tooltip,
    super.defaultValue,
    super.showName,
    super.style,
    super.level,
  });

  /// Property with a [value] that is computed only when needed.
  ///
  /// Use if computing the property [value] may throw an exception or is
  /// expensive.
  DoubleProperty.lazy(
    super.name,
    super.computeValue, {
    super.ifNull,
    super.showName,
    super.unit,
    super.tooltip,
    super.defaultValue,
    super.level,
  }) : super.lazy();

  @override
  String numberToString() => debugFormatDouble(value);
}

/// An int valued property with an optional unit the value is measured in.
///
/// Examples of units include 'px' and 'ms'.
class IntProperty extends _NumProperty<int> {
  /// Create a diagnostics property for integers.
  IntProperty(
    super.name,
    super.value, {
    super.ifNull,
    super.showName,
    super.unit,
    super.defaultValue,
    super.style,
    super.level,
  });

  @override
  String numberToString() => value.toString();
}

/// Property which clamps a [double] to between 0 and 1 and formats it as a
/// percentage.
class PercentProperty extends DoubleProperty {
  /// Create a diagnostics property for doubles that represent percentages or
  /// fractions.
  ///
  /// Setting [showName] to false is often reasonable for [PercentProperty]
  /// objects, as the fact that the property is shown as a percentage tends to
  /// be sufficient to disambiguate its meaning.
  PercentProperty(
    super.name,
    super.fraction, {
    super.ifNull,
    super.showName,
    super.tooltip,
    super.unit,
    super.level,
  });

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    if (value == null) {
      return value.toString();
    }
    return unit != null ? '${numberToString()} $unit' : numberToString();
  }

  @override
  String numberToString() {
    final double? v = value;
    if (v == null) {
      return value.toString();
    }
    return '${(clampDouble(v, 0.0, 1.0) * 100.0).toStringAsFixed(1)}%';
  }
}

/// Property where the description is either [ifTrue] or [ifFalse] depending on
/// whether [value] is true or false.
///
/// Using [FlagProperty] instead of [DiagnosticsProperty<bool>] can make
/// diagnostics display more polished. For example, given a property named
/// `visible` that is typically true, the following code will return 'hidden'
/// when `visible` is false and nothing when visible is true, in contrast to
/// `visible: true` or `visible: false`.
///
/// {@tool snippet}
///
/// ```dart
/// FlagProperty(
///   'visible',
///   value: true,
///   ifFalse: 'hidden',
/// )
/// ```
/// {@end-tool}
/// {@tool snippet}
///
/// [FlagProperty] should also be used instead of [DiagnosticsProperty<bool>]
/// if showing the bool value would not clearly indicate the meaning of the
/// property value.
///
/// ```dart
/// FlagProperty(
///   'inherit',
///   value: inherit,
///   ifTrue: '<all styles inherited>',
///   ifFalse: '<no style specified>',
/// )
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [ObjectFlagProperty], which provides similar behavior describing whether
///    a [value] is null.
class FlagProperty extends DiagnosticsProperty<bool> {
  /// Constructs a FlagProperty with the given descriptions with the specified descriptions.
  ///
  /// [showName] defaults to false as typically [ifTrue] and [ifFalse] should
  /// be descriptions that make the property name redundant.
  FlagProperty(
    String name, {
    required bool? value,
    this.ifTrue,
    this.ifFalse,
    bool showName = false,
    Object? defaultValue,
    DiagnosticLevel level = DiagnosticLevel.info,
  }) : assert(ifTrue != null || ifFalse != null),
       super(name, value, showName: showName, defaultValue: defaultValue, level: level);

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final Map<String, Object?> json = super.toJsonMap(delegate);
    if (ifTrue != null) {
      json['ifTrue'] = ifTrue;
    }
    if (ifFalse != null) {
      json['ifFalse'] = ifFalse;
    }

    return json;
  }

  /// Description to use if the property [value] is true.
  ///
  /// If not specified and [value] equals true the property's priority [level]
  /// will be [DiagnosticLevel.hidden].
  final String? ifTrue;

  /// Description to use if the property value is false.
  ///
  /// If not specified and [value] equals false, the property's priority [level]
  /// will be [DiagnosticLevel.hidden].
  final String? ifFalse;

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    return switch (value) {
      true when ifTrue != null => ifTrue!,
      false when ifFalse != null => ifFalse!,
      _ => super.valueToString(parentConfiguration: parentConfiguration),
    };
  }

  @override
  bool get showName {
    if (value == null ||
        ((value ?? false) && ifTrue == null) ||
        (!(value ?? true) && ifFalse == null)) {
      // We are missing a description for the flag value so we need to show the
      // flag name. The property will have DiagnosticLevel.hidden for this case
      // so users will not see this property in this case unless they are
      // displaying hidden properties.
      return true;
    }
    return super.showName;
  }

  @override
  DiagnosticLevel get level => switch (value) {
    true when ifTrue == null => DiagnosticLevel.hidden,
    false when ifFalse == null => DiagnosticLevel.hidden,
    _ => super.level,
  };
}

/// Property with an `Iterable<T>` [value] that can be displayed with
/// different [DiagnosticsTreeStyle] for custom rendering.
///
/// If [style] is [DiagnosticsTreeStyle.singleLine], the iterable is described
/// as a comma separated list, otherwise the iterable is described as a line
/// break separated list.
class IterableProperty<T> extends DiagnosticsProperty<Iterable<T>> {
  /// Create a diagnostics property for iterables (e.g. lists).
  ///
  /// The [ifEmpty] argument is used to indicate how an iterable [value] with 0
  /// elements is displayed. If [ifEmpty] equals null that indicates that an
  /// empty iterable [value] is not interesting to display similar to how
  /// [defaultValue] is used to indicate that a specific concrete value is not
  /// interesting to display.
  IterableProperty(
    String super.name,
    super.value, {
    super.defaultValue,
    super.ifNull,
    super.ifEmpty = '[]',
    super.style,
    super.showName,
    super.showSeparator,
    super.level,
  });

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    if (value == null) {
      return value.toString();
    }

    if (value!.isEmpty) {
      return ifEmpty ?? '[]';
    }

    final Iterable<String> formattedValues = value!.map((T v) {
      if (T == double && v is double) {
        return debugFormatDouble(v);
      } else {
        return v.toString();
      }
    });

    if (parentConfiguration != null && !parentConfiguration.lineBreakProperties) {
      // Always display the value as a single line and enclose the iterable
      // value in brackets to avoid ambiguity.
      return '[${formattedValues.join(', ')}]';
    }

    return formattedValues.join(_isSingleLine(style) ? ', ' : '\n');
  }

  /// Priority level of the diagnostic used to control which diagnostics should
  /// be shown and filtered.
  ///
  /// If [ifEmpty] is null and the [value] is an empty [Iterable] then level
  /// [DiagnosticLevel.fine] is returned in a similar way to how an
  /// [ObjectFlagProperty] handles when [ifNull] is null and the [value] is
  /// null.
  @override
  DiagnosticLevel get level {
    if (ifEmpty == null &&
        value != null &&
        value!.isEmpty &&
        super.level != DiagnosticLevel.hidden) {
      return DiagnosticLevel.fine;
    }
    return super.level;
  }

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final Map<String, Object?> json = super.toJsonMap(delegate);
    if (value != null) {
      json['values'] = value!.map<String>((T value) => value.toString()).toList();
    }
    return json;
  }
}

/// [DiagnosticsProperty] that has an [Enum] as value.
///
/// The enum value is displayed with the enum name stripped. For example:
/// [HitTestBehavior.deferToChild] is shown as `deferToChild`.
///
/// This class can be used with enums and returns the enum's name getter. It
/// can also be used with nullable properties; the null value is represented as
/// `null`.
///
/// See also:
///
///  * [DiagnosticsProperty] which documents named parameters common to all
///    [DiagnosticsProperty].
class EnumProperty<T extends Enum?> extends DiagnosticsProperty<T> {
  /// Create a diagnostics property that displays an enum.
  ///
  /// The [level] argument must also not be null.
  EnumProperty(String super.name, super.value, {super.defaultValue, super.level});

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    return value?.name ?? 'null';
  }
}

/// A property where the important diagnostic information is primarily whether
/// the [value] is present (non-null) or absent (null), rather than the actual
/// value of the property itself.
///
/// The [ifPresent] and [ifNull] strings describe the property [value] when it
/// is non-null and null respectively. If one of [ifPresent] or [ifNull] is
/// omitted, that is taken to mean that [level] should be
/// [DiagnosticLevel.hidden] when [value] is non-null or null respectively.
///
/// This kind of diagnostics property is typically used for opaque
/// values, like closures, where presenting the actual object is of dubious
/// value but where reporting the presence or absence of the value is much more
/// useful.
///
/// See also:
///
///
///  * [FlagsSummary], which provides similar functionality but accepts multiple
///    flags under the same name, and is preferred if there are multiple such
///    values that can fit into a same category (such as "listeners").
///  * [FlagProperty], which provides similar functionality describing whether
///    a [value] is true or false.
class ObjectFlagProperty<T> extends DiagnosticsProperty<T> {
  /// Create a diagnostics property for values that can be present (non-null) or
  /// absent (null), but for which the exact value's [Object.toString]
  /// representation is not very transparent (e.g. a callback).
  ///
  /// At least one of [ifPresent] or [ifNull] must be non-null.
  ObjectFlagProperty(
    String super.name,
    super.value, {
    this.ifPresent,
    super.ifNull,
    super.showName = false,
    super.level,
  }) : assert(ifPresent != null || ifNull != null);

  /// Shorthand constructor to describe whether the property has a value.
  ///
  /// Only use if prefixing the property name with the word 'has' is a good
  /// flag name.
  ObjectFlagProperty.has(String super.name, super.value, {super.level})
    : ifPresent = 'has $name',
      super(showName: false);

  /// Description to use if the property [value] is not null.
  ///
  /// If the property [value] is not null and [ifPresent] is null, the
  /// [level] for the property is [DiagnosticLevel.hidden] and the description
  /// from superclass is used.
  final String? ifPresent;

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    if (value != null) {
      if (ifPresent != null) {
        return ifPresent!;
      }
    } else {
      if (ifNull != null) {
        return ifNull!;
      }
    }
    return super.valueToString(parentConfiguration: parentConfiguration);
  }

  @override
  bool get showName {
    if ((value != null && ifPresent == null) || (value == null && ifNull == null)) {
      // We are missing a description for the flag value so we need to show the
      // flag name. The property will have DiagnosticLevel.hidden for this case
      // so users will not see this property in this case unless they are
      // displaying hidden properties.
      return true;
    }
    return super.showName;
  }

  @override
  DiagnosticLevel get level {
    if (value != null) {
      if (ifPresent == null) {
        return DiagnosticLevel.hidden;
      }
    } else {
      if (ifNull == null) {
        return DiagnosticLevel.hidden;
      }
    }

    return super.level;
  }

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final Map<String, Object?> json = super.toJsonMap(delegate);
    if (ifPresent != null) {
      json['ifPresent'] = ifPresent;
    }
    return json;
  }
}

/// A summary of multiple properties, indicating whether each of them is present
/// (non-null) or absent (null).
///
/// Each entry of [value] is described by its key. The eventual description will
/// be a list of keys of non-null entries.
///
/// The [ifEmpty] describes the entire collection of [value] when it contains no
/// non-null entries. If [ifEmpty] is omitted, [level] will be
/// [DiagnosticLevel.hidden] when [value] contains no non-null entries.
///
/// This kind of diagnostics property is typically used for opaque
/// values, like closures, where presenting the actual object is of dubious
/// value but where reporting the presence or absence of the value is much more
/// useful.
///
/// See also:
///
///  * [ObjectFlagProperty], which provides similar functionality but accepts
///    only one flag, and is preferred if there is only one entry.
///  * [IterableProperty], which provides similar functionality describing
///    the values a collection of objects.
class FlagsSummary<T> extends DiagnosticsProperty<Map<String, T?>> {
  /// Create a summary for multiple properties, indicating whether each of them
  /// is present (non-null) or absent (null).
  ///
  /// The [value], [showName], [showSeparator] and [level] arguments must not be
  /// null.
  FlagsSummary(
    String super.name,
    Map<String, T?> super.value, {
    super.ifEmpty,
    super.showName,
    super.showSeparator,
    super.level,
  });

  @override
  Map<String, T?> get value => super.value!;

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    if (!_hasNonNullEntry() && ifEmpty != null) {
      return ifEmpty!;
    }

    final Iterable<String> formattedValues = _formattedValues();
    if (parentConfiguration != null && !parentConfiguration.lineBreakProperties) {
      // Always display the value as a single line and enclose the iterable
      // value in brackets to avoid ambiguity.
      return '[${formattedValues.join(', ')}]';
    }

    return formattedValues.join(_isSingleLine(style) ? ', ' : '\n');
  }

  /// Priority level of the diagnostic used to control which diagnostics should
  /// be shown and filtered.
  ///
  /// If [ifEmpty] is null and the [value] contains no non-null entries, then
  /// level [DiagnosticLevel.hidden] is returned.
  @override
  DiagnosticLevel get level {
    if (!_hasNonNullEntry() && ifEmpty == null) {
      return DiagnosticLevel.hidden;
    }
    return super.level;
  }

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final Map<String, Object?> json = super.toJsonMap(delegate);
    if (value.isNotEmpty) {
      json['values'] = _formattedValues().toList();
    }
    return json;
  }

  bool _hasNonNullEntry() => value.values.any((T? o) => o != null);

  // An iterable of each entry's description in [value].
  //
  // For a non-null value, its description is its key.
  //
  // For a null value, it is omitted unless `includeEmpty` is true and
  // [ifEntryNull] contains a corresponding description.
  Iterable<String> _formattedValues() {
    return value.entries
        .where((MapEntry<String, T?> entry) => entry.value != null)
        .map((MapEntry<String, T?> entry) => entry.key);
  }
}

/// Signature for computing the value of a property.
///
/// May throw exception if accessing the property would throw an exception
/// and callers must handle that case gracefully. For example, accessing a
/// property may trigger an assert that layout constraints were violated.
typedef ComputePropertyValueCallback<T> = T? Function();

/// Property with a [value] of type [T].
///
/// If the default `value.toString()` does not provide an adequate description
/// of the value, specify `description` defining a custom description.
///
/// The [showSeparator] property indicates whether a separator should be placed
/// between the property [name] and its [value].
class DiagnosticsProperty<T> extends DiagnosticsNode {
  /// Create a diagnostics property.
  ///
  /// The [level] argument is just a suggestion and can be overridden if
  /// something else about the property causes it to have a lower or higher
  /// level. For example, if the property value is null and [missingIfNull] is
  /// true, [level] is raised to [DiagnosticLevel.warning].
  DiagnosticsProperty(
    String? name,
    T? value, {
    String? description,
    String? ifNull,
    this.ifEmpty,
    super.showName,
    super.showSeparator,
    this.defaultValue = kNoDefaultValue,
    this.tooltip,
    this.missingIfNull = false,
    super.linePrefix,
    this.expandableValue = false,
    this.allowWrap = true,
    this.allowNameWrap = true,
    DiagnosticsTreeStyle super.style = DiagnosticsTreeStyle.singleLine,
    DiagnosticLevel level = DiagnosticLevel.info,
  }) : _description = description,
       _valueComputed = true,
       _value = value,
       _computeValue = null,
       ifNull = ifNull ?? (missingIfNull ? 'MISSING' : null),
       _defaultLevel = level,
       super(name: name);

  /// Property with a [value] that is computed only when needed.
  ///
  /// Use if computing the property [value] may throw an exception or is
  /// expensive.
  ///
  /// The [level] argument is just a suggestion and can be overridden
  /// if something else about the property causes it to have a lower or higher
  /// level. For example, if calling `computeValue` throws an exception, [level]
  /// will always return [DiagnosticLevel.error].
  DiagnosticsProperty.lazy(
    String? name,
    ComputePropertyValueCallback<T> computeValue, {
    String? description,
    String? ifNull,
    this.ifEmpty,
    super.showName,
    super.showSeparator,
    this.defaultValue = kNoDefaultValue,
    this.tooltip,
    this.missingIfNull = false,
    this.expandableValue = false,
    this.allowWrap = true,
    this.allowNameWrap = true,
    DiagnosticsTreeStyle super.style = DiagnosticsTreeStyle.singleLine,
    DiagnosticLevel level = DiagnosticLevel.info,
  }) : assert(defaultValue == kNoDefaultValue || defaultValue is T?),
       _description = description,
       _valueComputed = false,
       _value = null,
       _computeValue = computeValue,
       _defaultLevel = level,
       ifNull = ifNull ?? (missingIfNull ? 'MISSING' : null),
       super(name: name);

  final String? _description;

  /// Whether to expose properties and children of the value as properties and
  /// children.
  final bool expandableValue;

  @override
  final bool allowWrap;

  @override
  final bool allowNameWrap;

  @override
  Map<String, Object?> toJsonMap(DiagnosticsSerializationDelegate delegate) {
    final T? v = value;
    List<Map<String, Object?>>? properties;
    if (delegate.expandPropertyValues &&
        delegate.includeProperties &&
        v is Diagnosticable &&
        getProperties().isEmpty) {
      // Exclude children for expanded nodes to avoid cycles.
      delegate = delegate.copyWith(subtreeDepth: 0, includeProperties: false);
      properties = DiagnosticsNode.toJsonList(
        delegate.filterProperties(v.toDiagnosticsNode().getProperties(), this),
        this,
        delegate,
      );
    }
    final Map<String, Object?> json = super.toJsonMap(delegate);
    if (properties != null) {
      json['properties'] = properties;
    }
    if (defaultValue != kNoDefaultValue) {
      json['defaultValue'] = defaultValue.toString();
    }
    if (ifEmpty != null) {
      json['ifEmpty'] = ifEmpty;
    }
    if (ifNull != null) {
      json['ifNull'] = ifNull;
    }
    if (tooltip != null) {
      json['tooltip'] = tooltip;
    }
    json['missingIfNull'] = missingIfNull;
    if (exception != null) {
      json['exception'] = exception.toString();
    }
    json['propertyType'] = propertyType.toString();
    json['defaultLevel'] = _defaultLevel.name;
    if (value is Diagnosticable || value is DiagnosticsNode) {
      json['isDiagnosticableValue'] = true;
    }
    if (v is num) {
      // TODO(jacob314): Workaround, since JSON.stringify replaces infinity and NaN with null,
      // https://github.com/flutter/flutter/issues/39937#issuecomment-529558033)
      json['value'] = v.isFinite ? v : v.toString();
    }
    if (value is String || value is bool || value == null) {
      json['value'] = value;
    }
    return json;
  }

  /// Returns a string representation of the property value.
  ///
  /// Subclasses should override this method instead of [toDescription] to
  /// customize how property values are converted to strings.
  ///
  /// Overriding this method ensures that behavior controlling how property
  /// values are decorated to generate a nice [toDescription] are consistent
  /// across all implementations. Debugging tools may also choose to use
  /// [valueToString] directly instead of [toDescription].
  ///
  /// `parentConfiguration` specifies how the parent is rendered as text art.
  /// For example, if the parent places all properties on one line, the value
  /// of the property should be displayed without line breaks if possible.
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    final T? v = value;
    // DiagnosticableTree values are shown using the shorter toStringShort()
    // instead of the longer toString() because the toString() for a
    // DiagnosticableTree value is likely too large to be useful.
    return v is DiagnosticableTree ? v.toStringShort() : v.toString();
  }

  @override
  String toDescription({TextTreeConfiguration? parentConfiguration}) {
    if (_description != null) {
      return _addTooltip(_description);
    }

    if (exception != null) {
      return 'EXCEPTION (${exception.runtimeType})';
    }

    if (ifNull != null && value == null) {
      return _addTooltip(ifNull!);
    }

    String result = valueToString(parentConfiguration: parentConfiguration);
    if (result.isEmpty && ifEmpty != null) {
      result = ifEmpty!;
    }
    return _addTooltip(result);
  }

  /// If a [tooltip] is specified, add the tooltip it to the end of `text`
  /// enclosing it parenthesis to disambiguate the tooltip from the rest of
  /// the text.
  String _addTooltip(String text) {
    return tooltip == null ? text : '$text ($tooltip)';
  }

  /// Description if the property [value] is null.
  final String? ifNull;

  /// Description if the property description would otherwise be empty.
  final String? ifEmpty;

  /// Optional tooltip typically describing the property.
  ///
  /// Example tooltip: 'physical pixels per logical pixel'
  ///
  /// If present, the tooltip is added in parenthesis after the raw value when
  /// generating the string description.
  final String? tooltip;

  /// Whether a [value] of null causes the property to have [level]
  /// [DiagnosticLevel.warning] warning that the property is missing a [value].
  final bool missingIfNull;

  /// The type of the property [value].
  ///
  /// This is determined from the type argument `T` used to instantiate the
  /// [DiagnosticsProperty] class. This means that the type is available even if
  /// [value] is null, but it also means that the [propertyType] is only as
  /// accurate as the type provided when invoking the constructor.
  ///
  /// Generally, this is only useful for diagnostic tools that should display
  /// null values in a manner consistent with the property type. For example, a
  /// tool might display a null [Color] value as an empty rectangle instead of
  /// the word "null".
  Type get propertyType => T;

  /// Returns the value of the property either from cache or by invoking a
  /// [ComputePropertyValueCallback].
  ///
  /// If an exception is thrown invoking the [ComputePropertyValueCallback],
  /// [value] returns null and the exception thrown can be found via the
  /// [exception] property.
  ///
  /// See also:
  ///
  ///  * [valueToString], which converts the property value to a string.
  @override
  T? get value {
    _maybeCacheValue();
    return _value;
  }

  T? _value;

  bool _valueComputed;

  Object? _exception;

  /// Exception thrown if accessing the property [value] threw an exception.
  ///
  /// Returns null if computing the property value did not throw an exception.
  Object? get exception {
    _maybeCacheValue();
    return _exception;
  }

  void _maybeCacheValue() {
    if (_valueComputed) {
      return;
    }

    _valueComputed = true;
    assert(_computeValue != null);
    try {
      _value = _computeValue!();
    } catch (exception) {
      // The error is reported to inspector; rethrowing would destroy the
      // debugging experience.
      _exception = exception;
      _value = null;
    }
  }

  /// The default value of this property, when it has not been set to a specific
  /// value.
  ///
  /// For most [DiagnosticsProperty] classes, if the [value] of the property
  /// equals [defaultValue], then the priority [level] of the property is
  /// downgraded to [DiagnosticLevel.fine] on the basis that the property value
  /// is uninteresting. This is implemented by [isInteresting].
  ///
  /// The [defaultValue] is [kNoDefaultValue] by default. Otherwise it must be of
  /// type `T?`.
  final Object? defaultValue;

  /// Whether to consider the property's value interesting. When a property is
  /// uninteresting, its [level] is downgraded to [DiagnosticLevel.fine]
  /// regardless of the value provided as the constructor's `level` argument.
  bool get isInteresting => defaultValue == kNoDefaultValue || value != defaultValue;

  final DiagnosticLevel _defaultLevel;

  /// Priority level of the diagnostic used to control which diagnostics should
  /// be shown and filtered.
  ///
  /// The property level defaults to the value specified by the [level]
  /// constructor argument. The level is raised to [DiagnosticLevel.error] if
  /// an [exception] was thrown getting the property [value]. The level is
  /// raised to [DiagnosticLevel.warning] if the property [value] is null and
  /// the property is not allowed to be null due to [missingIfNull]. The
  /// priority level is lowered to [DiagnosticLevel.fine] if the property
  /// [value] equals [defaultValue].
  @override
  DiagnosticLevel get level {
    if (_defaultLevel == DiagnosticLevel.hidden) {
      return _defaultLevel;
    }

    if (exception != null) {
      return DiagnosticLevel.error;
    }

    if (value == null && missingIfNull) {
      return DiagnosticLevel.warning;
    }

    if (!isInteresting) {
      return DiagnosticLevel.fine;
    }

    return _defaultLevel;
  }

  final ComputePropertyValueCallback<T>? _computeValue;

  @override
  List<DiagnosticsNode> getProperties() {
    if (expandableValue) {
      final T? object = value;
      if (object is DiagnosticsNode) {
        return object.getProperties();
      }
      if (object is Diagnosticable) {
        return object.toDiagnosticsNode(style: style).getProperties();
      }
    }
    return const <DiagnosticsNode>[];
  }

  @override
  List<DiagnosticsNode> getChildren() {
    if (expandableValue) {
      final T? object = value;
      if (object is DiagnosticsNode) {
        return object.getChildren();
      }
      if (object is Diagnosticable) {
        return object.toDiagnosticsNode(style: style).getChildren();
      }
    }
    return const <DiagnosticsNode>[];
  }
}

/// [DiagnosticsNode] that lazily calls the associated [Diagnosticable] [value]
/// to implement [getChildren] and [getProperties].
class DiagnosticableNode<T extends Diagnosticable> extends DiagnosticsNode {
  /// Create a diagnostics describing a [Diagnosticable] value.
  DiagnosticableNode({super.name, required this.value, required super.style});

  @override
  final T value;

  DiagnosticPropertiesBuilder? _cachedBuilder;

  /// Retrieve the [DiagnosticPropertiesBuilder] of current node.
  ///
  /// It will cache the result to prevent duplicate operation.
  DiagnosticPropertiesBuilder? get builder {
    if (kReleaseMode) {
      return null;
    } else {
      assert(() {
        if (_cachedBuilder == null) {
          _cachedBuilder = DiagnosticPropertiesBuilder();
          value.debugFillProperties(_cachedBuilder!);
        }
        return true;
      }());
      return _cachedBuilder;
    }
  }

  @override
  DiagnosticsTreeStyle get style {
    return kReleaseMode
        ? DiagnosticsTreeStyle.none
        : super.style ?? builder!.defaultDiagnosticsTreeStyle;
  }

  @override
  String? get emptyBodyDescription =>
      (kReleaseMode || kProfileMode) ? '' : builder!.emptyBodyDescription;

  @override
  List<DiagnosticsNode> getProperties() =>
      (kReleaseMode || kProfileMode) ? const <DiagnosticsNode>[] : builder!.properties;

  @override
  List<DiagnosticsNode> getChildren() {
    return const <DiagnosticsNode>[];
  }

  @override
  String toDescription({TextTreeConfiguration? parentConfiguration}) {
    String result = '';
    assert(() {
      result = value.toStringShort();
      return true;
    }());
    return result;
  }
}

/// [DiagnosticsNode] for an instance of [DiagnosticableTree].
class DiagnosticableTreeNode extends DiagnosticableNode<DiagnosticableTree> {
  /// Creates a [DiagnosticableTreeNode].
  DiagnosticableTreeNode({super.name, required super.value, required super.style});

  @override
  List<DiagnosticsNode> getChildren() => value.debugDescribeChildren();
}

/// Returns a 5 character long hexadecimal string generated from
/// [Object.hashCode]'s 20 least-significant bits.
String shortHash(Object? object) {
  return object.hashCode.toUnsigned(20).toRadixString(16).padLeft(5, '0');
}

/// Returns a summary of the runtime type and hash code of `object`.
///
/// See also:
///
///  * [Object.hashCode], a value used when placing an object in a [Map] or
///    other similar data structure, and which is also used in debug output to
///    distinguish instances of the same class (hash collisions are
///    possible, but rare enough that its use in debug output is useful).
///  * [Object.runtimeType], the [Type] of an object.
String describeIdentity(Object? object) =>
    '${objectRuntimeType(object, '<optimized out>')}#${shortHash(object)}';

/// Returns a short description of an enum value.
///
/// Strips off the enum class name from the `enumEntry.toString()`.
///
/// For real enums, this is redundant with calling the `name` getter on the enum
/// value (see [EnumName.name]), a feature that was added to Dart 2.15.
///
/// This function can also be used with classes whose `toString` return a value
/// in the same form as an enum (the class name, a dot, then the value name).
/// For example, it's used with [SemanticsAction], which is written to appear to
/// be an enum but is actually a bespoke class so that the index values can be
/// set as powers of two instead of as sequential integers.
///
/// {@tool snippet}
///
/// ```dart
/// enum Day {
///   monday, tuesday, wednesday, thursday, friday, saturday, sunday
/// }
///
/// void validateDescribeEnum() {
///   assert(Day.monday.toString() == 'Day.monday');
///   assert(describeEnum(Day.monday) == 'monday');
///   assert(Day.monday.name == 'monday'); // preferred for real enums
/// }
/// ```
/// {@end-tool}
@Deprecated(
  'Use the `name` getter on enums instead. '
  'This feature was deprecated after v3.14.0-2.0.pre.',
)
String describeEnum(Object enumEntry) {
  if (enumEntry is Enum) {
    return enumEntry.name;
  }
  final String description = enumEntry.toString();
  final int indexOfDot = description.indexOf('.');
  assert(
    indexOfDot != -1 && indexOfDot < description.length - 1,
    'The provided object "$enumEntry" is not an enum.',
  );
  return description.substring(indexOfDot + 1);
}

/// Builder to accumulate properties and configuration used to assemble a
/// [DiagnosticsNode] from a [Diagnosticable] object.
class DiagnosticPropertiesBuilder {
  /// Creates a [DiagnosticPropertiesBuilder] with [properties] initialize to
  /// an empty array.
  DiagnosticPropertiesBuilder() : properties = <DiagnosticsNode>[];

  /// Creates a [DiagnosticPropertiesBuilder] with a given [properties].
  DiagnosticPropertiesBuilder.fromProperties(this.properties);

  /// Add a property to the list of properties.
  void add(DiagnosticsNode property) {
    assert(() {
      properties.add(property);
      return true;
    }());
  }

  /// List of properties accumulated so far.
  final List<DiagnosticsNode> properties;

  /// Default style to use for the [DiagnosticsNode] if no style is specified.
  DiagnosticsTreeStyle defaultDiagnosticsTreeStyle = DiagnosticsTreeStyle.sparse;

  /// Description to show if the node has no displayed properties or children.
  String? emptyBodyDescription;
}

// Examples can assume:
// class ExampleSuperclass with Diagnosticable { late String message; late double stepWidth; late double scale; late double paintExtent; late double hitTestExtent; late double paintExtend; late double maxWidth; late bool primary; late double progress; late int maxLines; late Duration duration; late int depth; Iterable<BoxShadow>? boxShadow; late DiagnosticsTreeStyle style; late bool hasSize; late Matrix4 transform; Map<Listenable, VoidCallback>? handles; late Color color; late bool obscureText; late ImageRepeat repeat; late Size size; late Widget widget; late bool isCurrent; late bool keepAlive; late TextAlign textAlign; }

/// A mixin class for providing string and [DiagnosticsNode] debug
/// representations describing the properties of an object.
///
/// The string debug representation is generated from the intermediate
/// [DiagnosticsNode] representation. The [DiagnosticsNode] representation is
/// also used by debugging tools displaying interactive trees of objects and
/// properties.
///
/// See also:
///
///  * [debugFillProperties], which lists best practices for specifying the
///    properties of a [DiagnosticsNode]. The most common use case is to
///    override [debugFillProperties] defining custom properties for a subclass
///    of [DiagnosticableTreeMixin] using the existing [DiagnosticsProperty]
///    subclasses.
///  * [DiagnosticableTree], which extends this class to also describe the
///    children of a tree structured object.
///  * [DiagnosticableTree.debugDescribeChildren], which lists best practices
///    for describing the children of a [DiagnosticsNode]. Typically the base
///    class already describes the children of a node properly or a node has
///    no children.
///  * [DiagnosticsProperty], which should be used to create leaf diagnostic
///    nodes without properties or children. There are many
///    [DiagnosticsProperty] subclasses to handle common use cases.
mixin Diagnosticable {
  /// A brief description of this object, usually just the [runtimeType] and the
  /// [hashCode].
  ///
  /// See also:
  ///
  ///  * [toString], for a detailed description of the object.
  String toStringShort() => describeIdentity(this);

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    String? fullString;
    assert(() {
      fullString = toDiagnosticsNode(
        style: DiagnosticsTreeStyle.singleLine,
      ).toString(minLevel: minLevel);
      return true;
    }());
    return fullString ?? toStringShort();
  }

  /// Returns a debug representation of the object that is used by debugging
  /// tools and by [DiagnosticsNode.toStringDeep].
  ///
  /// Leave [name] as null if there is not a meaningful description of the
  /// relationship between the this node and its parent.
  ///
  /// Typically the [style] argument is only specified to indicate an atypical
  /// relationship between the parent and the node. For example, pass
  /// [DiagnosticsTreeStyle.offstage] to indicate that a node is offstage.
  DiagnosticsNode toDiagnosticsNode({String? name, DiagnosticsTreeStyle? style}) {
    return DiagnosticableNode<Diagnosticable>(name: name, value: this, style: style);
  }

  /// Add additional properties associated with the node.
  ///
  /// {@youtube 560 315 https://www.youtube.com/watch?v=DnC7eT-vh1k}
  ///
  /// Use the most specific [DiagnosticsProperty] existing subclass to describe
  /// each property instead of the [DiagnosticsProperty] base class. There are
  /// only a small number of [DiagnosticsProperty] subclasses each covering a
  /// common use case. Consider what values a property is relevant for users
  /// debugging as users debugging large trees are overloaded with information.
  /// Common named parameters in [DiagnosticsNode] subclasses help filter when
  /// and how properties are displayed.
  ///
  /// `defaultValue`, `showName`, `showSeparator`, and `level` keep string
  /// representations of diagnostics terse and hide properties when they are not
  /// very useful.
  ///
  ///  * Use `defaultValue` any time the default value of a property is
  ///    uninteresting. For example, specify a default value of null any time
  ///    a property being null does not indicate an error.
  ///  * Avoid specifying the `level` parameter unless the result you want
  ///    cannot be achieved by using the `defaultValue` parameter or using
  ///    the [ObjectFlagProperty] class to conditionally display the property
  ///    as a flag.
  ///  * Specify `showName` and `showSeparator` in rare cases where the string
  ///    output would look clumsy if they were not set.
  ///    ```dart
  ///    DiagnosticsProperty<Object>('child(3, 4)', null, ifNull: 'is null', showSeparator: false).toString()
  ///    ```
  ///    Shows using `showSeparator` to get output `child(3, 4) is null` which
  ///    is more polished than `child(3, 4): is null`.
  ///    ```dart
  ///    DiagnosticsProperty<IconData>('icon', icon, ifNull: '<empty>', showName: false).toString()
  ///    ```
  ///    Shows using `showName` to omit the property name as in this context the
  ///    property name does not add useful information.
  ///
  /// `ifNull`, `ifEmpty`, `unit`, and `tooltip` make property
  /// descriptions clearer. The examples in the code sample below illustrate
  /// good uses of all of these parameters.
  ///
  /// ## DiagnosticsProperty subclasses for primitive types
  ///
  ///  * [StringProperty], which supports automatically enclosing a [String]
  ///    value in quotes.
  ///  * [DoubleProperty], which supports specifying a unit of measurement for
  ///    a [double] value.
  ///  * [PercentProperty], which clamps a [double] to between 0 and 1 and
  ///    formats it as a percentage.
  ///  * [IntProperty], which supports specifying a unit of measurement for an
  ///    [int] value.
  ///  * [FlagProperty], which formats a [bool] value as one or more flags.
  ///    Depending on the use case it is better to format a bool as
  ///    `DiagnosticsProperty<bool>` instead of using [FlagProperty] as the
  ///    output is more verbose but unambiguous.
  ///
  /// ## Other important [DiagnosticsProperty] variants
  ///
  ///  * [EnumProperty], which provides terse descriptions of enum values
  ///    working around limitations of the `toString` implementation for Dart
  ///    enum types.
  ///  * [IterableProperty], which handles iterable values with display
  ///    customizable depending on the [DiagnosticsTreeStyle] used.
  ///  * [ObjectFlagProperty], which provides terse descriptions of whether a
  ///    property value is present or not. For example, whether an `onClick`
  ///    callback is specified or an animation is in progress.
  ///  * [ColorProperty], which must be used if the property value is
  ///    a [Color] or one of its subclasses.
  ///  * [IconDataProperty], which must be used if the property value
  ///    is of type [IconData].
  ///
  /// If none of these subclasses apply, use the [DiagnosticsProperty]
  /// constructor or in rare cases create your own [DiagnosticsProperty]
  /// subclass as in the case for [TransformProperty] which handles [Matrix4]
  /// that represent transforms. Generally any property value with a good
  /// `toString` method implementation works fine using [DiagnosticsProperty]
  /// directly.
  ///
  /// {@tool snippet}
  ///
  /// This example shows best practices for implementing [debugFillProperties]
  /// illustrating use of all common [DiagnosticsProperty] subclasses and all
  /// common [DiagnosticsProperty] parameters.
  ///
  /// ```dart
  /// class ExampleObject extends ExampleSuperclass {
  ///
  ///   // ...various members and properties...
  ///
  ///   @override
  ///   void debugFillProperties(DiagnosticPropertiesBuilder properties) {
  ///     // Always add properties from the base class first.
  ///     super.debugFillProperties(properties);
  ///
  ///     // Omit the property name 'message' when displaying this String property
  ///     // as it would just add visual noise.
  ///     properties.add(StringProperty('message', message, showName: false));
  ///
  ///     properties.add(DoubleProperty('stepWidth', stepWidth));
  ///
  ///     // A scale of 1.0 does nothing so should be hidden.
  ///     properties.add(DoubleProperty('scale', scale, defaultValue: 1.0));
  ///
  ///     // If the hitTestExtent matches the paintExtent, it is just set to its
  ///     // default value so is not relevant.
  ///     properties.add(DoubleProperty('hitTestExtent', hitTestExtent, defaultValue: paintExtent));
  ///
  ///     // maxWidth of double.infinity indicates the width is unconstrained and
  ///     // so maxWidth has no impact.
  ///     properties.add(DoubleProperty('maxWidth', maxWidth, defaultValue: double.infinity));
  ///
  ///     // Progress is a value between 0 and 1 or null. Showing it as a
  ///     // percentage makes the meaning clear enough that the name can be
  ///     // hidden.
  ///     properties.add(PercentProperty(
  ///       'progress',
  ///       progress,
  ///       showName: false,
  ///       ifNull: '<indeterminate>',
  ///     ));
  ///
  ///     // Most text fields have maxLines set to 1.
  ///     properties.add(IntProperty('maxLines', maxLines, defaultValue: 1));
  ///
  ///     // Specify the unit as otherwise it would be unclear that time is in
  ///     // milliseconds.
  ///     properties.add(IntProperty('duration', duration.inMilliseconds, unit: 'ms'));
  ///
  ///     // Tooltip is used instead of unit for this case as a unit should be a
  ///     // terse description appropriate to display directly after a number
  ///     // without a space.
  ///     properties.add(DoubleProperty(
  ///       'device pixel ratio',
  ///       devicePixelRatio,
  ///       tooltip: 'physical pixels per logical pixel',
  ///     ));
  ///
  ///     // Displaying the depth value would be distracting. Instead only display
  ///     // if the depth value is missing.
  ///     properties.add(ObjectFlagProperty<int>('depth', depth, ifNull: 'no depth'));
  ///
  ///     // bool flag that is only shown when the value is true.
  ///     properties.add(FlagProperty('using primary controller', value: primary));
  ///
  ///     properties.add(FlagProperty(
  ///       'isCurrent',
  ///       value: isCurrent,
  ///       ifTrue: 'active',
  ///       ifFalse: 'inactive',
  ///     ));
  ///
  ///     properties.add(DiagnosticsProperty<bool>('keepAlive', keepAlive));
  ///
  ///     // FlagProperty could have also been used in this case.
  ///     // This option results in the text "obscureText: true" instead
  ///     // of "obscureText" which is a bit more verbose but a bit clearer.
  ///     properties.add(DiagnosticsProperty<bool>('obscureText', obscureText, defaultValue: false));
  ///
  ///     properties.add(EnumProperty<TextAlign>('textAlign', textAlign, defaultValue: null));
  ///     properties.add(EnumProperty<ImageRepeat>('repeat', repeat, defaultValue: ImageRepeat.noRepeat));
  ///
  ///     // Warn users when the widget is missing but do not show the value.
  ///     properties.add(ObjectFlagProperty<Widget>('widget', widget, ifNull: 'no widget'));
  ///
  ///     properties.add(IterableProperty<BoxShadow>(
  ///       'boxShadow',
  ///       boxShadow,
  ///       defaultValue: null,
  ///       style: style,
  ///     ));
  ///
  ///     // Getting the value of size throws an exception unless hasSize is true.
  ///     properties.add(DiagnosticsProperty<Size>.lazy(
  ///       'size',
  ///       () => size,
  ///       description: '${ hasSize ? size : "MISSING" }',
  ///     ));
  ///
  ///     // If the `toString` method for the property value does not provide a
  ///     // good terse description, write a DiagnosticsProperty subclass as in
  ///     // the case of TransformProperty which displays a nice debugging view
  ///     // of a Matrix4 that represents a transform.
  ///     properties.add(TransformProperty('transform', transform));
  ///
  ///     // If the value class has a good `toString` method, use
  ///     // DiagnosticsProperty<YourValueType>. Specifying the value type ensures
  ///     // that debugging tools always know the type of the field and so can
  ///     // provide the right UI affordances. For example, in this case even
  ///     // if color is null, a debugging tool still knows the value is a Color
  ///     // and can display relevant color related UI.
  ///     properties.add(DiagnosticsProperty<Color>('color', color));
  ///
  ///     // Use a custom description to generate a more terse summary than the
  ///     // `toString` method on the map class.
  ///     properties.add(DiagnosticsProperty<Map<Listenable, VoidCallback>>(
  ///       'handles',
  ///       handles,
  ///       description: handles != null
  ///         ? '${handles!.length} active client${ handles!.length == 1 ? "" : "s" }'
  ///         : null,
  ///       ifNull: 'no notifications ever received',
  ///       showName: false,
  ///     ));
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// Used by [toDiagnosticsNode] and [toString].
  ///
  /// Do not add values that have lifetime shorter than the object.
  @protected
  @mustCallSuper
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {}
}

/// A base class for providing string and [DiagnosticsNode] debug
/// representations describing the properties and children of an object.
///
/// The string debug representation is generated from the intermediate
/// [DiagnosticsNode] representation. The [DiagnosticsNode] representation is
/// also used by debugging tools displaying interactive trees of objects and
/// properties.
///
/// See also:
///
///  * [DiagnosticableTreeMixin], a mixin that implements this class.
///  * [Diagnosticable], which should be used instead of this class to
///    provide diagnostics for objects without children.
abstract class DiagnosticableTree with Diagnosticable {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const DiagnosticableTree();

  /// Returns a one-line detailed description of the object.
  ///
  /// This description is often somewhat long. This includes the same
  /// information given by [toStringDeep], but does not recurse to any children.
  ///
  /// `joiner` specifies the string which is place between each part obtained
  /// from [debugFillProperties]. Passing a string such as `'\n '` will result
  /// in a multiline string that indents the properties of the object below its
  /// name (as per [toString]).
  ///
  /// `minLevel` specifies the minimum [DiagnosticLevel] for properties included
  /// in the output.
  ///
  /// See also:
  ///
  ///  * [toString], for a brief description of the object.
  ///  * [toStringDeep], for a description of the subtree rooted at this object.
  String toStringShallow({String joiner = ', ', DiagnosticLevel minLevel = DiagnosticLevel.debug}) {
    String? shallowString;
    assert(() {
      final StringBuffer result = StringBuffer();
      result.write(toString());
      result.write(joiner);
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      debugFillProperties(builder);
      result.write(
        builder.properties.where((DiagnosticsNode n) => !n.isFiltered(minLevel)).join(joiner),
      );
      shallowString = result.toString();
      return true;
    }());
    return shallowString ?? toString();
  }

  /// Returns a string representation of this node and its descendants.
  ///
  /// `prefixLineOne` will be added to the front of the first line of the
  /// output. `prefixOtherLines` will be added to the front of each other line.
  /// If `prefixOtherLines` is null, the `prefixLineOne` is used for every line.
  /// By default, there is no prefix.
  ///
  /// `minLevel` specifies the minimum [DiagnosticLevel] for properties included
  /// in the output.
  ///
  /// `wrapWidth` specifies the column number where word wrapping will be
  /// applied.
  ///
  /// The [toStringDeep] method takes other arguments, but those are intended
  /// for internal use when recursing to the descendants, and so can be ignored.
  ///
  /// See also:
  ///
  ///  * [toString], for a brief description of the object but not its children.
  ///  * [toStringShallow], for a detailed description of the object but not its
  ///    children.
  String toStringDeep({
    String prefixLineOne = '',
    String? prefixOtherLines,
    DiagnosticLevel minLevel = DiagnosticLevel.debug,
    int wrapWidth = 65,
  }) {
    return toDiagnosticsNode().toStringDeep(
      prefixLineOne: prefixLineOne,
      prefixOtherLines: prefixOtherLines,
      minLevel: minLevel,
      wrapWidth: wrapWidth,
    );
  }

  @override
  String toStringShort() => describeIdentity(this);

  @override
  DiagnosticsNode toDiagnosticsNode({String? name, DiagnosticsTreeStyle? style}) {
    return DiagnosticableTreeNode(name: name, value: this, style: style);
  }

  /// Returns a list of [DiagnosticsNode] objects describing this node's
  /// children.
  ///
  /// Children that are offstage should be added with `style` set to
  /// [DiagnosticsTreeStyle.offstage] to indicate that they are offstage.
  ///
  /// The list must not contain any null entries. If there are explicit null
  /// children to report, consider [DiagnosticsNode.message] or
  /// [DiagnosticsProperty<Object>] as possible [DiagnosticsNode] objects to
  /// provide.
  ///
  /// Used by [toStringDeep], [toDiagnosticsNode] and [toStringShallow].
  ///
  /// See also:
  ///
  ///  * [RenderTable.debugDescribeChildren], which provides high quality custom
  ///    descriptions for its child nodes.
  @protected
  List<DiagnosticsNode> debugDescribeChildren() => const <DiagnosticsNode>[];
}

/// A mixin that helps dump string and [DiagnosticsNode] representations of trees.
///
/// This mixin is identical to class [DiagnosticableTree].
mixin DiagnosticableTreeMixin implements DiagnosticableTree {
  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    return toDiagnosticsNode(style: DiagnosticsTreeStyle.singleLine).toString(minLevel: minLevel);
  }

  @override
  String toStringShallow({String joiner = ', ', DiagnosticLevel minLevel = DiagnosticLevel.debug}) {
    String? shallowString;
    assert(() {
      final StringBuffer result = StringBuffer();
      result.write(toStringShort());
      result.write(joiner);
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      debugFillProperties(builder);
      result.write(
        builder.properties.where((DiagnosticsNode n) => !n.isFiltered(minLevel)).join(joiner),
      );
      shallowString = result.toString();
      return true;
    }());
    return shallowString ?? toString();
  }

  @override
  String toStringDeep({
    String prefixLineOne = '',
    String? prefixOtherLines,
    DiagnosticLevel minLevel = DiagnosticLevel.debug,
    int wrapWidth = 65,
  }) {
    return toDiagnosticsNode().toStringDeep(
      prefixLineOne: prefixLineOne,
      prefixOtherLines: prefixOtherLines,
      minLevel: minLevel,
      wrapWidth: wrapWidth,
    );
  }

  @override
  String toStringShort() => describeIdentity(this);

  @override
  DiagnosticsNode toDiagnosticsNode({String? name, DiagnosticsTreeStyle? style}) {
    return DiagnosticableTreeNode(name: name, value: this, style: style);
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() => const <DiagnosticsNode>[];

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {}
}

/// [DiagnosticsNode] that exists mainly to provide a container for other
/// diagnostics that typically lacks a meaningful value of its own.
///
/// This class is typically used for displaying complex nested error messages.
class DiagnosticsBlock extends DiagnosticsNode {
  /// Creates a diagnostic with properties specified by [properties] and
  /// children specified by [children].
  DiagnosticsBlock({
    super.name,
    DiagnosticsTreeStyle super.style = DiagnosticsTreeStyle.whitespace,
    bool showName = true,
    super.showSeparator,
    super.linePrefix,
    this.value,
    String? description,
    this.level = DiagnosticLevel.info,
    this.allowTruncate = false,
    List<DiagnosticsNode> children = const <DiagnosticsNode>[],
    List<DiagnosticsNode> properties = const <DiagnosticsNode>[],
  }) : _description = description ?? '',
       _children = children,
       _properties = properties,
       super(showName: showName && name != null);

  final List<DiagnosticsNode> _children;
  final List<DiagnosticsNode> _properties;

  @override
  final DiagnosticLevel level;

  final String _description;

  @override
  final Object? value;

  @override
  final bool allowTruncate;

  @override
  List<DiagnosticsNode> getChildren() => _children;

  @override
  List<DiagnosticsNode> getProperties() => _properties;

  @override
  String toDescription({TextTreeConfiguration? parentConfiguration}) => _description;
}

/// A delegate that configures how a hierarchy of [DiagnosticsNode]s should be
/// serialized.
///
/// Implement this class in a subclass to fully configure how [DiagnosticsNode]s
/// get serialized.
abstract class DiagnosticsSerializationDelegate {
  /// Creates a simple [DiagnosticsSerializationDelegate] that controls the
  /// [subtreeDepth] and whether to [includeProperties].
  ///
  /// For additional configuration options, extend
  /// [DiagnosticsSerializationDelegate] and provide custom implementations
  /// for the methods of this class.
  const factory DiagnosticsSerializationDelegate({int subtreeDepth, bool includeProperties}) =
      _DefaultDiagnosticsSerializationDelegate;

  /// Returns a serializable map of additional information that will be included
  /// in the serialization of the given [DiagnosticsNode].
  ///
  /// This method is called for every [DiagnosticsNode] that's included in
  /// the serialization.
  Map<String, Object?> additionalNodeProperties(DiagnosticsNode node, {bool fullDetails = true});

  /// Filters the list of [DiagnosticsNode]s that will be included as children
  /// for the given `owner` node.
  ///
  /// The callback may return a subset of the children in the provided list
  /// or replace the entire list with new child nodes.
  ///
  /// See also:
  ///
  ///  * [subtreeDepth], which controls how many levels of children will be
  ///    included in the serialization.
  List<DiagnosticsNode> filterChildren(List<DiagnosticsNode> nodes, DiagnosticsNode owner);

  /// Filters the list of [DiagnosticsNode]s that will be included as properties
  /// for the given `owner` node.
  ///
  /// The callback may return a subset of the properties in the provided list
  /// or replace the entire list with new property nodes.
  ///
  /// By default, `nodes` is returned as-is.
  ///
  /// See also:
  ///
  ///  * [includeProperties], which controls whether properties will be included
  ///    at all.
  List<DiagnosticsNode> filterProperties(List<DiagnosticsNode> nodes, DiagnosticsNode owner);

  /// Truncates the given list of [DiagnosticsNode] that will be added to the
  /// serialization as children or properties of the `owner` node.
  ///
  /// The method must return a subset of the provided nodes and may
  /// not replace any nodes. While [filterProperties] and [filterChildren]
  /// completely hide a node from the serialization, truncating a node will
  /// leave a hint in the serialization that there were additional nodes in the
  /// result that are not included in the current serialization.
  ///
  /// By default, `nodes` is returned as-is.
  List<DiagnosticsNode> truncateNodesList(List<DiagnosticsNode> nodes, DiagnosticsNode? owner);

  /// Returns the [DiagnosticsSerializationDelegate] to be used
  /// for adding the provided [DiagnosticsNode] to the serialization.
  ///
  /// By default, this will return a copy of this delegate, which has the
  /// [subtreeDepth] reduced by one.
  ///
  /// This is called for nodes that will be added to the serialization as
  /// property or child of another node. It may return the same delegate if no
  /// changes to it are necessary.
  DiagnosticsSerializationDelegate delegateForNode(DiagnosticsNode node);

  /// Controls how many levels of children will be included in the serialized
  /// hierarchy of [DiagnosticsNode]s.
  ///
  /// Defaults to zero.
  ///
  /// See also:
  ///
  ///  * [filterChildren], which provides a way to filter the children that
  ///    will be included.
  int get subtreeDepth;

  /// Whether to include the properties of a [DiagnosticsNode] in the
  /// serialization.
  ///
  /// Defaults to false.
  ///
  /// See also:
  ///
  ///  * [filterProperties], which provides a way to filter the properties that
  ///    will be included.
  bool get includeProperties;

  /// Whether properties that have a [Diagnosticable] as value should be
  /// expanded.
  bool get expandPropertyValues;

  /// Creates a copy of this [DiagnosticsSerializationDelegate] with the
  /// provided values.
  DiagnosticsSerializationDelegate copyWith({int subtreeDepth, bool includeProperties});
}

class _DefaultDiagnosticsSerializationDelegate implements DiagnosticsSerializationDelegate {
  const _DefaultDiagnosticsSerializationDelegate({
    this.includeProperties = false,
    this.subtreeDepth = 0,
  });

  @override
  Map<String, Object?> additionalNodeProperties(DiagnosticsNode node, {bool fullDetails = true}) {
    return const <String, Object?>{};
  }

  @override
  DiagnosticsSerializationDelegate delegateForNode(DiagnosticsNode node) {
    return subtreeDepth > 0 ? copyWith(subtreeDepth: subtreeDepth - 1) : this;
  }

  @override
  bool get expandPropertyValues => false;

  @override
  List<DiagnosticsNode> filterChildren(List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
    return nodes;
  }

  @override
  List<DiagnosticsNode> filterProperties(List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
    return nodes;
  }

  @override
  final bool includeProperties;

  @override
  final int subtreeDepth;

  @override
  List<DiagnosticsNode> truncateNodesList(List<DiagnosticsNode> nodes, DiagnosticsNode? owner) {
    return nodes;
  }

  @override
  DiagnosticsSerializationDelegate copyWith({int? subtreeDepth, bool? includeProperties}) {
    return _DefaultDiagnosticsSerializationDelegate(
      subtreeDepth: subtreeDepth ?? this.subtreeDepth,
      includeProperties: includeProperties ?? this.includeProperties,
    );
  }
}
