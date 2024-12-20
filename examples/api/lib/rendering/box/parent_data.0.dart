// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

void main() => runApp(const SampleApp());

class SampleApp extends StatefulWidget {
  const SampleApp({super.key});

  @override
  State<SampleApp> createState() => _SampleAppState();
}

class _SampleAppState extends State<SampleApp> {
  // This can be toggled using buttons in the UI to change which layout render object is used.
  bool _compact = false;

  // This is the content we show in the rendering.
  //
  // Headline and Paragraph are simple custom widgets defined below.
  //
  // Any widget _could_ be specified here, and would render fine.
  // The Headline and Paragraph widgets are used so that the renderer
  // can distinguish between the kinds of content and use different
  // spacing between different children.
  static const List<Widget> body = <Widget>[
    Headline('Bugs that improve T for future bugs'),
    Paragraph(
      'The best bugs to fix are those that make us more productive '
      'in the future. Reducing test flakiness, reducing technical '
      'debt, increasing the number of team members who are able to '
      'review code confidently and well: this all makes future bugs '
      'easier to fix, which is a huge multiplier to our overall '
      'effectiveness and thus to developer happiness.',
    ),
    Headline('Bugs affecting more people are more valuable (maximize N)'),
    Paragraph('We will make more people happier if we fix a bug experienced by more people.'),
    Paragraph(
      'One thing to be careful about is to think about the number of '
      'people we are ignoring in our metrics. For example, if we had '
      'a bug that prevented our product from working on Windows, we '
      'would have no Windows users, so the bug would affect nobody. '
      'However, fixing the bug would enable millions of developers '
      "to use our product, and that's the number that counts.",
    ),
    Headline('Bugs with greater impact on developers are more valuable (maximize ΔH)'),
    Paragraph(
      'A slight improvement to the user experience is less valuable '
      'than a greater improvement. For example, if our application, '
      'under certain conditions, shows a message with a typo, and '
      'then crashes because of an off-by-one error in the code, '
      'fixing the crash is a higher priority than fixing the typo.',
    ),
  ];

  // This is the description of the demo's interface.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text('Custom Render Boxes'),
          // There are two buttons over to the top right of the demo that let you
          // toggle between the two rendering modes.
          actions: <Widget>[
            IconButton(
              icon: const Icon(Icons.density_small),
              isSelected: _compact,
              onPressed: () {
                setState(() {
                  _compact = true;
                });
              },
            ),
            IconButton(
              icon: const Icon(Icons.density_large),
              isSelected: !_compact,
              onPressed: () {
                setState(() {
                  _compact = false;
                });
              },
            ),
          ],
        ),
        body: SingleChildScrollView(
          padding: const EdgeInsets.symmetric(horizontal: 30.0, vertical: 20.0),
          // CompactLayout and OpenLayout are the two rendering widgets defined below.
          child: _compact ? const CompactLayout(children: body) : const OpenLayout(children: body),
        ),
      ),
    );
  }
}

// Headline and Paragraph are just wrappers around the Text widget, but they
// also introduce a TextCategory widget that the CompactLayout and OpenLayout
// widgets can read to determine what kind of child is being rendered.

class Headline extends StatelessWidget {
  const Headline(this.text, {super.key});

  final String text;

  @override
  Widget build(BuildContext context) {
    return TextCategory(
      category: 'headline',
      child: Text(text, style: Theme.of(context).textTheme.titleLarge),
    );
  }
}

class Paragraph extends StatelessWidget {
  const Paragraph(this.text, {super.key});

  final String text;

  @override
  Widget build(BuildContext context) {
    return TextCategory(
      category: 'paragraph',
      child: Text(text, style: Theme.of(context).textTheme.bodyLarge),
    );
  }
}

// This is the ParentDataWidget that allows us to specify what kind of child
// is being rendered. It allows information to be shared with the render object
// without violating the principle of agnostic composition (wherein parents should
// work with any child, not only support a fixed set of children).
class TextCategory extends ParentDataWidget<TextFlowParentData> {
  const TextCategory({super.key, required this.category, required super.child});

  final String category;

  @override
  void applyParentData(RenderObject renderObject) {
    final TextFlowParentData parentData = renderObject.parentData! as TextFlowParentData;
    if (parentData.category != category) {
      parentData.category = category;
      renderObject.parent!.markNeedsLayout();
    }
  }

  @override
  Type get debugTypicalAncestorWidgetClass => OpenLayout;
}

// This is one of the two layout variants. It is a widget that defers to
// a render object defined below (RenderCompactLayout).
class CompactLayout extends MultiChildRenderObjectWidget {
  const CompactLayout({super.key, super.children});

  @override
  RenderCompactLayout createRenderObject(BuildContext context) {
    return RenderCompactLayout();
  }

  @override
  void updateRenderObject(BuildContext context, RenderCompactLayout renderObject) {
    // nothing to update
  }
}

// This is the other of the two layout variants. It is a widget that defers to a
// render object defined below (RenderOpenLayout).
class OpenLayout extends MultiChildRenderObjectWidget {
  const OpenLayout({super.key, super.children});

  @override
  RenderOpenLayout createRenderObject(BuildContext context) {
    return RenderOpenLayout();
  }

  @override
  void updateRenderObject(BuildContext context, RenderOpenLayout renderObject) {
    // nothing to update
  }
}

// This is the data structure that contains the kind of data that can be
// passed to the parent to label the child. It is literally stored on
// the RenderObject child, in its "parentData" field.
class TextFlowParentData extends ContainerBoxParentData<RenderBox> {
  String category = '';
}

// This is the bulk of the layout logic. (It's similar to RenderListBody,
// but only supports vertical layout.) It has no properties.
//
// This is an abstract class that is then extended by RenderCompactLayout and
// RenderOpenLayout to get different layouts based on the children's categories,
// as stored in the ParentData structure defined above.
//
// The documentation for the RenderBox class and its members provides much
// more detail on how to implement each of the methods below.
abstract class RenderTextFlow extends RenderBox
    with
        ContainerRenderObjectMixin<RenderBox, TextFlowParentData>,
        RenderBoxContainerDefaultsMixin<RenderBox, TextFlowParentData> {
  RenderTextFlow({List<RenderBox>? children}) {
    addAll(children);
  }

  @override
  void setupParentData(RenderBox child) {
    if (child.parentData is! TextFlowParentData) {
      child.parentData = TextFlowParentData();
    }
  }

  // This is the function that is overridden by the subclasses to do the
  // actual decision about the space to use between children.
  double spacingBetween(String before, String after);

  // The next few functions are the layout functions. In each case we walk the
  // children, calling each one to determine the geometry of the child, and use
  // that to determine the layout.

  // The first two functions compute the intrinsic width of the render object,
  // as seen when using the IntrinsicWidth widget.
  //
  // They essentially defer to the widest child.

  @override
  double computeMinIntrinsicWidth(double height) {
    double width = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final double childWidth = child.getMinIntrinsicWidth(height);
      if (childWidth > width) {
        width = childWidth;
      }
      child = childAfter(child);
    }
    return width;
  }

  @override
  double computeMaxIntrinsicWidth(double height) {
    double width = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final double childWidth = child.getMaxIntrinsicWidth(height);
      if (childWidth > width) {
        width = childWidth;
      }
      child = childAfter(child);
    }
    return width;
  }

  // The next two functions compute the intrinsic height of the render object,
  // as seen when using the IntrinsicHeight widget.
  //
  // They add up the height contributed by each child.
  //
  // They have to take into account the categories of the children and the
  // spacing that will be added, hence the slightly more elaborate logic.

  @override
  double computeMinIntrinsicHeight(double width) {
    String? previousCategory;
    double height = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final String category = (child.parentData! as TextFlowParentData).category;
      if (previousCategory != null) {
        height += spacingBetween(previousCategory, category);
      }
      height += child.getMinIntrinsicHeight(width);
      previousCategory = category;
      child = childAfter(child);
    }
    return height;
  }

  @override
  double computeMaxIntrinsicHeight(double width) {
    String? previousCategory;
    double height = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final String category = (child.parentData! as TextFlowParentData).category;
      if (previousCategory != null) {
        height += spacingBetween(previousCategory, category);
      }
      height += child.getMaxIntrinsicHeight(width);
      previousCategory = category;
      child = childAfter(child);
    }
    return height;
  }

  // This function implements the baseline logic. Because this class does
  // nothing special, we just defer to the default implementation in the
  // RenderBoxContainerDefaultsMixin utility class.

  @override
  double? computeDistanceToActualBaseline(TextBaseline baseline) {
    return defaultComputeDistanceToFirstActualBaseline(baseline);
  }

  // Next we have a function similar to the intrinsic methods, but for both axes
  // at the same time.

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    final BoxConstraints innerConstraints = BoxConstraints.tightFor(width: constraints.maxWidth);
    String? previousCategory;
    double y = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final String category = (child.parentData! as TextFlowParentData).category;
      if (previousCategory != null) {
        y += spacingBetween(previousCategory, category);
      }
      final Size childSize = child.getDryLayout(innerConstraints);
      y += childSize.height;
      previousCategory = category;
      child = childAfter(child);
    }
    return constraints.constrain(Size(constraints.maxWidth, y));
  }

  // This is the core of the layout logic. Most of the time, this is the only
  // function that will be called. It computes the size and position of each
  // child, and stores it (in the parent data, as it happens!) for use during
  // the paint phase.

  @override
  void performLayout() {
    final BoxConstraints innerConstraints = BoxConstraints.tightFor(width: constraints.maxWidth);
    String? previousCategory;
    double y = 0.0;
    RenderBox? child = firstChild;
    while (child != null) {
      final String category = (child.parentData! as TextFlowParentData).category;
      if (previousCategory != null) {
        // This is where we call the function that computes the spacing between
        // the different children. The arguments are the categories, obtained
        // from the parentData property of each child.
        y += spacingBetween(previousCategory, category);
      }
      child.layout(innerConstraints, parentUsesSize: true);
      (child.parentData! as TextFlowParentData).offset = Offset(0.0, y);
      y += child.size.height;
      previousCategory = category;
      child = childAfter(child);
    }
    size = constraints.constrain(Size(constraints.maxWidth, y));
  }

  // Hit testing is normal for this widget, so we defer to the default implementation.
  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    return defaultHitTestChildren(result, position: position);
  }

  // Painting is normal for this widget, so we defer to the default
  // implementation. The default implementation expects to find the positions
  // configured in the parentData property of each child, which is why we
  // configure it that way in performLayout above.
  @override
  void paint(PaintingContext context, Offset offset) {
    defaultPaint(context, offset);
  }
}

// Finally we have the two render objects that implement the two layouts in this demo.

class RenderOpenLayout extends RenderTextFlow {
  @override
  double spacingBetween(String before, String after) {
    if (after == 'headline') {
      return 20.0;
    }
    if (before == 'headline') {
      return 5.0;
    }
    return 10.0;
  }
}

class RenderCompactLayout extends RenderTextFlow {
  @override
  double spacingBetween(String before, String after) {
    if (after == 'headline') {
      return 4.0;
    }
    return 2.0;
  }
}
