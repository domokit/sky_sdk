// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  // Generic reference variables.
  BuildContext context;
  RenderObjectWidget renderObjectWidget;
  RenderObject renderObject;
  Object object;

  // Changes made in https://github.com/flutter/flutter/pull/26259
  Scaffold scaffold = Scaffold(resizeToAvoidBottomPadding: true);
  scaffold = Scaffold(error: '');
  final bool resize = scaffold.resizeToAvoidBottomPadding;

  // Change made in https://github.com/flutter/flutter/pull/15303
  showDialog(child: Text('Fix me.'));
  showDialog(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/44189
  const Element element = Element(myWidget);
  element.inheritFromElement(ancestor);
  element.inheritFromWidgetOfExactType(targetType);
  element.ancestorInheritedElementForWidgetOfExactType(targetType);
  element.ancestorWidgetOfExactType(targetType);
  element.ancestorStateOfType(TypeMatcher<targetType>());
  element.rootAncestorStateOfType(TypeMatcher<targetType>());
  element.ancestorRenderObjectOfType(TypeMatcher<targetType>());

  // Changes made in https://github.com/flutter/flutter/pull/45941 and https://github.com/flutter/flutter/pull/83843
  final WidgetsBinding binding = WidgetsBinding.instance!;
  binding.deferFirstFrameReport();
  binding.allowFirstFrameReport();

  // Changes made in https://github.com/flutter/flutter/pull/44189
  const StatefulElement statefulElement = StatefulElement(myWidget);
  statefulElement.inheritFromElement(ancestor);

  // Changes made in https://github.com/flutter/flutter/pull/44189
  const BuildContext buildContext = Element(myWidget);
  buildContext.inheritFromElement(ancestor);
  buildContext.inheritFromWidgetOfExactType(targetType);
  buildContext.ancestorInheritedElementForWidgetOfExactType(targetType);
  buildContext.ancestorWidgetOfExactType(targetType);
  buildContext.ancestorStateOfType(TypeMatcher<targetType>());
  buildContext.rootAncestorStateOfType(TypeMatcher<targetType>());
  buildContext.ancestorRenderObjectOfType(TypeMatcher<targetType>());

  // Changes made in https://github.com/flutter/flutter/pull/66305
  const Stack stack = Stack(overflow: Overflow.visible);
  const Stack stack = Stack(overflow: Overflow.clip);
  const Stack stack = Stack(error: '');
  final behavior = stack.overflow;

  // Changes made in https://github.com/flutter/flutter/pull/61648
  const Form form = Form(autovalidate: true);
  const Form form = Form(autovalidate: false);
  const Form form = Form(error: '');
  final autoMode = form.autovalidate;

  // Changes made in https://github.com/flutter/flutter/pull/61648
  const FormField formField = FormField(autovalidate: true);
  const FormField formField = FormField(autovalidate: false);
  const FormField formField = FormField(error: '');
  final autoMode = formField.autovalidate;

  // Changes made in https://github.com/flutter/flutter/pull/61648
  const TextFormField textFormField = TextFormField(autovalidate: true);
  const TextFormField textFormField = TextFormField(autovalidate: false);
  const TextFormField textFormField = TextFormField(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/61648
  const DropdownButtonFormField dropDownButtonFormField =
      DropdownButtonFormField(autovalidate: true);
  const DropdownButtonFormField dropdownButtonFormField =
      DropdownButtonFormField(autovalidate: false);
  const DropdownButtonFormField dropdownButtonFormField =
      DropdownButtonFormField(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/68736
  MediaQuery.of(context, nullOk: true);
  MediaQuery.of(context, nullOk: false);
  MediaQuery.of(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/70726
  Navigator.of(context, nullOk: true);
  Navigator.of(context, nullOk: false);
  Navigator.of(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/68908
  ScaffoldMessenger.of(context, nullOk: true);
  ScaffoldMessenger.of(context, nullOk: false);
  ScaffoldMessenger.of(error: '');
  Scaffold.of(error: '');
  Scaffold.of(context, nullOk: true);
  Scaffold.of(context, nullOk: false);

  // Changes made in https://github.com/flutter/flutter/pull/68910
  Router.of(context, nullOk: true);
  Router.of(context, nullOk: false);
  Router.of(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/68911
  Localizations.localeOf(context, nullOk: true);
  Localizations.localeOf(context, nullOk: false);
  Localizations.localeOf(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/68917
  FocusTraversalOrder.of(context, nullOk: true);
  FocusTraversalOrder.of(context, nullOk: false);
  FocusTraversalOrder.of(error: '');
  FocusTraversalGroup.of(error: '');
  FocusTraversalGroup.of(context, nullOk: true);
  FocusTraversalGroup.of(context, nullOk: false);
  Focus.of(context, nullOk: true);
  Focus.of(context, nullOk: false);
  Focus.of(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/68921
  Shortcuts.of(context, nullOk: true);
  Shortcuts.of(context, nullOk: false);
  Shortcuts.of(error: '');
  Actions.find(error: '');
  Actions.find(context, nullOk: true);
  Actions.find(context, nullOk: false);
  Actions.handler(context, nullOk: true);
  Actions.handler(context, nullOk: false);
  Actions.handler(error: '');
  Actions.invoke(error: '');
  Actions.invoke(context, nullOk: true);
  Actions.invoke(context, nullOk: false);

  // Changes made in https://github.com/flutter/flutter/pull/68925
  AnimatedList.of(context, nullOk: true);
  AnimatedList.of(context, nullOk: false);
  AnimatedList.of(error: '');
  SliverAnimatedList.of(error: '');
  SliverAnimatedList.of(context, nullOk: true);
  SliverAnimatedList.of(context, nullOk: false);

  // Changes made in https://github.com/flutter/flutter/pull/68905
  MaterialBasedCupertinoThemeData.resolveFrom(context, nullOk: true);
  MaterialBasedCupertinoThemeData.resolveFrom(context, nullOk: false);
  MaterialBasedCupertinoThemeData.resolveFrom(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/72043
  TextField(maxLengthEnforced: true);
  TextField(maxLengthEnforced: false);
  TextField(error: '');
  final TextField textField;
  textField.maxLengthEnforced;
  TextFormField(maxLengthEnforced: true);
  TextFormField(maxLengthEnforced: false);
  TextFormField(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/59127
  const BottomNavigationBarItem bottomNavigationBarItem =
      BottomNavigationBarItem(title: myTitle);
  const BottomNavigationBarItem bottomNavigationBarItem =
      BottomNavigationBarItem();
  const BottomNavigationBarItem bottomNavigationBarItem =
      BottomNavigationBarItem(error: '');
  bottomNavigationBarItem.title;

  // Changes made in https://github.com/flutter/flutter/pull/65246
  RectangularSliderTrackShape(disabledThumbGapWidth: 2.0);
  RectangularSliderTrackShape(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/79160
  Draggable draggable = Draggable();
  draggable = Draggable(dragAnchor: DragAnchor.child);
  draggable = Draggable(dragAnchor: DragAnchor.pointer);
  draggable = Draggable(error: '');
  draggable.dragAnchor;

  // Changes made in https://github.com/flutter/flutter/pull/79160
  LongPressDraggable longPressDraggable = LongPressDraggable();
  longPressDraggable = LongPressDraggable(dragAnchor: DragAnchor.child);
  longPressDraggable = LongPressDraggable(dragAnchor: DragAnchor.pointer);
  longPressDraggable = LongPressDraggable(error: '');
  longPressDraggable.dragAnchor;

  // Changes made in https://github.com/flutter/flutter/pull/64254
  final LeafRenderObjectElement leafElement = LeafRenderObjectElement();
  leafElement.insertChildRenderObject(renderObject, object);
  leafElement.moveChildRenderObject(renderObject, object);
  leafElement.removeChildRenderObject(renderObject);
  final ListWheelElement listWheelElement = ListWheelElement();
  listWheelElement.insertChildRenderObject(renderObject, object);
  listWheelElement.moveChildRenderObject(renderObject, object);
  listWheelElement.removeChildRenderObject(renderObject);
  final MultiChildRenderObjectElement multiChildRenderObjectElement =
      MultiChildRenderObjectElement();
  multiChildRenderObjectElement.insertChildRenderObject(renderObject, object);
  multiChildRenderObjectElement.moveChildRenderObject(renderObject, object);
  multiChildRenderObjectElement.removeChildRenderObject(renderObject);
  final SingleChildRenderObjectElement singleChildRenderObjectElement =
      SingleChildRenderObjectElement();
  singleChildRenderObjectElement.insertChildRenderObject(renderObject, object);
  singleChildRenderObjectElement.moveChildRenderObject(renderObject, object);
  singleChildRenderObjectElement.removeChildRenderObject(renderObject);
  final SliverMultiBoxAdaptorElement sliverMultiBoxAdaptorElement =
      SliverMultiBoxAdaptorElement();
  sliverMultiBoxAdaptorElement.insertChildRenderObject(renderObject, object);
  sliverMultiBoxAdaptorElement.moveChildRenderObject(renderObject, object);
  sliverMultiBoxAdaptorElement.removeChildRenderObject(renderObject);
  final RenderObjectToWidgetElement renderObjectToWidgetElement =
      RenderObjectToWidgetElement(widget);
  renderObjectToWidgetElement.insertChildRenderObject(renderObject, object);
  renderObjectToWidgetElement.moveChildRenderObject(renderObject, object);
  renderObjectToWidgetElement.removeChildRenderObject(renderObject);

  // Changes made in https://docs.flutter.dev/release/breaking-changes/clip-behavior
  ListWheelScrollView listWheelScrollView = ListWheelScrollView();
  listWheelScrollView = ListWheelScrollView(clipToSize: true);
  listWheelScrollView = ListWheelScrollView(clipToSize: false);
  listWheelScrollView = ListWheelScrollView(error: '');
  listWheelScrollView = ListWheelScrollView.useDelegate(error: '');
  listWheelScrollView = ListWheelScrollView.useDelegate();
  listWheelScrollView = ListWheelScrollView.useDelegate(clipToSize: true);
  listWheelScrollView = ListWheelScrollView.useDelegate(clipToSize: false);
  listWheelScrollView.clipToSize;
  ListWheelViewport listWheelViewport = ListWheelViewport();
  listWheelViewport = ListWheelViewport(clipToSize: true);
  listWheelViewport = ListWheelViewport(clipToSize: false);
  listWheelViewport = ListWheelViewport(error: '');
  listWheelViewport.clipToSize;

  // Changes made in https://github.com/flutter/flutter/pull/87839
  final OverscrollIndicatorNotification notification =
      OverscrollIndicatorNotification(leading: true);
  final OverscrollIndicatorNotification notification =
      OverscrollIndicatorNotification(error: '');
  notification.disallowGlow();

  // Changes made in https://github.com/flutter/flutter/pull/96115
  Icon icon = Icons.pie_chart_outlined;

  // Changes made in https://github.com/flutter/flutter/pull/96957
  Scrollbar scrollbar = Scrollbar(isAlwaysShown: true);
  bool nowShowing = scrollbar.isAlwaysShown;
  ScrollbarThemeData scrollbarTheme = ScrollbarThemeData(
    isAlwaysShown: nowShowing,
  );
  scrollbarTheme.copyWith(isAlwaysShown: nowShowing);
  scrollbarTheme.isAlwaysShown;
  RawScrollbar rawScrollbar = RawScrollbar(isAlwaysShown: true);
  nowShowing = rawScrollbar.isAlwaysShown;

  // Changes made in https://github.com/flutter/flutter/pull/96174
  Chip chip = Chip();
  chip = Chip(useDeleteButtonTooltip: false);
  chip = Chip(useDeleteButtonTooltip: true);
  chip = Chip(
    useDeleteButtonTooltip: false,
    deleteButtonTooltipMessage: 'Delete Tooltip',
  );
  chip.useDeleteButtonTooltip;

  // Changes made in https://github.com/flutter/flutter/pull/96174
  InputChip inputChip = InputChip();
  inputChip = InputChip(useDeleteButtonTooltip: false);
  inputChip = InputChip(useDeleteButtonTooltip: true);
  inputChip = InputChip(
    useDeleteButtonTooltip: false,
    deleteButtonTooltipMessage: 'Delete Tooltip',
  );
  inputChip.useDeleteButtonTooltip;

  // Changes made in https://github.com/flutter/flutter/pull/96174
  RawChip rawChip = Rawchip();
  rawChip = RawChip(useDeleteButtonTooltip: false);
  rawChip = RawChip(useDeleteButtonTooltip: true);
  rawChip = RawChip(
    useDeleteButtonTooltip: false,
    deleteButtonTooltipMessage: 'Delete Tooltip',
  );
  rawChip.useDeleteButtonTooltip;

  // Change made in https://github.com/flutter/flutter/pull/100381
  TextSelectionOverlay.fadeDuration;

  // Changes made in https://github.com/flutter/flutter/pull/105291
  ButtonStyle elevationButtonStyle = ElevatedButton.styleFrom(
    primary: Colors.blue,
    onPrimary: Colors.white,
    onSurface: Colors.grey,
  );
  ButtonStyle outlinedButtonStyle = OutlinedButton.styleFrom(
    primary: Colors.blue,
    onSurface: Colors.grey,
  );
  ButtonStyle textButtonStyle = TextButton.styleFrom(
    primary: Colors.blue,
    onSurface: Colors.grey,
  );

  // Changes made in https://github.com/flutter/flutter/pull/78588
  final ScrollBehavior scrollBehavior = ScrollBehavior();
  scrollBehavior.buildViewportChrome(context, child, axisDirection);
  final MaterialScrollBehavior materialScrollBehavior =
      MaterialScrollBehavior();
  materialScrollBehavior.buildViewportChrome(context, child, axisDirection);

  // Changes made in https://github.com/flutter/flutter/pull/111706
  Scrollbar scrollbar = Scrollbar(showTrackOnHover: true);
  bool nowShowing = scrollbar.showTrackOnHover;
  // The 3 expressions below have `bulkApply` set to true thus can't be tested yet.
  //ScrollbarThemeData scrollbarTheme = ScrollbarThemeData(showTrackOnHover: nowShowing);
  //scrollbarTheme.copyWith(showTrackOnHover: nowShowing);
  //scrollbarTheme.showTrackOnHover;

  // Changes made in https://github.com/flutter/flutter/pull/114459
  MediaQuery.boldTextOverride(context);

  // Changes made in https://github.com/flutter/flutter/pull/122555
  final ScrollableDetails details = ScrollableDetails(
    direction: AxisDirection.down,
    clipBehavior: Clip.none,
  );
  final Clip clip = details.clipBehavior;

  // Changes made in https://github.com/flutter/flutter/pull/134417
  const Curve curve = standardEasing;
  const Curve curve = accelerateEasing;
  const Curve curve = decelerateEasing;

  final PlatformMenuBar platformMenuBar = PlatformMenuBar(
    menus: <PlatformMenuItem>[],
    body: const SizedBox(),
  );
  final Widget bodyValue = platformMenuBar.body;

  // Changes made in https://github.com/flutter/flutter/pull/154972
  final InputBorder outlineBorder = MaterialStateOutlineInputBorder.resolveWith(
    (states) => const OutlineInputBorder(),
  );
  final InputBorder underlineBorder =
      MaterialStateUnderlineInputBorder.resolveWith(
        (states) => const UnderlineInputBorder(),
      );
}
