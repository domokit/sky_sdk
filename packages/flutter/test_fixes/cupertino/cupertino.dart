// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

void main() {
  // Generic reference variables.
  BuildContext context;
  RenderObjectWidget renderObjectWidget;
  RenderObject renderObject;
  Object object;

  // Change made in https://github.com/flutter/flutter/pull/41859
  CupertinoTextThemeData themeData = CupertinoTextThemeData(
    brightness: Brightness.dark,
  );
  themeData.copyWith(brightness: Brightness.light);
  themeData = CupertinoTextThemeData(error: '');
  themeData.copyWith(error: '');

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

  // Changes made in https://github.com/flutter/flutter/pull/68736
  MediaQuery.of(context, nullOk: true);
  MediaQuery.of(context, nullOk: false);
  MediaQuery.of(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/70726
  Navigator.of(context, nullOk: true);
  Navigator.of(context, nullOk: false);
  Navigator.of(error: '');

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
  CupertinoDynamicColor.resolve(Color(0), context, nullOk: true);
  CupertinoDynamicColor.resolve(Color(0), context, nullOk: false);
  CupertinoDynamicColor.resolve(error: '');
  CupertinoDynamicColor.resolveFrom(error: '');
  CupertinoDynamicColor.resolveFrom(context, nullOk: true);
  CupertinoDynamicColor.resolveFrom(context, nullOk: false);
  CupertinoUserInterfaceLevel.of(context, nullOk: true);
  CupertinoUserInterfaceLevel.of(context, nullOk: false);
  CupertinoUserInterfaceLevel.of(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/68736
  CupertinoTheme.brightnessOf(context, nullOk: true);
  CupertinoTheme.brightnessOf(context, nullOk: false);
  CupertinoTheme.brightnessOf(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/68905
  CupertinoThemeData.resolveFrom(context, nullOk: true);
  CupertinoThemeData.resolveFrom(context, nullOk: false);
  CupertinoThemeData.resolveFrom(error: '');
  NoDefaultCupertinoThemeData.resolveFrom(error: '');
  NoDefaultCupertinoThemeData.resolveFrom(context, nullOk: true);
  NoDefaultCupertinoThemeData.resolveFrom(context, nullOk: false);
  CupertinoTextThemeData.resolveFrom(context, nullOk: true);
  CupertinoTextThemeData.resolveFrom(context, nullOk: false);
  CupertinoTextThemeData.resolveFrom(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/72043
  CupertinoTextField(maxLengthEnforced: true);
  CupertinoTextField(maxLengthEnforced: false);
  CupertinoTextField(error: '');
  CupertinoTextField.borderless(error: '');
  CupertinoTextField.borderless(maxLengthEnforced: true);
  CupertinoTextField.borderless(maxLengthEnforced: false);
  final CupertinoTextField textField;
  textField.maxLengthEnforced;

  // Changes made in https://github.com/flutter/flutter/pull/59127
  const BottomNavigationBarItem bottomNavigationBarItem =
      BottomNavigationBarItem(title: myTitle);
  const BottomNavigationBarItem bottomNavigationBarItem =
      BottomNavigationBarItem();
  const BottomNavigationBarItem bottomNavigationBarItem =
      BottomNavigationBarItem(error: '');
  bottomNavigationBarItem.title;

  // Changes made in https://github.com/flutter/flutter/pull/79160
  Draggable draggable = Draggable();
  draggable = Draggable(error: '');
  draggable = Draggable(dragAnchor: DragAnchor.child);
  draggable = Draggable(dragAnchor: DragAnchor.pointer);
  draggable.dragAnchor;

  // Changes made in https://github.com/flutter/flutter/pull/79160
  LongPressDraggable longPressDraggable = LongPressDraggable();
  longPressDraggable = LongPressDraggable(error: '');
  longPressDraggable = LongPressDraggable(dragAnchor: DragAnchor.child);
  longPressDraggable = LongPressDraggable(dragAnchor: DragAnchor.pointer);
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
  listWheelScrollView = ListWheelScrollView.useDelegate();
  listWheelScrollView = ListWheelScrollView.useDelegate(clipToSize: true);
  listWheelScrollView = ListWheelScrollView.useDelegate(clipToSize: false);
  listWheelScrollView = ListWheelScrollView.useDelegate(error: '');
  listWheelScrollView.clipToSize;
  ListWheelViewport listWheelViewport = ListWheelViewport();
  listWheelViewport = ListWheelViewport(clipToSize: true);
  listWheelViewport = ListWheelViewport(clipToSize: false);
  listWheelViewport = ListWheelViewport(error: '');
  listWheelViewport.clipToSize;

  // Changes made in https://github.com/flutter/flutter/pull/87839
  OverscrollIndicatorNotification notification =
      OverscrollIndicatorNotification(leading: true);
  notification = OverscrollIndicatorNotification(error: '');
  notification.disallowGlow();

  // Changes made in https://github.com/flutter/flutter/pull/96957
  CupertinoScrollbar scrollbar = CupertinoScrollbar(isAlwaysShown: true);
  bool nowShowing = scrollbar.isAlwaysShown;
  RawScrollbar rawScrollbar = RawScrollbar(isAlwaysShown: true);
  nowShowing = rawScrollbar.isAlwaysShown;

  // Change made in https://github.com/flutter/flutter/pull/100381
  TextSelectionOverlay.fadeDuration;

  // Changes made in https://github.com/flutter/flutter/pull/78588
  final ScrollBehavior scrollBehavior = ScrollBehavior();
  scrollBehavior.buildViewportChrome(context, child, axisDirection);
  final CupertinoScrollBehavior cupertinoScrollBehavior =
      CupertinoScrollBehavior();
  cupertinoScrollBehavior.buildViewportChrome(context, child, axisDirection);

  // Changes made in https://github.com/flutter/flutter/pull/114459
  MediaQuery.boldTextOverride(context);

  // Changes made in https://github.com/flutter/flutter/pull/122555
  final ScrollableDetails details = ScrollableDetails(
    direction: AxisDirection.down,
    clipBehavior: Clip.none,
  );
  final Clip clip = details.clipBehavior;

  final PlatformMenuBar platformMenuBar = PlatformMenuBar(
    menus: <PlatformMenuItem>[],
    body: const SizedBox(),
  );
  final Widget bodyValue = platformMenuBar.body;

  // Changes made in https://github.com/flutter/flutter/pull/151367
  final cupertinoSwitch = CupertinoSwitch(
    value: value,
    onChanged: onChanged,
    activeColor: Colors.red,
  );
  Color? activeTrackColor = cupertinoSwitch.activeColor;

  // Changes made in https://github.com/flutter/flutter/pull/151367
  final cupertinoSwitch = CupertinoSwitch(
    value: value,
    onChanged: onChanged,
    trackColor: Colors.red,
  );
  Color? inactiveTrackColor = cupertinoSwitch.trackColor;

  // https://github.com/flutter/flutter/pull/152981
  CupertinoCheckbox(inactiveColor: Colors.red);
  CupertinoCheckbox(inactiveColor: Colors.red, activeColor: Colors.white);
  CupertinoCheckbox(
    inactiveColor: Colors.red,
    fillColor: WidgetStatePropertyAll(CupertinoColors.white),
  );
}
