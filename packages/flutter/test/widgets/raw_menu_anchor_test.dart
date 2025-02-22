// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_testing/leak_tracker_testing.dart';

// Tests that apply to select constructors have a suffix:
//  * [Default]: Applies to [RawMenuAnchor]
//  * [Group]: Applies to [RawMenuAnchorGroup].
// Otherwise, a test applies to all constructors.

void main() {
  late MenuController controller;
  String? focusedMenu;
  final List<Tag> selected = <Tag>[];
  final List<Tag> opened = <Tag>[];
  final List<Tag> closed = <Tag>[];

  void onPressed(Tag item) {
    selected.add(item);
  }

  void onOpen(Tag item) {
    opened.add(item);
  }

  void onClose(Tag item) {
    opened.remove(item);
    closed.add(item);
  }

  void handleFocusChange() {
    focusedMenu = (primaryFocus?.debugLabel ?? primaryFocus).toString();
  }

  setUp(() {
    focusedMenu = null;
    selected.clear();
    opened.clear();
    closed.clear();
    controller = MenuController();
    focusedMenu = null;
  });

  Future<void> changeSurfaceSize(WidgetTester tester, Size size) async {
    await tester.binding.setSurfaceSize(size);
    addTearDown(() async {
      await tester.binding.setSurfaceSize(null);
    });
  }

  void listenForFocusChanges() {
    FocusManager.instance.addListener(handleFocusChange);
    addTearDown(() => FocusManager.instance.removeListener(handleFocusChange));
  }

  T findMenuPanelDescendent<T extends Widget>(WidgetTester tester) {
    return tester.firstWidget<T>(find.descendant(of: find.byType(Panel), matching: find.byType(T)));
  }

  List<RenderObject> findAncestorRenderTheaters(RenderObject child) {
    final List<RenderObject> results = <RenderObject>[];
    RenderObject? node = child;
    while (node != null) {
      if (node.runtimeType.toString() == '_RenderTheater') {
        results.add(node);
      }
      final RenderObject? parent = node.parent;
      node = parent is RenderObject ? parent : null;
    }
    return results;
  }

  testWidgets("[Default] MenuController.isOpen is true when a menu's overlay is shown", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(find.text(Tag.a.text), findsOneWidget);

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(controller.isOpen, isFalse);
    expect(find.text(Tag.a.text), findsNothing);
  });

  testWidgets("MenuController.animationStatus is completed when a menu's overlay is shown", (
    WidgetTester tester,
  ) async {
    final MenuController anchorController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Menu(
            controller: anchorController,
            menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    expect(controller.animationStatus, equals(AnimationStatus.dismissed));
    expect(anchorController.animationStatus, equals(AnimationStatus.dismissed));
    expect(find.text(Tag.a.text), findsNothing);

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(controller.animationStatus, equals(AnimationStatus.completed));
    expect(anchorController.animationStatus, equals(AnimationStatus.completed));
    expect(find.text(Tag.a.text), findsOneWidget);

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(controller.animationStatus, equals(AnimationStatus.dismissed));
    expect(anchorController.animationStatus, equals(AnimationStatus.dismissed));
    expect(find.text(Tag.a.text), findsNothing);
  });

  testWidgets('[Default] MenuController.open() and .close() toggle overlay visibility', (
    WidgetTester tester,
  ) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          menuPanel: Panel(
            children: <Widget>[
              Text(Tag.a.text),
              Menu(
                controller: nestedController,
                menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                child: const AnchorButton(Tag.b),
              ),
            ],
          ),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    // Create the menu. The menu is closed, so no menu items should be found in
    // the widget tree.
    expect(controller.isOpen, isFalse);
    expect(find.text(Tag.anchor.text), findsOne);
    expect(find.text(Tag.a.text), findsNothing);

    // Open the menu.
    controller.open();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(nestedController.isOpen, isFalse);
    expect(find.text(Tag.a.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);

    // Open the nested menu.
    nestedController.open();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(nestedController.isOpen, isTrue);
    expect(find.text(Tag.a.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Close the menu from the root controller.
    controller.close();
    await tester.pump();

    // All menus should be closed.
    expect(controller.isOpen, isFalse);
    expect(find.text(Tag.a.text), findsNothing);
    expect(find.text(Tag.b.a.text), findsNothing);

    // Open the nested menu.
    controller.open();
    await tester.pump();

    nestedController.open();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(nestedController.isOpen, isTrue);
    expect(find.text(Tag.a.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Close the nested menu, but not the root menu.
    nestedController.close();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(nestedController.isOpen, isFalse);
    expect(find.text(Tag.a.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);
  });

  testWidgets('[Default] MenuController.closeChildren closes submenu children', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          menuPanel: Panel(
            children: <Widget>[
              Text(Tag.a.text),
              Menu(
                focusNode: focusNode,
                menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                child: AnchorButton(Tag.b, focusNode: focusNode),
              ),
            ],
          ),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    await tester.tap(find.text(Tag.b.text));
    await tester.pump();

    focusNode.requestFocus();
    await tester.pump();

    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    controller.closeChildren();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);

    // Focus should stay on the anchor button.
    expect(FocusManager.instance.primaryFocus, focusNode);
  });

  testWidgets('[Default] Can only have one open child anchor', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          overlayBuilder: (BuildContext context, RawMenuOverlayInfo position) {
            return Column(
              children: <Widget>[
                Menu(
                  menuPanel: Panel(children: <Widget>[Text(Tag.a.a.text)]),
                  child: const AnchorButton(Tag.a),
                ),
                Menu(
                  menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                  child: const AnchorButton(Tag.b),
                ),
              ],
            );
          },
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.a.a.text), findsNothing);
    expect(find.text(Tag.b.a.text), findsNothing);

    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    expect(find.text(Tag.a.a.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);

    await tester.tap(find.text(Tag.b.text));
    await tester.pump();

    expect(find.text(Tag.a.a.text), findsNothing);
    expect(find.text(Tag.b.a.text), findsOneWidget);
  });

  testWidgets('[Default] Context menus can be nested', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        Menu(
          menuPanel: Panel(children: <Widget>[Button.tag(Tag.a.a)]),
          builder: (BuildContext context, MenuController controller, Widget? child) {
            return Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                const AnchorButton(Tag.a),
                Menu(
                  menuPanel: Panel(children: <Widget>[Button.tag(Tag.b.a)]),
                  child: const AnchorButton(Tag.b),
                ),
              ],
            );
          },
        ),
      ),
    );

    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    expect(find.text(Tag.a.a.text), findsOneWidget);

    await tester.tap(find.text(Tag.b.text));
    await tester.pump();

    expect(find.text(Tag.b.a.text), findsOneWidget);
  });

  testWidgets('[Group] MenuController.isOpen is true when a descendent menu is open', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Row(
            children: <Widget>[
              Menu(
                menuPanel: Panel(children: <Widget>[Text(Tag.a.a.text)]),
                child: const AnchorButton(Tag.a),
              ),
              // Menu should not need to be a direct descendent.
              Padding(
                padding: const EdgeInsets.all(8.0),
                child: Menu(
                  menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                  child: const AnchorButton(Tag.b),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(controller.isOpen, isFalse);

    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(find.text(Tag.a.a.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);

    await tester.tap(find.text(Tag.b.text));
    await tester.pump();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(find.text(Tag.a.a.text), findsNothing);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    await tester.tap(find.text(Tag.b.text));
    await tester.pump();

    expect(controller.isOpen, isFalse);
    expect(find.text(Tag.b.a.text), findsNothing);
  });

  testWidgets('[Group] MenuController.open does nothing', (WidgetTester tester) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Column(
            children: <Widget>[
              Menu(
                controller: nestedController,
                menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                child: const AnchorButton(Tag.b),
              ),
            ],
          ),
        ),
      ),
    );

    // Create the menu. The menu is closed, so no menu items should be found in
    // the widget tree.
    expect(controller.isOpen, isFalse);
    expect(find.text(Tag.b.text), findsOne);
    expect(find.text(Tag.b.a.text), findsNothing);

    // Open the menu (which should do nothing).
    controller.open();
    await tester.pump();

    expect(controller.isOpen, isFalse);
    expect(nestedController.isOpen, isFalse);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);
  });

  testWidgets('[Group] MenuController.close closes children', (WidgetTester tester) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Column(
            children: <Widget>[
              Menu(
                controller: nestedController,
                menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                child: const AnchorButton(Tag.b),
              ),
            ],
          ),
        ),
      ),
    );

    // Open the nested anchor.
    nestedController.open();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(nestedController.isOpen, isTrue);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Close the root menu panel
    controller.close();
    await tester.pump();

    expect(controller.isOpen, isFalse);
    expect(nestedController.isOpen, isFalse);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);
  });

  testWidgets('[Group] MenuController.closeChildren closes children', (WidgetTester tester) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Column(
            children: <Widget>[
              Menu(
                controller: nestedController,
                menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                child: const AnchorButton(Tag.b),
              ),
            ],
          ),
        ),
      ),
    );

    // Open the nested anchor.
    nestedController.open();
    await tester.pump();

    expect(controller.isOpen, isTrue);
    expect(nestedController.isOpen, isTrue);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Close the root menu panel.
    controller.closeChildren();
    await tester.pump();

    expect(controller.isOpen, isFalse);
    expect(nestedController.isOpen, isFalse);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);
  });

  testWidgets('[Group] Should only display one open child anchor at a time', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Row(
            children: <Widget>[
              Menu(
                menuPanel: Panel(children: <Widget>[Text(Tag.a.a.text)]),
                child: const AnchorButton(Tag.a),
              ),
              Menu(
                menuPanel: Panel(children: <Widget>[Text(Tag.b.a.text)]),
                child: const AnchorButton(Tag.b),
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(find.text(Tag.b.text), findsOneWidget);
    expect(find.text(Tag.a.a.text), findsNothing);
    expect(find.text(Tag.b.a.text), findsNothing);

    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    expect(find.text(Tag.a.a.text), findsOneWidget);
    expect(find.text(Tag.b.a.text), findsNothing);

    await tester.tap(find.text(Tag.b.text));
    await tester.pump();

    expect(find.text(Tag.a.a.text), findsNothing);
    expect(find.text(Tag.b.a.text), findsOneWidget);
  });

  testWidgets('MenuController.maybeIsOpenOf notifies dependents when isOpen changes', (
    WidgetTester tester,
  ) async {
    final MenuController groupController = MenuController();
    final MenuController controller = MenuController();
    final MenuController nestedController = MenuController();
    bool? panelIsOpen;
    bool? overlayIsOpen;
    bool? anchorIsOpen;
    int panelBuilds = 0;
    int anchorBuilds = 0;
    int overlayBuilds = 0;

    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: groupController,
          child: Column(
            children: <Widget>[
              // Panel context.
              Builder(
                builder: (BuildContext context) {
                  panelIsOpen = MenuController.maybeIsOpenOf(context);
                  panelBuilds += 1;
                  return Text(Tag.a.text);
                },
              ),
              Menu(
                controller: controller,
                menuPanel: Panel(
                  children: <Widget>[
                    // Overlay context.
                    Builder(
                      builder: (BuildContext context) {
                        overlayIsOpen = MenuController.maybeIsOpenOf(context);
                        overlayBuilds += 1;
                        return Text(Tag.b.a.a.text);
                      },
                    ),
                    Menu(
                      controller: nestedController,
                      menuPanel: Panel(children: <Widget>[Button.tag(Tag.b.a.b.a)]),
                      child: Button.tag(Tag.b.a.b),
                    ),
                  ],
                ),
                // Anchor context.
                child: Builder(
                  builder: (BuildContext context) {
                    anchorIsOpen = MenuController.maybeIsOpenOf(context);
                    anchorBuilds += 1;
                    return Text(Tag.b.a.text);
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(panelIsOpen, isFalse);
    expect(anchorIsOpen, isFalse);
    expect(panelBuilds, equals(1));
    expect(anchorBuilds, equals(1));
    expect(overlayBuilds, equals(0));

    controller.open();
    await tester.pump();

    expect(panelIsOpen, isTrue);
    expect(anchorIsOpen, isTrue);
    expect(overlayIsOpen, isTrue);
    expect(panelBuilds, equals(2));
    expect(anchorBuilds, equals(2));
    expect(overlayBuilds, equals(1));

    nestedController.open();
    await tester.pump();

    // No new builds should have occurred since all controllers are already open.
    expect(panelIsOpen, isTrue);
    expect(anchorIsOpen, isTrue);
    expect(overlayIsOpen, isTrue);
    expect(panelBuilds, equals(2));
    expect(anchorBuilds, equals(2));
    expect(overlayBuilds, equals(1));

    controller.close();
    await tester.pump();

    expect(panelIsOpen, isFalse);
    expect(anchorIsOpen, isFalse);

    // Will be true because builder cannot rebuild when the menu is closed.
    expect(overlayIsOpen, isTrue);
    expect(panelBuilds, equals(3));
    expect(anchorBuilds, equals(3));
    expect(overlayBuilds, equals(1));
  });

  testWidgets(
    'MenuController.maybeAnimationStatusOf notifies dependents when AnimationStatus changes',
    (WidgetTester tester) async {
      final MenuController groupController = MenuController();
      final MenuController controller = MenuController();
      final MenuController nestedController = MenuController();
      AnimationStatus? panelAnimationStatus;
      AnimationStatus? overlayAnimationStatus;
      AnimationStatus? anchorAnimationStatus;
      int panelBuilds = 0;
      int anchorBuilds = 0;
      int overlayBuilds = 0;

      await tester.pumpWidget(
        App(
          RawMenuAnchorGroup(
            controller: groupController,
            child: Column(
              children: <Widget>[
                // Panel context.
                Builder(
                  builder: (BuildContext context) {
                    panelAnimationStatus = MenuController.maybeAnimationStatusOf(context);
                    panelBuilds += 1;
                    return Text(Tag.a.text);
                  },
                ),
                Menu(
                  controller: controller,
                  menuPanel: Panel(
                    children: <Widget>[
                      // Overlay context.
                      Builder(
                        builder: (BuildContext context) {
                          overlayAnimationStatus = MenuController.maybeAnimationStatusOf(context);
                          overlayBuilds += 1;
                          return Text(Tag.b.a.a.text);
                        },
                      ),
                      Menu(
                        controller: nestedController,
                        menuPanel: Panel(children: <Widget>[Button.tag(Tag.b.a.b.a)]),
                        child: Button.tag(Tag.b.a.b),
                      ),
                    ],
                  ),
                  // Anchor context.
                  child: Builder(
                    builder: (BuildContext context) {
                      anchorAnimationStatus = MenuController.maybeAnimationStatusOf(context);
                      anchorBuilds += 1;
                      return Text(Tag.b.a.text);
                    },
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      expect(panelAnimationStatus, equals(AnimationStatus.dismissed));
      expect(anchorAnimationStatus, equals(AnimationStatus.dismissed));
      expect(panelBuilds, equals(1));
      expect(anchorBuilds, equals(1));
      expect(overlayBuilds, equals(0));

      controller.open();
      await tester.pump();

      expect(panelAnimationStatus, equals(AnimationStatus.completed));
      expect(anchorAnimationStatus, equals(AnimationStatus.completed));
      expect(overlayAnimationStatus, equals(AnimationStatus.completed));
      expect(panelBuilds, equals(2));
      expect(anchorBuilds, equals(2));
      expect(overlayBuilds, equals(1));

      nestedController.open();
      await tester.pump();

      // No new builds should have occurred since all controllers are already open.
      expect(panelAnimationStatus, equals(AnimationStatus.completed));
      expect(anchorAnimationStatus, equals(AnimationStatus.completed));
      expect(overlayAnimationStatus, equals(AnimationStatus.completed));
      expect(panelBuilds, equals(2));
      expect(anchorBuilds, equals(2));
      expect(overlayBuilds, equals(1));

      controller.close();
      await tester.pump();

      expect(panelAnimationStatus, equals(AnimationStatus.dismissed));
      expect(anchorAnimationStatus, equals(AnimationStatus.dismissed));

      // Will be true because builder cannot rebuild when the menu is closed.
      expect(overlayAnimationStatus, equals(AnimationStatus.completed));
      expect(panelBuilds, equals(3));
      expect(anchorBuilds, equals(3));
      expect(overlayBuilds, equals(1));
    },
  );

  testWidgets('MenuController can be changed', (WidgetTester tester) async {
    final MenuController controller = MenuController();
    final MenuController groupController = MenuController();

    final MenuController newController = MenuController();
    final MenuController newGroupController = MenuController();

    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Menu(
            controller: groupController,
            menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(controller.isOpen, isTrue);
    expect(groupController.isOpen, isTrue);
    expect(newController.isOpen, isFalse);
    expect(newGroupController.isOpen, isFalse);

    // Swap the controllers.
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: newController,
          child: Menu(
            controller: newGroupController,
            menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(controller.isOpen, isFalse);
    expect(groupController.isOpen, isFalse);
    expect(newController.isOpen, isTrue);
    expect(newGroupController.isOpen, isTrue);

    // Close the new controller.
    newController.close();
    await tester.pump();

    expect(newController.isOpen, isFalse);
    expect(newGroupController.isOpen, isFalse);
    expect(find.text(Tag.a.text), findsNothing);
  });

  testWidgets('[Group] MenuController can be moved to a different menu', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Menu(
            menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(controller.isOpen, isTrue);

    // Swap the controllers.
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          key: UniqueKey(),
          controller: controller,
          child: Menu(
            menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    expect(find.text(Tag.a.text), findsNothing);
    expect(controller.isOpen, isFalse);

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(controller.isOpen, isTrue);

    // Close the menu.
    controller.close();
    await tester.pump();

    expect(controller.isOpen, isFalse);
    expect(find.text(Tag.a.text), findsNothing);
  });

  testWidgets('[Default] MenuController can be moved to a different menu', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: MenuController(),
          child: Menu(
            controller: controller,
            menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(controller.isOpen, isTrue);

    // Swap the controllers.
    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: MenuController(),
          child: Menu(
            key: UniqueKey(),
            controller: controller,
            menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    expect(find.text(Tag.a.text), findsNothing);
    expect(controller.isOpen, isFalse);

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(controller.isOpen, isTrue);

    // Close the menu.
    controller.close();
    await tester.pump();

    expect(controller.isOpen, isFalse);
    expect(find.text(Tag.a.text), findsNothing);
  });

  testWidgets('MenuController.maybeOf does not notify dependents when MenuController changes', (
    WidgetTester tester,
  ) async {
    final GlobalKey anchorKey = GlobalKey();
    final MenuController demoControllerOne = MenuController();
    final MenuController demoControllerTwo = MenuController();

    MenuController? panelController;
    MenuController? overlayController;
    MenuController? anchorController;
    int panelBuilds = 0;
    int anchorBuilds = 0;
    int overlayBuilds = 0;

    Widget buildAnchor({required MenuController panel, required MenuController overlay}) {
      return App(
        RawMenuAnchorGroup(
          controller: panel,
          child: Column(
            children: <Widget>[
              // Panel context.
              Builder(
                builder: (BuildContext context) {
                  panelController = MenuController.maybeOf(context);
                  panelBuilds += 1;
                  return Text(Tag.a.text);
                },
              ),
              Menu(
                controller: overlay,
                menuPanel: Panel(
                  children: <Widget>[
                    // Overlay context.
                    Builder(
                      builder: (BuildContext context) {
                        overlayController = MenuController.maybeOf(context);
                        overlayBuilds += 1;
                        return Text(Tag.b.a.a.text);
                      },
                    ),
                  ],
                ),
                // Anchor context.
                child: Builder(
                  key: anchorKey,
                  builder: (BuildContext context) {
                    anchorController = MenuController.maybeOf(context);
                    anchorBuilds += 1;
                    return Text(Tag.b.a.text);
                  },
                ),
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAnchor(panel: demoControllerOne, overlay: demoControllerTwo));

    expect(panelController, isNot(controller));
    expect(anchorController, isNot(controller));

    await tester.pumpWidget(buildAnchor(panel: controller, overlay: demoControllerTwo));

    expect(panelController, equals(controller));
    expect(anchorController, isNot(controller));
    expect(panelBuilds, equals(2));
    expect(anchorBuilds, equals(2));

    MenuController.maybeOf(anchorKey.currentContext!)?.open();
    await tester.pump();

    expect(panelBuilds, equals(2));
    expect(anchorBuilds, equals(2));
    expect(overlayBuilds, equals(1));

    await tester.pumpWidget(buildAnchor(panel: demoControllerOne, overlay: controller));

    expect(panelController, isNot(controller));
    expect(anchorController, equals(controller));
    expect(overlayController, equals(controller));
    expect(panelBuilds, equals(3));
    expect(anchorBuilds, equals(3));
    expect(overlayBuilds, equals(2));
  });

  // Regression test for https://github.com/flutter/flutter/issues/156572.
  testWidgets('Detached MenuController does not throw when calling close', (
    WidgetTester tester,
  ) async {
    final MenuController controller = MenuController();
    controller.close();
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('Detached MenuController returns false when calling isOpen', (
    WidgetTester tester,
  ) async {
    final MenuController controller = MenuController();
    expect(controller.isOpen, false);
  });

  testWidgets(
    'Detached MenuController returns AnimationStatus.dismissed when calling animationStatus',
    (WidgetTester tester) async {
      final MenuController controller = MenuController();
      expect(controller.animationStatus, AnimationStatus.dismissed);
    },
  );

  testWidgets('[Default] MenuController is detached on update', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          menuPanel: const SizedBox.shrink(),
          child: const SizedBox.shrink(),
        ),
      ),
    );

    // Should not throw because the controller is attached to the menu.
    controller.closeChildren();

    await tester.pumpWidget(
      const App(Menu(menuPanel: SizedBox.shrink(), child: SizedBox.shrink())),
    );

    String serializedException = '';
    runZonedGuarded(controller.closeChildren, (Object exception, StackTrace stackTrace) {
      serializedException = exception.toString();
    });

    expect(serializedException, contains('_anchor != null'));
  });

  testWidgets('[Group] MenuController is detached on update', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(RawMenuAnchorGroup(controller: controller, child: const SizedBox.shrink())),
    );

    // Should not throw because the controller is attached to the menu.
    controller.closeChildren();

    await tester.pumpWidget(
      App(RawMenuAnchorGroup(controller: MenuController(), child: const SizedBox.shrink())),
    );

    String serializedException = '';
    runZonedGuarded(controller.closeChildren, (Object exception, StackTrace stackTrace) {
      serializedException = exception.toString();
    });

    expect(serializedException, contains('_anchor != null'));
  });

  testWidgets('[Default] MenuController is detached on dispose', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(Menu(controller: controller, menuPanel: const SizedBox(), child: const SizedBox())),
    );

    // Should not throw because the controller is attached to the menu.
    controller.closeChildren();

    await tester.pumpWidget(const App(SizedBox()));

    String serializedException = '';
    runZonedGuarded(controller.closeChildren, (Object exception, StackTrace stackTrace) {
      serializedException = exception.toString();
    });

    expect(serializedException, contains('_anchor != null'));
  });

  testWidgets('[Group] MenuController is detached on dispose', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(RawMenuAnchorGroup(controller: controller, child: const SizedBox())),
    );

    // Should not throw because the controller is attached to the menu.
    controller.closeChildren();

    await tester.pumpWidget(const App(SizedBox()));

    String serializedException = '';
    runZonedGuarded(controller.closeChildren, (Object exception, StackTrace stackTrace) {
      serializedException = exception.toString();
    });

    expect(serializedException, contains('_anchor != null'));
  });

  testWidgets('[Default] MenuOverlayPosition.anchorRect applies transformations to panel', (
    WidgetTester tester,
  ) async {
    RawMenuOverlayInfo? builderPosition;
    final GlobalKey anchorKey = GlobalKey();
    await tester.pumpWidget(
      App(
        Transform(
          transform: Matrix4.translationValues(-50, 50, 0)..scale(1.2),
          child: RawMenuAnchor(
            controller: controller,
            overlayBuilder: (BuildContext context, RawMenuOverlayInfo position) {
              builderPosition = position;
              return Positioned.fromRect(
                rect: position.anchorRect,
                child: Container(color: const Color(0xFF0000FF)),
              );
            },
            child: AnchorButton(Tag.b, key: anchorKey),
          ),
        ),
      ),
    );

    controller.open();
    await tester.pump();

    expect(tester.getRect(find.byType(AnchorButton)), equals(builderPosition?.anchorRect));
  });

  testWidgets('Escape key closes menus', (WidgetTester tester) async {
    final FocusNode aFocusNode = FocusNode();
    final FocusNode baaFocusNode = FocusNode();
    final MenuController menuController = MenuController();
    addTearDown(aFocusNode.dispose);
    addTearDown(baaFocusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchorGroup(
          controller: controller,
          child: Row(
            children: <Widget>[
              Button.tag(Tag.a, focusNode: aFocusNode),
              Menu(
                controller: menuController,
                menuPanel: Panel(
                  children: <Widget>[
                    Menu(
                      menuPanel: Panel(
                        children: <Widget>[Button.tag(Tag.b.a.a, focusNode: baaFocusNode)],
                      ),
                      child: AnchorButton(Tag.b.a),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.b),
              ),
            ],
          ),
        ),
      ),
    );

    menuController.open();
    await tester.pump();

    aFocusNode.requestFocus();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, aFocusNode);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Test panel child can close siblings with escape key.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();

    expect(find.text(Tag.b.a.text), findsNothing);

    await tester.tap(find.text(Tag.b.text));
    await tester.pump();
    await tester.tap(find.text(Tag.b.a.text));
    await tester.pump();
    baaFocusNode.requestFocus();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, baaFocusNode);

    // Test ancestors menus are closed with escape key.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();

    expect(find.text(Tag.b.a.text), findsNothing);
  });

  // Credit to Closure library for the test idea.
  testWidgets('Intents are not blocked by closed anchor', (WidgetTester tester) async {
    final List<Intent> invokedIntents = <Intent>[];
    final FocusNode aFocusNode = FocusNode();
    addTearDown(aFocusNode.dispose);

    await tester.pumpWidget(
      App(
        Actions(
          actions: <Type, Action<Intent>>{
            DirectionalFocusIntent: CallbackAction<DirectionalFocusIntent>(
              onInvoke: (DirectionalFocusIntent intent) {
                invokedIntents.add(intent);
                return;
              },
            ),
            NextFocusIntent: CallbackAction<NextFocusIntent>(
              onInvoke: (NextFocusIntent intent) {
                invokedIntents.add(intent);
                return;
              },
            ),
            PreviousFocusIntent: CallbackAction<PreviousFocusIntent>(
              onInvoke: (PreviousFocusIntent intent) {
                invokedIntents.add(intent);
                return;
              },
            ),
            DismissIntent: CallbackAction<DismissIntent>(
              onInvoke: (DismissIntent intent) {
                invokedIntents.add(intent);
                return;
              },
            ),
          },
          child: RawMenuAnchorGroup(
            controller: controller,
            child: Row(
              children: <Widget>[
                Menu(
                  menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
                  child: AnchorButton(Tag.anchor, focusNode: aFocusNode),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    aFocusNode.requestFocus();
    await tester.pump();
    Actions.invoke(aFocusNode.context!, const DirectionalFocusIntent(TraversalDirection.up));
    Actions.invoke(aFocusNode.context!, const NextFocusIntent());
    Actions.invoke(aFocusNode.context!, const PreviousFocusIntent());
    Actions.invoke(aFocusNode.context!, const DismissIntent());
    await tester.pump();

    expect(
      invokedIntents,
      equals(const <Intent>[
        DirectionalFocusIntent(TraversalDirection.up),
        NextFocusIntent(),
        PreviousFocusIntent(),
        DismissIntent(),
      ]),
    );
  });

  testWidgets('[Default] Focus traversal shortcuts are not bound to actions', (
    WidgetTester tester,
  ) async {
    final FocusNode anchorFocusNode = FocusNode(debugLabel: Tag.anchor.focusNode);
    final FocusNode bFocusNode = FocusNode(debugLabel: Tag.b.focusNode);
    addTearDown(anchorFocusNode.dispose);
    addTearDown(bFocusNode.dispose);

    final Map<ShortcutActivator, Intent> traversalShortcuts = <ShortcutActivator, Intent>{
      LogicalKeySet(LogicalKeyboardKey.tab): const NextFocusIntent(),
      LogicalKeySet(LogicalKeyboardKey.shift, LogicalKeyboardKey.tab): const PreviousFocusIntent(),
      LogicalKeySet(LogicalKeyboardKey.arrowLeft): const DirectionalFocusIntent(
        TraversalDirection.left,
      ),
    };

    final List<Intent> invokedIntents = <Intent>[];
    await tester.pumpWidget(
      App(
        Column(
          children: <Widget>[
            Button.tag(Tag.a),
            Actions(
              actions: <Type, Action<Intent>>{
                DirectionalFocusIntent: CallbackAction<DirectionalFocusIntent>(
                  onInvoke: (DirectionalFocusIntent intent) {
                    invokedIntents.add(intent);
                    return null;
                  },
                ),
                NextFocusIntent: CallbackAction<NextFocusIntent>(
                  onInvoke: (NextFocusIntent intent) {
                    invokedIntents.add(intent);
                    return null;
                  },
                ),
                PreviousFocusIntent: CallbackAction<PreviousFocusIntent>(
                  onInvoke: (PreviousFocusIntent intent) {
                    invokedIntents.add(intent);
                    return null;
                  },
                ),
              },
              child: RawMenuAnchor(
                controller: controller,
                overlayBuilder: (BuildContext context, RawMenuOverlayInfo position) {
                  return Column(
                    children: <Widget>[
                      Button.tag(Tag.a),
                      Shortcuts(
                        // Web doesn't automatically handle directional traversal.
                        shortcuts: traversalShortcuts,
                        child: Button.tag(Tag.b, focusNode: bFocusNode),
                      ),
                      Button.tag(Tag.d),
                    ],
                  );
                },
                child: AnchorButton(Tag.anchor, focusNode: anchorFocusNode),
              ),
            ),
            Button.tag(Tag.c),
          ],
        ),
      ),
    );

    listenForFocusChanges();

    controller.open();
    await tester.pump();

    anchorFocusNode.requestFocus();
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.home);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.end);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    bFocusNode.requestFocus();
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.home);
    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.end);
    expect(focusedMenu, equals(Tag.b.focusNode));

    expect(
      invokedIntents,
      equals(const <Intent>[
        DirectionalFocusIntent(TraversalDirection.left),
        NextFocusIntent(),
        PreviousFocusIntent(),
        DirectionalFocusIntent(TraversalDirection.left),
        NextFocusIntent(),
        PreviousFocusIntent(),
      ]),
    );
  });

  testWidgets('Actions that wrap Menu are invoked by both anchor and overlay', (
    WidgetTester tester,
  ) async {
    final FocusNode anchorFocusNode = FocusNode();
    final FocusNode aFocusNode = FocusNode();
    addTearDown(anchorFocusNode.dispose);
    addTearDown(aFocusNode.dispose);
    bool invokedAnchor = false;
    bool invokedOverlay = false;

    await tester.pumpWidget(
      App(
        Actions(
          actions: <Type, Action<Intent>>{
            VoidCallbackIntent: CallbackAction<VoidCallbackIntent>(
              onInvoke: (VoidCallbackIntent intent) {
                intent.callback();
                return null;
              },
            ),
          },
          child: Menu(
            focusNode: anchorFocusNode,
            menuPanel: Panel(children: <Widget>[Button.tag(Tag.a, focusNode: aFocusNode)]),
            child: AnchorButton(Tag.anchor, focusNode: anchorFocusNode),
          ),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    Actions.invoke(
      anchorFocusNode.context!,
      VoidCallbackIntent(() {
        invokedAnchor = true;
      }),
    );
    Actions.invoke(
      aFocusNode.context!,
      VoidCallbackIntent(() {
        invokedOverlay = true;
      }),
    );

    await tester.pump();

    // DismissIntent should not close the menu.
    expect(invokedAnchor, isTrue);
    expect(invokedOverlay, isTrue);
  });

  testWidgets('DismissMenuAction closes menus', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    await tester.pumpWidget(
      App(
        Menu(
          menuPanel: Panel(
            children: <Widget>[
              Text(Tag.a.text),
              Menu(
                menuPanel: Panel(
                  children: <Widget>[
                    Text(Tag.b.a.text),
                    Menu(
                      controller: controller,
                      menuPanel: Panel(children: <Widget>[Text(Tag.b.b.a.text)]),
                      child: AnchorButton(Tag.b.b, focusNode: focusNode),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.b),
              ),
            ],
          ),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();
    await tester.tap(find.text(Tag.b.text));
    await tester.pump();
    await tester.tap(find.text(Tag.b.b.text));
    await tester.pump();

    expect(controller.isOpen, isTrue);

    focusNode.requestFocus();
    await tester.pump();

    const ActionDispatcher().invokeAction(
      DismissMenuAction(controller: controller),
      const DismissIntent(),
      focusNode.context,
    );

    await tester.pump();

    expect(find.text(Tag.a.text), findsNothing);
  });

  testWidgets('[Group] Menu panel builder', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        alignment: AlignmentDirectional.topStart,
        RawMenuAnchorGroup(
          controller: controller,
          child: Padding(
            key: Tag.anchor.key,
            padding: const EdgeInsets.all(8.0),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                Container(width: 100, height: 100, color: const ui.Color(0xff0000ff)),
                Container(width: 100, height: 100, color: const ui.Color(0xFFFF00D4)),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.byKey(Tag.anchor.key), findsOneWidget);
    expect(tester.getRect(find.byKey(Tag.anchor.key)), const Rect.fromLTWH(0, 0, 216, 116));
  });

  testWidgets('[Default] Overlay builder is passed anchor rect', (WidgetTester tester) async {
    RawMenuOverlayInfo? overlayPosition;
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          overlayBuilder: (BuildContext context, RawMenuOverlayInfo position) {
            overlayPosition = position;
            return const SizedBox();
          },
          controller: controller,
          child: AnchorButton(Tag.anchor, onPressed: onPressed),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(overlayPosition!.anchorRect, tester.getRect(find.byType(Button)));
  });

  testWidgets('[Default] Overlay contents can be positioned', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          overlayBuilder: (BuildContext context, RawMenuOverlayInfo position) {
            return Positioned(
              top: position.anchorRect.top,
              left: position.anchorRect.left,
              child: Container(
                key: Tag.a.key,
                width: 200,
                height: 200,
                color: const Color(0xFF00FF00),
              ),
            );
          },
          child: AnchorButton(Tag.anchor, onPressed: onPressed),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    final ui.Offset anchorCorner = tester.getTopLeft(find.byType(Button));
    final ui.Rect contentRect = tester.getRect(find.byKey(Tag.a.key));

    expect(contentRect, anchorCorner & const Size(200, 200));
  });

  testWidgets('[Default] TapRegion group ID is passed to overlay', (WidgetTester tester) async {
    bool? insideTap;

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          overlayBuilder: (BuildContext context, RawMenuOverlayInfo position) {
            return Positioned.fromRect(
              rect: position.anchorRect.translate(200, 200),
              child: TapRegion(
                onTapInside: (PointerDownEvent event) {
                  insideTap = true;
                },
                onTapOutside: (PointerDownEvent event) {
                  insideTap = false;
                },
                groupId: insideTap ?? false ? null : position.tapRegionGroupId,
                child: Button.tag(Tag.a),
              ),
            );
          },
          child: AnchorButton(Tag.anchor, onPressed: onPressed),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);

    // Start by testing that the tap region has the correct group ID.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(insideTap, isTrue);

    // The menu should close when the tap region is tapped, so we need to
    // reopen.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(insideTap, isTrue);

    // Now test that setting the tap region group ID to null will cause the
    // tap to be considered outside the tap region.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(insideTap, isFalse);
  });

  testWidgets('Menus close and consume tap when consumesOutsideTap is true', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        Column(
          children: <Widget>[
            Button.tag(
              Tag.outside,
              onPressed: () {
                selected.add(Tag.outside);
              },
            ),
            RawMenuAnchorGroup(
              controller: controller,
              child: Column(
                children: <Widget>[
                  Menu(
                    consumeOutsideTaps: true,
                    onOpen: () => onOpen(Tag.anchor),
                    onClose: () => onClose(Tag.anchor),
                    menuPanel: Panel(
                      children: <Widget>[
                        Menu(
                          consumeOutsideTaps: true,
                          onOpen: () => onOpen(Tag.a),
                          onClose: () => onClose(Tag.a),
                          menuPanel: Panel(children: <Widget>[Text(Tag.a.a.text)]),
                          child: AnchorButton(Tag.a, onPressed: onPressed),
                        ),
                      ],
                    ),
                    child: AnchorButton(Tag.anchor, onPressed: onPressed),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );

    expect(opened, isEmpty);
    expect(closed, isEmpty);

    // Doesn't consume tap when the menu is closed.
    await tester.tap(find.text(Tag.outside.text));
    await tester.pump();

    expect(selected, equals(<NestedTag>[Tag.outside]));
    selected.clear();

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();
    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    expect(opened, equals(<NestedTag>[Tag.anchor, Tag.a]));
    expect(closed, isEmpty);
    expect(selected, equals(<NestedTag>[Tag.anchor, Tag.a]));
    opened.clear();
    closed.clear();
    selected.clear();

    await tester.tap(find.text(Tag.outside.text));
    await tester.pump();

    expect(opened, isEmpty);
    expect(closed, equals(<NestedTag>[Tag.a, Tag.anchor]));
    expect(selected, isEmpty);

    // When the menu is open, don't expect the outside button to be selected.
    expect(selected, isEmpty);
    selected.clear();
    opened.clear();
    closed.clear();
  });

  testWidgets('[Default] Menus close and do not consume tap when consumesOutsideTap is false', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        Column(
          children: <Widget>[
            Button.tag(
              Tag.outside,
              onPressed: () {
                selected.add(Tag.outside);
              },
            ),
            RawMenuAnchorGroup(
              controller: controller,
              child: Column(
                children: <Widget>[
                  Menu(
                    onOpen: () => onOpen(Tag.anchor),
                    onClose: () => onClose(Tag.anchor),
                    // ignore: avoid_redundant_argument_values
                    consumeOutsideTaps: false,
                    menuPanel: Panel(
                      children: <Widget>[
                        Menu(
                          onOpen: () => onOpen(Tag.a),
                          onClose: () => onClose(Tag.a),
                          menuPanel: Panel(children: <Widget>[Text(Tag.a.a.text)]),
                          child: AnchorButton(Tag.a, onPressed: onPressed),
                        ),
                      ],
                    ),
                    child: AnchorButton(Tag.anchor, onPressed: onPressed),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );

    expect(opened, isEmpty);
    expect(closed, isEmpty);

    await tester.tap(find.text(Tag.outside.text));
    await tester.pump();

    // Doesn't consume tap when the menu is closed.
    expect(selected, equals(<Tag>[Tag.outside]));

    selected.clear();

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();
    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    expect(opened, equals(<Tag>[Tag.anchor, Tag.a]));
    expect(closed, isEmpty);
    expect(selected, equals(<Tag>[Tag.anchor, Tag.a]));

    opened.clear();
    closed.clear();
    selected.clear();

    await tester.tap(find.text(Tag.outside.text));
    await tester.pumpAndSettle();

    // Because consumesOutsideTap is false, outsideButton is expected to
    // receive a tap.
    expect(opened, isEmpty);
    expect(closed, equals(<Tag>[Tag.a, Tag.anchor]));
    expect(selected, equals(<Tag>[Tag.outside]));

    selected.clear();
    opened.clear();
    closed.clear();
  });

  testWidgets('onOpen is called when the menu is opened', (WidgetTester tester) async {
    bool opened = false;
    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          onOpen: () {
            opened = true;
          },
          menuPanel: const Panel(children: <Widget>[]),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(opened, isTrue);

    opened = false;
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    // onOpen should not be called again.
    expect(opened, isFalse);

    controller.open();
    await tester.pump();

    expect(opened, isTrue);
  });

  testWidgets('onClose is called when the menu is closed', (WidgetTester tester) async {
    bool closed = true;
    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          onOpen: () {
            closed = false;
          },
          onClose: () {
            closed = true;
          },
          menuPanel: const Panel(children: <Widget>[]),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(closed, isFalse);

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(closed, isTrue);

    controller.open();
    await tester.pump();

    expect(closed, isFalse);

    controller.close();
    await tester.pump();

    expect(closed, isTrue);
  });

  testWidgets('[Default] diagnostics', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final Widget menuAnchor = RawMenuAnchor(
      controller: controller,
      childFocusNode: focusNode,
      overlayBuilder: (BuildContext context, RawMenuOverlayInfo info) {
        return const SizedBox();
      },
    );

    await tester.pumpWidget(App(menuAnchor));
    controller.open();
    await tester.pump();

    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    menuAnchor.debugFillProperties(builder);
    final List<String> properties =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(properties, const <String>['has focusNode', 'use nearest overlay']);
  });

  testWidgets('[Group] diagnostics', (WidgetTester tester) async {
    final Widget menuNode = RawMenuAnchorGroup(
      controller: controller,
      child: const SizedBox(height: 30, width: 30),
    );

    await tester.pumpWidget(App(menuNode));
    await tester.pump();

    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    menuNode.debugFillProperties(builder);
    final Iterable<String> properties = builder.properties
        .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
        .map((DiagnosticsNode node) => node.toString());
    expect(properties, equals(const <String>['has controller']));
  });

  testWidgets('Surface clip behavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          menuPanel: const Panel(children: <Widget>[Text('Button 1')]),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    controller.open();
    await tester.pump();

    // Test default clip behavior.
    expect(findMenuPanelDescendent<Container>(tester).clipBehavior, equals(Clip.antiAlias));

    await tester.pumpWidget(
      App(
        Menu(
          controller: controller,
          menuPanel: const Panel(clipBehavior: Clip.hardEdge, children: <Widget>[Text('Button 1')]),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    // Test custom clip behavior.
    expect(findMenuPanelDescendent<Container>(tester).clipBehavior, equals(Clip.hardEdge));
  });

  // Menu implementations differ as to whether tabbing traverses a closes a
  // menu or traverses its items. By default, we let the user choose whether
  // to close the menu or traverse its items.
  testWidgets('Tab traversal is not handled', (WidgetTester tester) async {
    final FocusNode bFocusNode = FocusNode(debugLabel: Tag.b.focusNode);
    final FocusNode bbFocusNode = FocusNode(debugLabel: Tag.b.b.focusNode);
    addTearDown(bFocusNode.dispose);
    addTearDown(bbFocusNode.dispose);
    final List<Intent> invokedIntents = <Intent>[];

    await tester.pumpWidget(
      App(
        Row(
          children: <Widget>[
            Actions(
              actions: <Type, Action<Intent>>{
                NextFocusIntent: CallbackAction<NextFocusIntent>(
                  onInvoke: (NextFocusIntent intent) {
                    invokedIntents.add(intent);
                    return null;
                  },
                ),
                PreviousFocusIntent: CallbackAction<PreviousFocusIntent>(
                  onInvoke: (PreviousFocusIntent intent) {
                    invokedIntents.add(intent);
                    return null;
                  },
                ),
              },
              child: RawMenuAnchorGroup(
                controller: controller,
                child: Column(
                  children: <Widget>[
                    Button.tag(Tag.a),
                    Menu(
                      menuPanel: Panel(
                        children: <Widget>[
                          Button.tag(Tag.b.a),
                          Button.tag(Tag.b.b, focusNode: bbFocusNode),
                          Button.tag(Tag.b.c),
                        ],
                      ),
                      child: AnchorButton(Tag.b, focusNode: bFocusNode),
                    ),
                    Button.tag(Tag.c),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );

    listenForFocusChanges();

    bFocusNode.requestFocus();
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));

    // Open and move focus to nested menu
    await tester.tap(find.text(Tag.b.text));
    await tester.pump();
    bbFocusNode.requestFocus();
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.b.focusNode));

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.b.focusNode));
    expect(
      invokedIntents,
      equals(const <Intent>[
        NextFocusIntent(),
        PreviousFocusIntent(),
        NextFocusIntent(),
        PreviousFocusIntent(),
      ]),
    );
  });

  testWidgets('Menu closes on view size change', (WidgetTester tester) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    final MediaQueryData mediaQueryData = MediaQueryData.fromView(tester.view);

    bool opened = false;
    bool closed = false;

    Widget build(Size size) {
      return MediaQuery(
        data: mediaQueryData.copyWith(size: size),
        child: App(
          SingleChildScrollView(
            controller: scrollController,
            child: Container(
              height: 1000,
              alignment: Alignment.center,
              child: Menu(
                onOpen: () {
                  opened = true;
                  closed = false;
                },
                onClose: () {
                  opened = false;
                  closed = true;
                },
                controller: controller,
                menuPanel: Panel(children: <Widget>[Text(Tag.a.text)]),
                child: const AnchorButton(Tag.anchor),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build(mediaQueryData.size));
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(opened, isTrue);
    expect(closed, isFalse);

    const Size smallSize = Size(200, 200);
    await changeSurfaceSize(tester, smallSize);
    await tester.pumpWidget(build(smallSize));

    expect(opened, isFalse);
    expect(closed, isTrue);
  });

  testWidgets('Menu closes on ancestor scroll', (WidgetTester tester) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      App(
        SingleChildScrollView(
          controller: scrollController,
          child: Menu(
            onOpen: () {
              onOpen(Tag.anchor);
            },
            onClose: () {
              onClose(Tag.anchor);
            },
            menuPanel: Panel(
              children: <Widget>[
                Button.tag(Tag.a),
                Button.tag(Tag.b),
                Button.tag(Tag.c),
                Button.tag(Tag.d),
              ],
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(opened, isNotEmpty);
    expect(closed, isEmpty);
    opened.clear();

    scrollController.jumpTo(1000);
    await tester.pump();

    expect(opened, isEmpty);
    expect(closed, isNotEmpty);
  });

  testWidgets('Menus do not close on root menu internal scroll', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/122168.
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    bool rootOpened = false;
    const BoxConstraints largeButtonConstraints = BoxConstraints.tightFor(width: 200, height: 300);

    await tester.pumpWidget(
      App(
        SingleChildScrollView(
          controller: scrollController,
          child: Container(
            height: 700,
            alignment: Alignment.topLeft,
            child: Menu(
              onOpen: () {
                rootOpened = true;
              },
              onClose: () {
                rootOpened = false;
              },
              menuPanel: Panel(
                children: <Widget>[
                  Menu(
                    onOpen: () {
                      onOpen(Tag.a);
                    },
                    onClose: () {
                      onClose(Tag.a);
                    },
                    menuPanel: Panel(
                      children: <Widget>[Button.tag(Tag.a.a, constraints: largeButtonConstraints)],
                    ),
                    child: const AnchorButton(Tag.a, constraints: largeButtonConstraints),
                  ),
                  Button.tag(Tag.b, constraints: largeButtonConstraints),
                  Button.tag(Tag.c, constraints: largeButtonConstraints),
                  Button.tag(Tag.d, constraints: largeButtonConstraints),
                ],
              ),
              child: const AnchorButton(Tag.anchor),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();
    expect(rootOpened, true);

    // Hover the first submenu anchor.
    final TestPointer pointer = TestPointer(1, ui.PointerDeviceKind.mouse);
    await tester.tap(find.text(Tag.a.text));
    await tester.sendEventToBinding(pointer.hover(tester.getCenter(find.text(Tag.a.text))));
    await tester.pump();
    expect(opened, equals(<Tag>[Tag.a]));

    // Menus do not close on internal scroll.
    await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 30.0)));
    await tester.pump();
    expect(rootOpened, true);
    expect(closed, isEmpty);

    // Menus close on external scroll.
    scrollController.jumpTo(700);
    await tester.pump();
    expect(rootOpened, false);
    expect(closed, equals(<Tag>[Tag.a]));
  });

  // Copied from [MenuAnchor] tests.
  //
  // Regression test for https://github.com/flutter/flutter/issues/157606.
  testWidgets('Menu builder rebuilds when isOpen state changes', (WidgetTester tester) async {
    bool isOpen = false;
    int openCount = 0;
    int closeCount = 0;

    await tester.pumpWidget(
      App(
        Menu(
          menuPanel: Panel(children: <Widget>[Button.text('Menu Item')]),
          builder: (BuildContext context, MenuController controller, Widget? child) {
            isOpen = controller.isOpen;
            return Button(
              Text(isOpen ? 'close' : 'open'),
              onPressed: () {
                if (controller.isOpen) {
                  controller.close();
                } else {
                  controller.open();
                }
              },
            );
          },
          onOpen: () => openCount++,
          onClose: () => closeCount++,
        ),
      ),
    );

    expect(find.text('open'), findsOneWidget);
    expect(isOpen, false);
    expect(openCount, 0);
    expect(closeCount, 0);

    await tester.tap(find.text('open'));
    await tester.pump();

    expect(find.text('close'), findsOneWidget);
    expect(isOpen, true);
    expect(openCount, 1);
    expect(closeCount, 0);

    await tester.tap(find.text('close'));
    await tester.pump();

    expect(find.text('open'), findsOneWidget);
    expect(isOpen, false);
    expect(openCount, 1);
    expect(closeCount, 1);
  });

  // Copied from [MenuAnchor] tests.
  //
  // Regression test for https://github.com/flutter/flutter/issues/155034.
  testWidgets('Content is shown in the root overlay when useRootOverlay is true', (
    WidgetTester tester,
  ) async {
    final MenuController controller = MenuController();
    final UniqueKey overlayKey = UniqueKey();
    final Finder a = find.text(Tag.a.text);
    final Finder aa = find.text(Tag.a.a.text);

    late final OverlayEntry overlayEntry;
    addTearDown(() {
      overlayEntry.remove();
      overlayEntry.dispose();
    });

    await tester.pumpWidget(
      App(
        Overlay(
          key: overlayKey,
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return Center(
                  child: Menu(
                    useRootOverlay: true,
                    controller: controller,
                    menuPanel: Panel(
                      children: <Widget>[
                        Menu(
                          menuPanel: Panel(children: <Widget>[Button.tag(Tag.a.a)]),
                          child: const AnchorButton(Tag.a),
                        ),
                      ],
                    ),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(a, findsNothing);

    // Open the menu.
    controller.open();
    await tester.pump();
    await tester.tap(a);
    await tester.pump();

    expect(a, findsOne);
    expect(aa, findsOne);

    // Expect two overlays: the root overlay created by WidgetsApp and the
    // overlay created by the boilerplate code.
    expect(find.byType(Overlay), findsNWidgets(2));

    final Iterable<Overlay> overlays = tester.widgetList<Overlay>(find.byType(Overlay));
    final Overlay nonRootOverlay = tester.widget(find.byKey(overlayKey));
    final Overlay rootOverlay = overlays.firstWhere((Overlay overlay) => overlay != nonRootOverlay);

    final RenderObject menuTheater = findAncestorRenderTheaters(tester.renderObject(a)).first;
    final RenderObject submenuTheater = findAncestorRenderTheaters(tester.renderObject(aa)).first;

    // Check that the ancestor _RenderTheater for the menu item is the one
    // from the root overlay.
    expect(menuTheater, tester.renderObject(find.byWidget(rootOverlay)));
    expect(menuTheater, submenuTheater);
  });

  testWidgets('Content is shown in the nearest ancestor overlay when useRootOverlay is false', (
    WidgetTester tester,
  ) async {
    final MenuController controller = MenuController();
    final UniqueKey overlayKey = UniqueKey();
    final Finder a = find.text(Tag.a.text);
    final Finder aa = find.text(Tag.a.a.text);

    late final OverlayEntry overlayEntry;
    addTearDown(() {
      overlayEntry.remove();
      overlayEntry.dispose();
    });

    await tester.pumpWidget(
      App(
        Overlay(
          key: overlayKey,
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return Center(
                  child: Menu(
                    controller: controller,
                    menuPanel: Panel(
                      children: <Widget>[
                        // Nested menus should be rendered in the same overlay as
                        // their parent, so useRootOverlay should have no effect.
                        Menu(
                          useRootOverlay: true,
                          menuPanel: Panel(children: <Widget>[Button.tag(Tag.a.a)]),
                          child: const AnchorButton(Tag.a),
                        ),
                      ],
                    ),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(a, findsNothing);

    // Open the menu.
    controller.open();
    await tester.pump();
    await tester.tap(a);
    await tester.pump();

    expect(a, findsOne);
    expect(aa, findsOne);

    // Expect two overlays: the root overlay created by WidgetsApp and the
    // overlay created by the boilerplate code.
    expect(find.byType(Overlay), findsNWidgets(2));

    final Overlay nonRootOverlay = tester.widget(find.byKey(overlayKey));
    final RenderObject menuTheater = findAncestorRenderTheaters(tester.renderObject(a)).first;
    final RenderObject submenuTheater = findAncestorRenderTheaters(tester.renderObject(aa)).first;

    // Check that the ancestor _RenderTheater for the menu item is the one
    // from the root overlay.
    expect(menuTheater, tester.renderObject(find.byWidget(nonRootOverlay)));
    expect(menuTheater, submenuTheater);
  });

  testWidgets('Parent updates are not triggered during builds', (WidgetTester tester) async {
    // This test ensures that _MenuAnchor._childChangedOpenState does not
    // rebuild a child's parent if that parent is currently building.
    final MediaQueryData mediaQueryData = MediaQueryData.fromView(tester.view);

    Widget build(Size size) {
      return App(
        MediaQuery(
          data: mediaQueryData.copyWith(size: size),
          child: RawMenuAnchorGroup(
            controller: controller,
            child: const Menu(
              menuPanel: Panel(children: <Widget>[]),
              child: AnchorButton(Tag.anchor),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(build(mediaQueryData.size));

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    const Size smallSize = Size(200, 200);
    await changeSurfaceSize(tester, smallSize);

    await tester.pumpWidget(build(smallSize));
    await tester.pump();

    expect(tester.takeException(), isNull);
  });

  group('MenuControllerDecorator', () {
    testWidgets('External controller triggers opening and closing animations', (
      WidgetTester tester,
    ) async {
      final Key panelKey = UniqueKey();
      late Animation<double> animationController;

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              Animation<double> animation,
            ) {
              animationController = animation;
              return Menu(
                menuPanel: SizedBox(key: panelKey),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Overlay is closed; animation is at 0.
      expect(controller.isOpen, isFalse);
      expect(animationController.value, equals(0));
      expect(find.byKey(panelKey), findsNothing);

      // Open menu
      controller.open();

      expect(controller.isOpen, isTrue);
      expect(find.byKey(panelKey), findsNothing);

      await tester.pump();

      expect(find.byKey(panelKey), findsOneWidget);

      await tester.pump(const Duration(milliseconds: 100));

      expect(controller.isOpen, isTrue);
      expect(animationController.value, equals(0.5));
      expect(find.byKey(panelKey), findsOneWidget);

      await tester.pump(const Duration(milliseconds: 101));

      expect(controller.isOpen, isTrue);
      expect(animationController.value, equals(1));
      expect(find.byKey(panelKey), findsOneWidget);

      // Close menu
      controller.close();

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      expect(controller.isOpen, isTrue);
      expect(animationController.value, closeTo(0.5, 0.01));
      expect(find.byKey(panelKey), findsOneWidget);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 101));

      expect(controller.isOpen, isFalse);
      expect(animationController.value, equals(0));
      expect(find.byKey(panelKey), findsNothing);
    });

    testWidgets('Decorated controller triggers opening and closing animations', (
      WidgetTester tester,
    ) async {
      final Key panelKey = UniqueKey();
      late AnimationController animationController;
      late MenuControllerDecorator decorator;

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              animationController = animation;
              decorator = decoratedController;
              return Menu(
                menuPanel: SizedBox(key: panelKey),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Overlay is closed, animation is at 0.
      expect(decorator.isOpen, isFalse);
      expect(animationController.value, equals(0));
      expect(find.byKey(panelKey), findsNothing);

      // Open menu
      decorator.open();

      expect(decorator.isOpen, isTrue);
      expect(find.byKey(panelKey), findsNothing);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      expect(decorator.isOpen, isTrue);
      expect(animationController.value, closeTo(0.5, 0.01));
      expect(find.byKey(panelKey), findsOneWidget);

      await tester.pump(const Duration(milliseconds: 101));

      // Fully open
      expect(decorator.isOpen, isTrue);
      expect(animationController.value, equals(1));
      expect(find.byKey(panelKey), findsOneWidget);

      // Close menu
      decorator.close();

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 100));

      expect(decorator.isOpen, isTrue);
      expect(animationController.value, closeTo(0.5, 0.01));
      expect(find.byKey(panelKey), findsOneWidget);

      await tester.pump(const Duration(milliseconds: 101));

      // Fully closed
      expect(decorator.isOpen, isFalse);
      expect(animationController.value, equals(0));
      expect(find.byKey(panelKey), findsNothing);
    });

    testWidgets('Animations can be interrupted', (WidgetTester tester) async {
      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            builder: (
              BuildContext context,
              MenuControllerDecorator controller,
              AnimationController animation,
            ) {
              return Menu(
                menuPanel: SizeTransition(
                  sizeFactor: animation,
                  child: Container(color: const Color(0xFF000000), width: 100, height: 100),
                ),
                controller: controller,
                child: Button.tag(
                  Tag.anchor,
                  onPressed: () {
                    if (controller.isOpen) {
                      controller.close();
                    } else {
                      controller.open();
                    }
                  },
                ),
              );
            },
          ),
        ),
      );

      // Start opening
      controller.open();
      await tester.pump();

      expect(controller.animationStatus, equals(AnimationStatus.forward));

      // Reverse before animation completes
      await tester.pump(const Duration(milliseconds: 100));
      controller.close();
      await tester.pump();

      expect(controller.animationStatus, equals(AnimationStatus.reverse));

      // Test that the interrupted animation dismisses
      await tester.pump(const Duration(milliseconds: 101));

      expect(controller.isOpen, isFalse);
      expect(controller.animationStatus, equals(AnimationStatus.dismissed));

      // Reopen halfway
      controller.open();
      await tester.pump();

      expect(controller.animationStatus, equals(AnimationStatus.forward));

      // Reverse
      await tester.pump(const Duration(milliseconds: 100));
      controller.close();
      await tester.pump();

      expect(controller.animationStatus, equals(AnimationStatus.reverse));

      await tester.pump(const Duration(milliseconds: 25));

      // Reverse again
      controller.open();

      await tester.pump(const Duration(milliseconds: 25));

      expect(controller.animationStatus, equals(AnimationStatus.forward));

      // Test that the interrupted animation completes
      await tester.pump(const Duration(milliseconds: 200));

      expect(controller.isOpen, isTrue);
      expect(controller.animationStatus, equals(AnimationStatus.completed));
    });

    testWidgets(
      'MenuController.maybeAnimationStatusOf notifies dependents on AnimationStatus changes',
      (WidgetTester tester) async {
        final MenuController groupController = MenuController();
        final MenuController controller = MenuController();
        final MenuController nestedController = MenuController();
        AnimationStatus? panelAnimationStatus;
        AnimationStatus? overlayAnimationStatus;
        AnimationStatus? anchorAnimationStatus;
        int panelBuilds = 0;
        int anchorBuilds = 0;
        int overlayBuilds = 0;

        await tester.pumpWidget(
          App(
            RawMenuAnchorGroup(
              controller: groupController,
              child: Column(
                children: <Widget>[
                  // Panel context.
                  Builder(
                    builder: (BuildContext context) {
                      panelAnimationStatus = MenuController.maybeAnimationStatusOf(context);
                      panelBuilds += 1;
                      return Text(Tag.a.text);
                    },
                  ),
                  DecoratedMenu(
                    controller: controller,
                    builder: (
                      BuildContext context,
                      MenuControllerDecorator decoratedController,
                      AnimationController animation,
                    ) {
                      return Menu(
                        controller: decoratedController,
                        menuPanel: Panel(
                          children: <Widget>[
                            Builder(
                              builder: (BuildContext context) {
                                overlayAnimationStatus = MenuController.maybeAnimationStatusOf(
                                  context,
                                );
                                overlayBuilds += 1;
                                return Text(Tag.b.a.a.text);
                              },
                            ),
                            Menu(
                              controller: nestedController,
                              menuPanel: Panel(children: <Widget>[Button.tag(Tag.b.a.b.a)]),
                              child: Button.tag(Tag.b.a.b),
                            ),
                          ],
                        ),
                        child: Builder(
                          builder: (BuildContext context) {
                            anchorAnimationStatus = MenuController.maybeAnimationStatusOf(context);
                            anchorBuilds += 1;
                            return Text(Tag.b.a.text);
                          },
                        ),
                      );
                    },
                  ),
                ],
              ),
            ),
          ),
        );

        expect(panelAnimationStatus, equals(AnimationStatus.dismissed));
        expect(anchorAnimationStatus, equals(AnimationStatus.dismissed));
        expect(panelBuilds, equals(1));
        expect(anchorBuilds, equals(1));
        expect(overlayBuilds, equals(0));

        controller.open();
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 50));
        expect(panelAnimationStatus, equals(AnimationStatus.completed));
        expect(anchorAnimationStatus, equals(AnimationStatus.forward));
        expect(overlayAnimationStatus, equals(AnimationStatus.forward));
        expect(panelBuilds, equals(2));
        expect(anchorBuilds, equals(2));
        expect(overlayBuilds, equals(1));

        await tester.pump(const Duration(milliseconds: 151));

        expect(panelAnimationStatus, equals(AnimationStatus.completed));
        expect(anchorAnimationStatus, equals(AnimationStatus.completed));
        expect(overlayAnimationStatus, equals(AnimationStatus.completed));
        expect(panelBuilds, equals(2));
        expect(anchorBuilds, equals(3));
        expect(overlayBuilds, equals(2));

        nestedController.open();
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 201));

        // No new builds should have occurred since all controllers are already open.
        expect(panelAnimationStatus, equals(AnimationStatus.completed));
        expect(anchorAnimationStatus, equals(AnimationStatus.completed));
        expect(overlayAnimationStatus, equals(AnimationStatus.completed));
        expect(panelBuilds, equals(2));
        expect(anchorBuilds, equals(3));
        expect(overlayBuilds, equals(2));

        controller.close();
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 100));

        expect(panelAnimationStatus, equals(AnimationStatus.completed));
        expect(overlayAnimationStatus, equals(AnimationStatus.reverse));
        expect(anchorAnimationStatus, equals(AnimationStatus.reverse));
        expect(panelBuilds, equals(2));
        expect(anchorBuilds, equals(4));
        expect(overlayBuilds, equals(3));

        await tester.pump(const Duration(milliseconds: 101));

        // overlayAnimationStatus will be AnimationStatus.reverse because the
        // builder cannot rebuild when the menu is closed.
        expect(panelAnimationStatus, equals(AnimationStatus.dismissed));
        expect(overlayAnimationStatus, equals(AnimationStatus.reverse));
        expect(anchorAnimationStatus, equals(AnimationStatus.dismissed));
        expect(panelBuilds, equals(3));
        expect(anchorBuilds, equals(5));
        expect(overlayBuilds, equals(3));
      },
    );

    testWidgets('External controller transitions through all menu animation states', (
      WidgetTester tester,
    ) async {
      late AnimationController menuAnimation;
      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              menuAnimation = animation;
              return Menu(
                menuPanel: const SizedBox(),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Checks that the status of the menu controller and the menu animation
      // are in sync. This is done before and after a pump to ensure that the
      // status is updated synchronously and stays updated.
      Future<void> statusMatches(AnimationStatus status) async {
        expect(controller.animationStatus, status);
        expect(menuAnimation.status, status);

        await tester.pump();

        expect(controller.animationStatus, status);
        expect(menuAnimation.status, status);
      }

      // Initial state should be closed
      await statusMatches(AnimationStatus.dismissed);

      // Test: closed -> opening
      controller.open();

      await statusMatches(AnimationStatus.forward);

      // Test: opening -> opened
      await tester.pump(const Duration(milliseconds: 201));

      await statusMatches(AnimationStatus.completed);

      // Test: opened -> closing
      controller.close();

      await statusMatches(AnimationStatus.reverse);

      await tester.pump(const Duration(milliseconds: 100));

      // Test: closing -> opening
      controller.open();

      await statusMatches(AnimationStatus.forward);

      await tester.pump(const Duration(milliseconds: 50));

      // Test: opening -> closing
      controller.close();

      await statusMatches(AnimationStatus.reverse);

      // Test: closing -> closed
      await tester.pump(const Duration(milliseconds: 201));

      await statusMatches(AnimationStatus.dismissed);
    });

    testWidgets('Decorated controller transitions through all menu animation states', (
      WidgetTester tester,
    ) async {
      late MenuControllerDecorator decorator;
      late Animation<double> menuAnimation;

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              Animation<double> animation,
            ) {
              decorator = decoratedController;
              menuAnimation = animation;
              return Menu(
                menuPanel: const SizedBox(),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Checks that the status of the decorated menu controller and the menu
      // animation, are in sync. This is done before and after a pump to ensure
      // that the status is updated synchronously and stays updated.
      Future<void> statusMatches(AnimationStatus status) async {
        expect(decorator.animationStatus, status);
        expect(menuAnimation.status, status);

        await tester.pump();

        expect(decorator.animationStatus, status);
        expect(menuAnimation.status, status);
      }

      // Initial state should be closed
      await statusMatches(AnimationStatus.dismissed);

      // Test: closed -> opening
      decorator.open();

      await statusMatches(AnimationStatus.forward);

      // Test: opening -> opened
      await tester.pump(const Duration(milliseconds: 201));

      await statusMatches(AnimationStatus.completed);

      // Test: opened -> closing
      decorator.close();

      await statusMatches(AnimationStatus.reverse);

      await tester.pump(const Duration(milliseconds: 100));

      // Test: closing -> opening
      decorator.open();

      await statusMatches(AnimationStatus.forward);

      await tester.pump(const Duration(milliseconds: 50));

      // Test: opening -> closing
      decorator.close();

      await statusMatches(AnimationStatus.reverse);

      // Test: closing -> closed
      await tester.pump(const Duration(milliseconds: 201));

      await statusMatches(AnimationStatus.dismissed);

      // Test: closed -> opened (forced with markMenuOpened)
      decorator.markMenuOpened();

      await statusMatches(AnimationStatus.completed);

      // Test: opened -> closed (forced with markMenuClosed)
      decorator.markMenuClosed();

      await statusMatches(AnimationStatus.dismissed);

      decorator.open();

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      await statusMatches(AnimationStatus.forward);

      // Test: opening -> opened (forced with markMenuOpened)
      decorator.markMenuOpened();

      await statusMatches(AnimationStatus.completed);

      // Test: closing -> closed (forced with markMenuClosed)
      decorator.close();

      await statusMatches(AnimationStatus.reverse);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      decorator.markMenuClosed();

      await statusMatches(AnimationStatus.dismissed);

      decorator.open();

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      await statusMatches(AnimationStatus.forward);

      // Test: opening -> closed (forced with markMenuClosed)
      decorator.markMenuClosed();

      await statusMatches(AnimationStatus.dismissed);

      decorator.open();

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      decorator.close();

      await statusMatches(AnimationStatus.reverse);

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      // Test: closing -> opened (forced with markMenuOpened)
      decorator.markMenuOpened();

      await statusMatches(AnimationStatus.completed);
    });

    testWidgets('External controller can be changed', (WidgetTester tester) async {
      final MenuController controllerTwo = MenuController();
      late MenuController decorator;

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              decorator = decoratedController;
              return Menu(
                consumeOutsideTaps: true,
                menuPanel: SizeTransition(sizeFactor: animation, child: const SizedBox()),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      controller.open();

      await tester.pump();
      await tester.pump(const Duration(milliseconds: 50));

      expect(controller.isOpen, isTrue);
      expect(controller.animationStatus, equals(AnimationStatus.forward));
      expect(decorator.isOpen, isTrue);
      expect(decorator.animationStatus, equals(AnimationStatus.forward));

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controllerTwo,
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              decorator = decoratedController;
              return Menu(
                consumeOutsideTaps: true,
                menuPanel: SizeTransition(sizeFactor: animation, child: const SizedBox()),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      await tester.pump();

      expect(controller.isOpen, isFalse);
      expect(controller.animationStatus, equals(AnimationStatus.dismissed));

      expect(controllerTwo.isOpen, isTrue);
      expect(controllerTwo.animationStatus, equals(AnimationStatus.forward));

      expect(decorator.isOpen, isTrue);
      expect(decorator.animationStatus, equals(AnimationStatus.forward));

      controllerTwo.close();

      await tester.pump();

      expect(controllerTwo.animationStatus, equals(AnimationStatus.reverse));
      expect(decorator.animationStatus, equals(AnimationStatus.reverse));

      decorator.open();

      await tester.pump();

      expect(controllerTwo.animationStatus, equals(AnimationStatus.forward));
      expect(decorator.animationStatus, equals(AnimationStatus.forward));
    });

    testWidgets('External controller position is passed to handleMenuOpenRequest.', (
      WidgetTester tester,
    ) async {
      Offset? position;
      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            onOpenRequest: (ui.Offset? value) {
              position = value;
            },
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              return Menu(
                consumeOutsideTaps: true,
                menuPanel: SizeTransition(sizeFactor: animation, child: const SizedBox()),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Pass empty position.
      controller.open();
      await tester.pump();

      expect(position, isNull);

      // Pass position.

      controller.open(position: const Offset(100, 100));
      await tester.pump();

      expect(position, equals(const Offset(100, 100)));
    });

    testWidgets('Open position is passed to handleMenuOpenRequest.', (WidgetTester tester) async {
      late MenuController decorator;
      Offset? position;
      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            onOpenRequest: (ui.Offset? value) {
              position = value;
            },
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              decorator = decoratedController;
              return Menu(
                consumeOutsideTaps: true,
                menuPanel: SizeTransition(sizeFactor: animation, child: const SizedBox()),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Pass empty position.
      decorator.open();
      await tester.pump();

      expect(position, isNull);

      decorator.open(position: const Offset(100, 100));
      await tester.pump();

      expect(position, equals(const Offset(100, 100)));

      controller.open();
      await tester.pump();

      expect(position, isNull);

      controller.open(position: const Offset(200, 200));
      await tester.pump();

      expect(position, equals(const Offset(200, 200)));
    });

    testWidgets('Position is passed to handleMenuOpenRequest.', (WidgetTester tester) async {
      Offset? position;

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            onOpenRequest: (ui.Offset? value) {
              position = value;
            },
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              return Menu(
                consumeOutsideTaps: true,
                menuPanel: SizeTransition(sizeFactor: animation, child: const SizedBox()),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Pass empty position.
      controller.open();
      await tester.pump();

      expect(position, isNull);

      // Pass position.
      controller.open(position: const Offset(100, 100));
      await tester.pump();

      expect(position, equals(const Offset(100, 100)));
    });

    testWidgets('Position is passed to handleMenuOpenRequest.', (WidgetTester tester) async {
      Offset? position;

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            onOpenRequest: (ui.Offset? value) {
              position = value;
            },
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              return Menu(
                consumeOutsideTaps: true,
                menuPanel: SizeTransition(sizeFactor: animation, child: const SizedBox()),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Pass empty position.
      controller.open();
      await tester.pump();

      expect(position, isNull);

      // Pass position.
      controller.open(position: const Offset(100, 100));
      await tester.pump();

      expect(position, equals(const Offset(100, 100)));
    });

    testWidgets('Position is received by overlayBuilder', (WidgetTester tester) async {
      Offset? position;

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            onOpenRequest: (ui.Offset? value) {
              position = value;
            },
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              return Menu(
                consumeOutsideTaps: true,
                overlayBuilder: (BuildContext context, RawMenuOverlayInfo info) {
                  position = info.position;
                  return Positioned(left: position?.dx, top: position?.dy, child: const SizedBox());
                },
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor),
              );
            },
          ),
        ),
      );

      // Pass empty position.
      controller.open();
      await tester.pump();

      expect(position, isNull);

      // Pass position.
      controller.open(position: const Offset(100, 100));
      await tester.pump();

      expect(position, equals(const Offset(100, 100)));
    });

    testWidgets('Ancestor scroll triggers handleMenuCloseRequest', (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      late Animation<double> rootMenuAnimation;

      await tester.pumpWidget(
        App(
          SingleChildScrollView(
            controller: scrollController,
            child: Container(
              height: 1000,
              alignment: Alignment.center,
              child: DecoratedMenu(
                controller: controller,
                builder: (
                  BuildContext context,
                  MenuControllerDecorator decoratedController,
                  AnimationController animation,
                ) {
                  rootMenuAnimation = animation;
                  return Menu(
                    consumeOutsideTaps: true,
                    menuPanel: SizeTransition(
                      sizeFactor: animation,
                      child: Panel(
                        decoration: const BoxDecoration(color: Color(0xFF121212)),
                        children: <Widget>[
                          Button.tag(Tag.a),
                          Button.tag(Tag.b),
                          Button.tag(Tag.c),
                          Button.tag(Tag.d),
                        ],
                      ),
                    ),
                    controller: decoratedController,
                    child: const AnchorButton(Tag.anchor),
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 202));

      expect(controller.animationStatus, equals(AnimationStatus.completed));
      expect(rootMenuAnimation.value, equals(1.0));

      scrollController.jumpTo(1000);
      await tester.pump();

      expect(controller.animationStatus, equals(AnimationStatus.reverse));

      await tester.pump(const Duration(milliseconds: 100));

      expect(controller.animationStatus, equals(AnimationStatus.reverse));
      expect(rootMenuAnimation.value, closeTo(0.5, 0.01));

      await tester.pump(const Duration(milliseconds: 101));

      expect(controller.animationStatus, equals(AnimationStatus.dismissed));
      expect(rootMenuAnimation.value, equals(0.0));
    });

    testWidgets('View size change triggers handleMenuCloseRequest', (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      late Animation<double> rootMenuAnimation;
      final MediaQueryData mediaQueryData = MediaQueryData.fromView(tester.view);

      Widget buildWidget(MediaQueryData data) {
        return MediaQuery(
          data: data,
          child: App(
            SingleChildScrollView(
              controller: scrollController,
              child: Container(
                height: 1000,
                alignment: Alignment.center,
                child: DecoratedMenu(
                  controller: controller,
                  builder: (
                    BuildContext context,
                    MenuControllerDecorator decoratedController,
                    AnimationController animation,
                  ) {
                    rootMenuAnimation = animation;
                    return Menu(
                      consumeOutsideTaps: true,
                      menuPanel: SizeTransition(
                        sizeFactor: animation,
                        child: Panel(
                          decoration: const BoxDecoration(color: Color(0xFF121212)),
                          children: <Widget>[
                            Button.tag(Tag.a),
                            Button.tag(Tag.b),
                            Button.tag(Tag.c),
                            Button.tag(Tag.d),
                          ],
                        ),
                      ),
                      controller: decoratedController,
                      child: const AnchorButton(Tag.anchor),
                    );
                  },
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildWidget(mediaQueryData));

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 202));

      expect(controller.animationStatus, equals(AnimationStatus.completed));
      expect(rootMenuAnimation.value, equals(1.0));

      const Size smallSize = Size(200, 200);
      await changeSurfaceSize(tester, smallSize);
      await tester.pumpWidget(buildWidget(mediaQueryData.copyWith(size: smallSize)));

      await tester.pump();

      expect(controller.animationStatus, equals(AnimationStatus.reverse));

      await tester.pump(const Duration(milliseconds: 100));

      expect(controller.animationStatus, equals(AnimationStatus.reverse));
      expect(rootMenuAnimation.value, closeTo(0.5, 0.01));

      await tester.pump(const Duration(milliseconds: 101));

      expect(controller.animationStatus, equals(AnimationStatus.dismissed));
      expect(rootMenuAnimation.value, equals(0.0));
    });

    testWidgets('DismissMenuAction triggers handleMenuCloseRequest', (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(
        App(
          DecoratedMenu(
            controller: controller,
            builder: (
              BuildContext context,
              MenuControllerDecorator decoratedController,
              AnimationController animation,
            ) {
              return Menu(
                consumeOutsideTaps: true,
                menuPanel: SizeTransition(
                  sizeFactor: animation,
                  child: Panel(
                    decoration: const BoxDecoration(color: Color(0xFF121212)),
                    children: <Widget>[
                      Button.tag(Tag.a),
                      Button.tag(Tag.b),
                      Button.tag(Tag.c),
                      Button.tag(Tag.d),
                    ],
                  ),
                ),
                controller: decoratedController,
                child: const AnchorButton(Tag.anchor, autofocus: true),
              );
            },
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 201));

      expect(controller.isOpen, isTrue);
      expect(controller.animationStatus, equals(AnimationStatus.completed));

      focusNode.requestFocus();
      await tester.pump();

      const ActionDispatcher().invokeAction(
        DismissMenuAction(controller: controller),
        const DismissIntent(),
        focusNode.context,
      );

      await tester.pump();

      expect(controller.isOpen, isTrue);
      expect(controller.animationStatus, equals(AnimationStatus.reverse));

      await tester.pump(const Duration(milliseconds: 201));

      expect(controller.isOpen, isFalse);
      expect(controller.animationStatus, equals(AnimationStatus.dismissed));
    });

    testWidgets('Outside tap triggers handleMenuCloseRequest', (WidgetTester tester) async {
      final MenuController groupController = MenuController();

      await tester.pumpWidget(
        App(
          RawMenuAnchorGroup(
            controller: groupController,
            child: DecoratedMenu(
              controller: controller,
              builder: (
                BuildContext context,
                MenuControllerDecorator decoratedController,
                AnimationController animation,
              ) {
                return Menu(
                  consumeOutsideTaps: true,
                  menuPanel: SizeTransition(
                    sizeFactor: animation,
                    child: Panel(
                      decoration: const BoxDecoration(color: Color(0xFF121212)),
                      children: <Widget>[
                        Button.tag(Tag.a),
                        Button.tag(Tag.b),
                        Button.tag(Tag.c),
                        Button.tag(Tag.d),
                      ],
                    ),
                  ),
                  controller: decoratedController,
                  child: const AnchorButton(Tag.anchor, autofocus: true),
                );
              },
            ),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 201));

      expect(controller.isOpen, isTrue);
      expect(controller.animationStatus, equals(AnimationStatus.completed));

      await tester.tapAt(Offset.zero);
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 25));

      expect(controller.isOpen, isTrue);
      expect(controller.animationStatus, equals(AnimationStatus.reverse));

      await tester.pump(const Duration(milliseconds: 201));

      expect(controller.isOpen, isFalse);
      expect(controller.animationStatus, equals(AnimationStatus.dismissed));
    });

    testWidgets(
      'Throws an AssertionError if a MenuControllerDecorator wraps a MenuControllerDecorator',
      experimentalLeakTesting:
          LeakTesting.settings
              .withIgnoredAll(), // leaking by design because of exception. See https://github.com/dart-lang/leak_tracker/blob/main/doc/leak_tracking/TROUBLESHOOT.md#4-the-test-throws-flutter-exception
      (WidgetTester tester) async {
        await tester.pumpWidget(
          App(
            DecoratedMenu(
              controller: controller,
              builder: (
                BuildContext context,
                MenuControllerDecorator decoratedController,
                AnimationController animation,
              ) {
                return DecoratedMenu(
                  controller: decoratedController,
                  builder: (
                    BuildContext context,
                    MenuControllerDecorator decoratedController,
                    AnimationController animation,
                  ) {
                    return Menu(
                      controller: decoratedController,
                      consumeOutsideTaps: true,
                      menuPanel: const SizedBox(),
                      child: const SizedBox(),
                    );
                  },
                );
              },
            ),
          ),
        );

        expect(
          tester.takeException(),
          isInstanceOf<AssertionError>().having(
            (AssertionError e) => e.message,
            'message',
            contains(
              'A $MenuControllerDecorator cannot be used as the $MenuController '
              'for another $MenuControllerDecorator. Use a $MenuController instead of a '
              '$MenuControllerDecorator.',
            ),
          ),
        );
      },
    );
  });
}

// ********* UTILITIES *********  //
/// Allows the creation of arbitrarily-nested tags in tests.
abstract class Tag {
  const Tag();

  static const NestedTag anchor = NestedTag('anchor');
  static const NestedTag outside = NestedTag('outside');
  static const NestedTag a = NestedTag('a');
  static const NestedTag b = NestedTag('b');
  static const NestedTag c = NestedTag('c');
  static const NestedTag d = NestedTag('d');

  String get text;
  String get focusNode;
  int get level;

  @override
  String toString() {
    return 'Tag($text, level: $level)';
  }
}

class NestedTag extends Tag {
  const NestedTag(String name, {Tag? prefix, this.level = 0})
    : assert(
        // Limit the nesting level to prevent stack overflow.
        level < 9,
        'NestedTag.level must be less than 9 (was $level).',
      ),
      _name = name,
      _prefix = prefix;

  final String _name;
  final Tag? _prefix;

  @override
  final int level;

  NestedTag get a => NestedTag('a', prefix: this, level: level + 1);
  NestedTag get b => NestedTag('b', prefix: this, level: level + 1);
  NestedTag get c => NestedTag('c', prefix: this, level: level + 1);

  @override
  String get text {
    if (level == 0 || _prefix == null) {
      return _name;
    }
    return '${_prefix.text}.$_name';
  }

  @override
  String get focusNode {
    return 'Focus[$text]';
  }

  Key get key => ValueKey<String>('${text}_Key');
}

// A simple, focusable button that calls onPressed when tapped.
//
// The widgets library can't import the material library, so a separate button
// widget has to be created.
class Button extends StatefulWidget {
  const Button(
    this.child, {
    super.key,
    this.onPressed,
    this.focusNode,
    this.autofocus = false,
    this.onFocusChange,
    String? focusNodeLabel,
    BoxConstraints? constraints,
  }) : _focusNodeLabel = focusNodeLabel,
       constraints = constraints ?? const BoxConstraints.tightFor(width: 225, height: 32);

  factory Button.text(
    String text, {
    Key? key,
    VoidCallback? onPressed,
    FocusNode? focusNode,
    bool autofocus = false,
    BoxConstraints? constraints,
    void Function(bool)? onFocusChange,
  }) {
    return Button(
      Text(text),
      key: key,
      onPressed: onPressed,
      focusNode: focusNode,
      autofocus: autofocus,
      constraints: constraints,
      onFocusChange: onFocusChange,
    );
  }

  factory Button.tag(
    Tag tag, {
    Key? key,
    VoidCallback? onPressed,
    FocusNode? focusNode,
    bool autofocus = false,
    BoxConstraints? constraints,
    void Function(bool)? onFocusChange,
  }) {
    return Button(
      Text(tag.text),
      key: key,
      onPressed: onPressed,
      focusNode: focusNode,
      autofocus: autofocus,
      constraints: constraints,
      onFocusChange: onFocusChange,
      focusNodeLabel: tag.focusNode,
    );
  }

  final Widget child;
  final VoidCallback? onPressed;
  final void Function(bool)? onFocusChange;
  final FocusNode? focusNode;
  final bool autofocus;
  final BoxConstraints? constraints;
  final String? _focusNodeLabel;

  @override
  State<Button> createState() => _ButtonState();
}

class _ButtonState extends State<Button> {
  late final Map<Type, Action<Intent>> _actions = <Type, Action<Intent>>{
    ActivateIntent: CallbackAction<ActivateIntent>(onInvoke: _activateOnIntent),
    ButtonActivateIntent: CallbackAction<ButtonActivateIntent>(onInvoke: _activateOnIntent),
  };
  FocusNode get _focusNode => widget.focusNode ?? _internalFocusNode!;
  FocusNode? _internalFocusNode;
  final WidgetStatesController _states = WidgetStatesController();
  ui.Brightness _brightness = ui.Brightness.light;

  @override
  void initState() {
    super.initState();
    if (widget.focusNode == null) {
      _internalFocusNode = FocusNode(debugLabel: widget._focusNodeLabel);
    }
    _states.addListener(() {
      setState(() {
        /* Rebuild on state changes. */
      });
    });
  }

  @override
  void didUpdateWidget(Button oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.focusNode != widget.focusNode) {
      if (widget.focusNode == null) {
        _internalFocusNode = FocusNode(debugLabel: widget._focusNodeLabel);
      } else {
        _internalFocusNode?.dispose();
        _internalFocusNode = null;
      }
    }
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _brightness = MediaQuery.maybePlatformBrightnessOf(context) ?? _brightness;
  }

  @override
  void dispose() {
    _internalFocusNode?.dispose();
    super.dispose();
  }

  void _activateOnIntent(Intent intent) {
    _handlePressed();
  }

  void _handlePressed() {
    widget.onPressed?.call();
    _states.update(WidgetState.pressed, true);
  }

  void _handleTapDown(TapDownDetails details) {
    _states.update(WidgetState.pressed, true);
  }

  void _handleFocusChange(bool value) {
    _states.update(WidgetState.focused, value);
    widget.onFocusChange?.call(value);
  }

  void _handleExit(PointerExitEvent event) {
    _states.update(WidgetState.hovered, false);
  }

  void _handleHover(PointerHoverEvent event) {
    _states.update(WidgetState.hovered, true);
  }

  void _handleTapUp(TapUpDetails details) {
    _states.update(WidgetState.pressed, false);
    _handlePressed.call();
  }

  void _handleTapCancel() {
    _states.update(WidgetState.pressed, false);
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTextStyle.merge(
      style: _textStyle,
      child: MergeSemantics(
        child: Semantics(
          button: true,
          child: Actions(
            actions: _actions,
            child: Focus(
              debugLabel: widget._focusNodeLabel,
              onFocusChange: _handleFocusChange,
              autofocus: widget.autofocus,
              focusNode: _focusNode,
              child: MouseRegion(
                onHover: _handleHover,
                onExit: _handleExit,
                child: GestureDetector(
                  onTapDown: _handleTapDown,
                  onTapCancel: _handleTapCancel,
                  onTapUp: _handleTapUp,
                  child: Container(
                    constraints: widget.constraints,
                    decoration: _decoration,
                    padding: const EdgeInsets.symmetric(horizontal: 12),
                    child: Align(alignment: AlignmentDirectional.centerStart, child: widget.child),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }

  // Used for visualizing tests.
  BoxDecoration? get _decoration {
    if (_states.value.contains(WidgetState.pressed)) {
      return const BoxDecoration(color: Color(0xFF007BFF));
    }
    if (_states.value.contains(WidgetState.focused)) {
      return switch (_brightness) {
        Brightness.dark => const BoxDecoration(color: Color(0x95007BFF)),
        Brightness.light => const BoxDecoration(color: Color(0x95007BFF)),
      };
    }
    if (_states.value.contains(WidgetState.hovered)) {
      return const BoxDecoration(color: Color(0x22BBBBBB));
    }
    return null;
  }

  TextStyle get _textStyle {
    if (_states.value.contains(WidgetState.pressed)) {
      return const TextStyle(color: Color.fromARGB(255, 255, 255, 255));
    }
    return switch (_brightness) {
      Brightness.dark => const TextStyle(color: Color(0xFFFFFFFF)),
      Brightness.light => const TextStyle(color: Color(0xFF000000)),
    };
  }
}

class Panel extends StatelessWidget {
  const Panel({
    super.key,
    this.clipBehavior = Clip.antiAlias,
    this.constraints,
    this.decoration = const BoxDecoration(color: Color(0xFFFFFFFF)),
    required this.children,
  });

  final Decoration? decoration;
  final Clip clipBehavior;
  final BoxConstraints? constraints;
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    final Widget body = SingleChildScrollView(
      clipBehavior: Clip.none,
      child: ListBody(children: children),
    );

    Widget child = IntrinsicWidth(
      child: Builder(
        builder: (BuildContext context) {
          return Container(clipBehavior: clipBehavior, decoration: decoration, child: body);
        },
      ),
    );

    if (constraints != null) {
      child = ConstrainedBox(constraints: constraints!, child: child);
    }

    // The menu's items can grow beyond the size of the overlay, but will be
    // clipped by the overlay's bounds.
    return UnconstrainedBox(
      clipBehavior: Clip.hardEdge,
      alignment: AlignmentDirectional.centerStart,
      constrainedAxis: Axis.vertical,
      child: child,
    );
  }
}

class Menu extends StatefulWidget {
  const Menu({
    super.key,
    this.menuPanel,
    this.controller,
    this.child,
    this.builder,
    this.focusNode,
    this.onOpen,
    this.onClose,
    this.useRootOverlay = false,
    this.consumeOutsideTaps = false,
    this.overlayBuilder,
  });
  final Widget? menuPanel;
  final Widget? child;
  final bool useRootOverlay;
  final VoidCallback? onOpen;
  final VoidCallback? onClose;
  final FocusNode? focusNode;
  final RawMenuAnchorChildBuilder? builder;
  final RawMenuAnchorOverlayBuilder? overlayBuilder;
  final MenuController? controller;
  final bool consumeOutsideTaps;

  @override
  State<Menu> createState() => _MenuState();
}

class _MenuState extends State<Menu> {
  MenuController? _controller;
  @override
  Widget build(BuildContext context) {
    return RawMenuAnchor(
      childFocusNode: widget.focusNode,
      controller: widget.controller ?? (_controller ??= MenuController()),
      onOpen: widget.onOpen,
      onClose: widget.onClose,
      consumeOutsideTaps: widget.consumeOutsideTaps,
      useRootOverlay: widget.useRootOverlay,
      builder: widget.builder,
      overlayBuilder:
          widget.overlayBuilder ??
          (BuildContext context, RawMenuOverlayInfo info) {
            return Positioned(
              top: info.anchorRect.bottom,
              left: info.anchorRect.left,
              child: widget.menuPanel!,
            );
          },
      child: widget.child,
    );
  }
}

class AnchorButton extends StatelessWidget {
  const AnchorButton(
    this.tag, {
    super.key,
    this.onPressed,
    this.constraints,
    this.autofocus = false,
    this.focusNode,
  });

  final Tag tag;
  final void Function(Tag)? onPressed;
  final bool autofocus;
  final BoxConstraints? constraints;
  final FocusNode? focusNode;

  @override
  Widget build(BuildContext context) {
    final MenuController? controller = MenuController.maybeOf(context);
    return Button.tag(
      tag,
      onPressed: () {
        onPressed?.call(tag);
        if (controller != null) {
          if (MenuController.maybeAnimationStatusOf(context)!.isForwardOrCompleted) {
            controller.close();
          } else {
            controller.open();
          }
        }
      },
      focusNode: focusNode,
      constraints: constraints,
      autofocus: autofocus,
    );
  }
}

class AnimatedMenuController extends MenuControllerDecorator {
  const AnimatedMenuController({
    required super.menuController,
    required this.animationController,
    required this.onOpenRequest,
  });
  final AnimationController animationController;
  final void Function(Offset?)? onOpenRequest;

  @override
  void handleMenuOpenRequest({ui.Offset? position}) {
    onOpenRequest?.call(position);
    animationController.forward().whenComplete(markMenuOpened);
  }

  @override
  void handleMenuCloseRequest() {
    animationController.reverse().whenComplete(markMenuClosed);
  }

  @override
  void markMenuOpened() {
    super.markMenuOpened();
    animationController.value = 1.0;
  }

  @override
  void markMenuClosed() {
    super.markMenuClosed();
    animationController.value = 0.0;
  }
}

class DecoratedMenu extends StatefulWidget {
  const DecoratedMenu({
    super.key,
    required this.controller,
    required this.builder,
    this.onOpenRequest,
  });

  final MenuController controller;
  final Widget Function(BuildContext, MenuControllerDecorator, AnimationController) builder;
  final void Function(Offset? position)? onOpenRequest;

  @override
  State<DecoratedMenu> createState() => _DecoratedMenuState();
}

class _DecoratedMenuState extends State<DecoratedMenu> with SingleTickerProviderStateMixin {
  late final AnimationController animationController;
  late MenuControllerDecorator decoratedController;

  @override
  void initState() {
    super.initState();
    animationController = AnimationController(
      duration: const Duration(milliseconds: 200),
      vsync: this,
    );
    decoratedController = AnimatedMenuController(
      menuController: widget.controller,
      animationController: animationController,
      onOpenRequest: widget.onOpenRequest,
    );
  }

  @override
  void didUpdateWidget(covariant DecoratedMenu oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.controller != oldWidget.controller) {
      decoratedController = AnimatedMenuController(
        menuController: widget.controller,
        animationController: animationController,
        onOpenRequest: widget.onOpenRequest,
      );
    }
  }

  @override
  void dispose() {
    animationController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return widget.builder.call(context, decoratedController, animationController);
  }
}

class App extends StatefulWidget {
  const App(this.child, {super.key, this.textDirection, this.alignment = Alignment.center});
  final Widget child;
  final TextDirection? textDirection;
  final AlignmentGeometry alignment;

  @override
  State<App> createState() => _AppState();
}

class _AppState extends State<App> {
  TextDirection? _directionality;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _directionality = Directionality.maybeOf(context);
  }

  @override
  Widget build(BuildContext context) {
    return ColoredBox(
      color: const Color(0xff000000),
      child: WidgetsApp(
        color: const Color(0xff000000),
        onGenerateRoute: (RouteSettings settings) {
          return PageRouteBuilder<void>(settings: settings, pageBuilder: _buildPage);
        },
      ),
    );
  }

  Widget _buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return Directionality(
      textDirection: widget.textDirection ?? _directionality ?? TextDirection.ltr,
      child: Align(alignment: widget.alignment, child: widget.child),
    );
  }
}
