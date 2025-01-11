// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

// Tests that apply to select constructors have a suffix that indicates which
// constructor the test applies to:
//  * [Default]: Applies to [RawMenuAnchor],
//  * [OverlayBuilder]: Applies to [RawMenuAnchor.overlayBuilder]
//  * [Node]: Applies to [RawMenuAnchor.menuPanel].
//  * [Overlays]: Applies to [RawMenuAnchor] and [RawMenuAnchor.overlayBuilder]
// Otherwise, the test applies to all constructors.

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

  Finder findOverlayContents() {
    return find.descendant(of: find.byType(RawMenuPanel), matching: find.byType(IntrinsicWidth));
  }

  T findMenuPanelDescendent<T extends Widget>(WidgetTester tester) {
    return tester.firstWidget<T>(
      find.descendant(of: find.byType(RawMenuPanel), matching: find.byType(T)),
    );
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

  testWidgets("[Overlays] MenuController.isOpen is true when a menu's overlay is shown", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.text)]),
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

  testWidgets('[Overlays] MenuController.open() and .close() toggle overlay visibility', (
    WidgetTester tester,
  ) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          panel: RawMenuPanel(
            menuChildren: <Widget>[
              Text(Tag.a.text),
              RawMenuAnchor(
                controller: nestedController,
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets('[Overlays] MenuController.closeChildren closes submenu children', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          panel: RawMenuPanel(
            menuChildren: <Widget>[
              Text(Tag.a.text),
              RawMenuAnchor(
                childFocusNode: focusNode,
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets('[Overlays] Can only have one open child anchor', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor.overlayBuilder(
          overlayBuilder: (BuildContext context, RawMenuAnchorOverlayPosition position) {
            return Column(
              children: <Widget>[
                RawMenuAnchor(
                  panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.a.text)]),
                  child: const AnchorButton(Tag.a),
                ),
                RawMenuAnchor(
                  panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets('[Overlays] Context menus can be nested', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          menuAlignment: AlignmentDirectional.bottomStart,
          alignment: AlignmentDirectional.topStart,
          panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a.a)]),
          builder: (BuildContext context, MenuController controller, Widget? child) {
            return Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                const AnchorButton(Tag.a),
                RawMenuAnchor(
                  alignment: AlignmentDirectional.bottomStart,
                  menuAlignment: AlignmentDirectional.topStart,
                  panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.b.a)]),
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

  testWidgets('[Node] MenuController.isOpen is true when a descendent menu is open', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          controller: controller,
          child: Row(
            children: <Widget>[
              RawMenuAnchor(
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.a.text)]),
                child: const AnchorButton(Tag.a),
              ),
              // Menu should not need to be a direct descendent.
              Padding(
                padding: const EdgeInsets.all(8.0),
                child: RawMenuAnchor(
                  panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets('[Node] MenuController.open does nothing', (WidgetTester tester) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          controller: controller,
          child: Column(
            children: <Widget>[
              RawMenuAnchor(
                controller: nestedController,
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets('[Node] MenuController.close closes children', (WidgetTester tester) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          controller: controller,
          child: Column(
            children: <Widget>[
              RawMenuAnchor(
                controller: nestedController,
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets('[Node] MenuController.closeChildren closes children', (WidgetTester tester) async {
    final MenuController nestedController = MenuController();
    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          controller: controller,
          child: Column(
            children: <Widget>[
              RawMenuAnchor(
                controller: nestedController,
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets('[Node] Should only display one open child anchor at a time', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          child: Row(
            children: <Widget>[
              RawMenuAnchor(
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.a.text)]),
                child: const AnchorButton(Tag.a),
              ),
              RawMenuAnchor(
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.a.text)]),
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

  testWidgets(
    'MenuController.maybeOf notifies dependents on open and close when createDependency is true',
    (WidgetTester tester) async {
      final MenuController controller = MenuController();
      final MenuController nestedController = MenuController();
      MenuController? panelController;
      MenuController? overlayController;
      MenuController? anchorController;
      int panelBuilds = 0;
      int anchorBuilds = 0;
      int overlayBuilds = 0;

      await tester.pumpWidget(
        App(
          RawMenuAnchor.node(
            child: Column(
              children: <Widget>[
                // Panel context.
                Builder(
                  builder: (BuildContext context) {
                    panelController = MenuController.maybeOf(context, createDependency: true);
                    panelBuilds += 1;
                    return Text(Tag.a.text);
                  },
                ),
                RawMenuAnchor(
                  controller: controller,
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[
                      // Overlay context.
                      Builder(
                        builder: (BuildContext context) {
                          overlayController = MenuController.maybeOf(
                            context,
                            createDependency: true,
                          );
                          overlayBuilds += 1;
                          return Text(Tag.b.a.a.text);
                        },
                      ),
                      RawMenuAnchor(
                        controller: nestedController,
                        panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.b.a.b.a)]),
                        child: Button.tag(Tag.b.a.b),
                      ),
                    ],
                  ),
                  // Anchor context.
                  child: Builder(
                    builder: (BuildContext context) {
                      anchorController = MenuController.maybeOf(context, createDependency: true);
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

      expect(panelController!.isOpen, isFalse);
      expect(anchorController!.isOpen, isFalse);
      expect(panelBuilds, equals(1));
      expect(anchorBuilds, equals(1));
      expect(overlayBuilds, equals(0));

      controller.open();
      await tester.pump();

      expect(panelController!.isOpen, isTrue);
      expect(anchorController!.isOpen, isTrue);
      expect(overlayController!.isOpen, isTrue);
      expect(panelBuilds, equals(2));
      expect(anchorBuilds, equals(2));
      expect(overlayBuilds, equals(1));

      nestedController.open();
      await tester.pump();

      // No new builds should have occurred since all controllers are already open.
      expect(panelController!.isOpen, isTrue);
      expect(anchorController!.isOpen, isTrue);
      expect(overlayController!.isOpen, isTrue);
      expect(panelBuilds, equals(2));
      expect(anchorBuilds, equals(2));
      expect(overlayBuilds, equals(1));

      controller.close();
      await tester.pump();

      expect(panelController!.isOpen, isFalse);
      expect(anchorController!.isOpen, isFalse);
      expect(overlayController!.isOpen, isFalse);
      expect(panelBuilds, equals(3));
      expect(anchorBuilds, equals(3));
      expect(overlayBuilds, equals(1));
    },
  );

  testWidgets('MenuController.maybeOf notifies dependents when set if createDependency is true', (
    WidgetTester tester,
  ) async {
    final GlobalKey anchorKey = GlobalKey();
    MenuController? panelController;
    MenuController? overlayController;
    MenuController? anchorController;
    int panelBuilds = 0;
    int anchorBuilds = 0;
    int overlayBuilds = 0;

    Widget buildAnchor({MenuController? panel, MenuController? overlay}) {
      return App(
        RawMenuAnchor.node(
          controller: panel,
          child: Column(
            children: <Widget>[
              // Panel context.
              Builder(
                builder: (BuildContext context) {
                  panelController = MenuController.maybeOf(context, createDependency: true);
                  panelBuilds += 1;
                  return Text(Tag.a.text);
                },
              ),
              RawMenuAnchor(
                controller: overlay,
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    // Overlay context.
                    Builder(
                      builder: (BuildContext context) {
                        overlayController = MenuController.maybeOf(context, createDependency: true);
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
                    anchorController = MenuController.maybeOf(context, createDependency: true);
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

    await tester.pumpWidget(buildAnchor());

    expect(panelController, isNot(controller));
    expect(anchorController, isNot(controller));

    await tester.pumpWidget(buildAnchor(panel: controller));

    expect(panelController, equals(controller));
    expect(anchorController, isNot(controller));
    expect(panelBuilds, equals(2));
    expect(anchorBuilds, equals(2));

    MenuController.maybeOf(anchorKey.currentContext!)?.open();
    await tester.pump();

    expect(panelBuilds, equals(3));
    expect(anchorBuilds, equals(3));
    expect(overlayBuilds, equals(1));

    await tester.pumpWidget(buildAnchor(overlay: controller));

    expect(panelController, isNot(controller));
    expect(anchorController, equals(controller));
    expect(overlayController, equals(controller));
    expect(panelBuilds, equals(4));
    expect(anchorBuilds, equals(4));
    expect(overlayBuilds, equals(2));
  });

  testWidgets(
    'MenuController.maybeOf does not notify dependents on open and close if createDependency is false',
    (WidgetTester tester) async {
      final MenuController controller = MenuController();
      final MenuController nestedController = MenuController();
      MenuController? panelController;
      MenuController? overlayController;
      MenuController? anchorController;
      int panelBuilds = 0;
      int anchorBuilds = 0;
      int overlayBuilds = 0;

      await tester.pumpWidget(
        App(
          RawMenuAnchor.node(
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
                RawMenuAnchor(
                  controller: controller,
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[
                      // Overlay context.
                      Builder(
                        builder: (BuildContext context) {
                          overlayController = MenuController.maybeOf(context);
                          overlayBuilds += 1;
                          return Text(Tag.b.a.a.text);
                        },
                      ),
                      RawMenuAnchor(
                        controller: nestedController,
                        panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.b.a.b.a)]),
                        child: Button.tag(Tag.b.a.b),
                      ),
                    ],
                  ),
                  // Anchor context.
                  child: Builder(
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
        ),
      );

      expect(panelController!.isOpen, isFalse);
      expect(anchorController!.isOpen, isFalse);
      expect(panelBuilds, equals(1));
      expect(anchorBuilds, equals(1));
      expect(overlayBuilds, equals(0));

      controller.open();
      await tester.pump();

      expect(panelController!.isOpen, isTrue);
      expect(anchorController!.isOpen, isTrue);
      expect(overlayController!.isOpen, isTrue);
      expect(panelBuilds, equals(1));
      expect(anchorBuilds, equals(1));
      expect(overlayBuilds, equals(1));

      nestedController.open();
      await tester.pump();

      // No new builds should have occurred since all controllers are already open.
      expect(panelController!.isOpen, isTrue);
      expect(anchorController!.isOpen, isTrue);
      expect(overlayController!.isOpen, isTrue);
      expect(panelBuilds, equals(1));
      expect(anchorBuilds, equals(1));
      expect(overlayBuilds, equals(1));

      controller.close();
      await tester.pump();

      expect(panelController!.isOpen, isFalse);
      expect(anchorController!.isOpen, isFalse);
      expect(overlayController!.isOpen, isFalse);
      expect(panelBuilds, equals(1));
      expect(anchorBuilds, equals(1));
      expect(overlayBuilds, equals(1));
    },
  );

  testWidgets(
    'MenuController.maybeOf does not notify dependents when set if createDependency is false',
    (WidgetTester tester) async {
      final GlobalKey anchorKey = GlobalKey();
      MenuController? panelController;
      MenuController? overlayController;
      MenuController? anchorController;
      int panelBuilds = 0;
      int anchorBuilds = 0;
      int overlayBuilds = 0;

      Widget buildAnchor({MenuController? panel, MenuController? overlay}) {
        return App(
          RawMenuAnchor.node(
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
                RawMenuAnchor(
                  controller: overlay,
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[
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

      await tester.pumpWidget(buildAnchor());

      expect(panelController, isNot(controller));
      expect(anchorController, isNot(controller));

      await tester.pumpWidget(buildAnchor(panel: controller));

      expect(panelController, equals(controller));
      expect(anchorController, isNot(controller));
      expect(panelBuilds, equals(2));
      expect(anchorBuilds, equals(2));

      MenuController.maybeOf(anchorKey.currentContext!)?.open();
      await tester.pump();

      expect(panelBuilds, equals(2));
      expect(anchorBuilds, equals(2));
      expect(overlayBuilds, equals(1));

      await tester.pumpWidget(buildAnchor(overlay: controller));

      expect(panelController, isNot(controller));
      expect(anchorController, equals(controller));
      expect(overlayController, equals(controller));
      expect(panelBuilds, equals(3));
      expect(anchorBuilds, equals(3));
      expect(overlayBuilds, equals(2));
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/156572.
  testWidgets('Unattached MenuController does not throw when calling close', (
    WidgetTester tester,
  ) async {
    final MenuController controller = MenuController();
    controller.close();
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('Unattached MenuController returns false when calling isOpen', (
    WidgetTester tester,
  ) async {
    final MenuController controller = MenuController();
    expect(controller.isOpen, false);
  });

  testWidgets('[Overlays] RawMenuAnchorOverlayPosition.anchorRect applies transformations', (
    WidgetTester tester,
  ) async {
    RawMenuAnchorOverlayPosition? builderPosition;
    final GlobalKey anchorKey = GlobalKey();
    await tester.pumpWidget(
      App(
        Transform(
          transform: Matrix4.translationValues(-50, 50, 0)..scale(1.2),
          child: RawMenuAnchor.overlayBuilder(
            controller: controller,
            overlayBuilder: (BuildContext context, RawMenuAnchorOverlayPosition position) {
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

  testWidgets('[Default] Previous focus is restored on menu close', (WidgetTester tester) async {
    final FocusNode externalFocus = FocusNode();
    final FocusNode aaaFocusNode = FocusNode();
    addTearDown(aaaFocusNode.dispose);
    addTearDown(externalFocus.dispose);

    await tester.pumpWidget(
      App(
        Column(
          children: <Widget>[
            RawMenuAnchor.node(
              controller: controller,
              child: Row(
                children: <Widget>[
                  RawMenuAnchor(
                    panel: RawMenuPanel(
                      menuChildren: <Widget>[
                        RawMenuAnchor(
                          panel: RawMenuPanel(
                            menuChildren: <Widget>[Button.tag(Tag.a.a.a, focusNode: aaaFocusNode)],
                          ),
                          child: AnchorButton(Tag.a.a),
                        ),
                      ],
                    ),
                    child: const AnchorButton(Tag.a),
                  ),
                ],
              ),
            ),
            Button.tag(Tag.b, autofocus: true, focusNode: externalFocus),
          ],
        ),
      ),
    );

    await tester.pump();

    expect(FocusManager.instance.primaryFocus, equals(externalFocus));

    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    await tester.tap(find.text(Tag.a.a.text));
    await tester.pump();

    aaaFocusNode.requestFocus();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, isNot(externalFocus));

    controller.close();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, equals(externalFocus));
  });

  testWidgets('[Default] Previous focus is restored on submenu close', (WidgetTester tester) async {
    final FocusNode acaFocusNode = FocusNode();
    final FocusNode buttonFocus = FocusNode();
    addTearDown(acaFocusNode.dispose);
    addTearDown(buttonFocus.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          child: Row(
            children: <Widget>[
              RawMenuAnchor(
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    Button.tag(Tag.a.a, focusNode: buttonFocus),
                    Button.tag(Tag.a.b),
                    RawMenuAnchor(
                      controller: controller,
                      panel: RawMenuPanel(
                        menuChildren: <Widget>[Button.tag(Tag.a.c.a, focusNode: acaFocusNode)],
                      ),
                      child: AnchorButton(Tag.a.c),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.a),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text(Tag.a.text));
    await tester.pump();

    buttonFocus.requestFocus();
    await tester.pump();

    await tester.tap(find.text(Tag.a.c.text));
    await tester.pump();

    acaFocusNode.requestFocus();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, isNot(buttonFocus));

    controller.close();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, equals(buttonFocus));
  });

  testWidgets('Escape key closes menus', (WidgetTester tester) async {
    final FocusNode aFocusNode = FocusNode();
    final FocusNode baaFocusNode = FocusNode();
    addTearDown(aFocusNode.dispose);
    addTearDown(baaFocusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          child: Row(
            children: <Widget>[
              Button.tag(Tag.a, focusNode: aFocusNode),
              RawMenuAnchor(
                controller: controller,
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    RawMenuAnchor(
                      panel: RawMenuPanel(
                        menuChildren: <Widget>[Button.tag(Tag.b.a.a, focusNode: baaFocusNode)],
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

    controller.open();
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
          child: RawMenuAnchor.node(
            child: Row(
              children: <Widget>[
                RawMenuAnchor(
                  panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.text)]),
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

  testWidgets('[OverlayBuilder] Focus traversal shortcuts are not bound to actions', (
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
              child: RawMenuAnchor.overlayBuilder(
                controller: controller,
                overlayBuilder: (BuildContext context, RawMenuAnchorOverlayPosition position) {
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

  testWidgets('Actions that wrap RawMenuAnchor are invoked by both anchor and overlay', (
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
          child: RawMenuAnchor(
            childFocusNode: anchorFocusNode,
            panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a, focusNode: aFocusNode)]),
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
        RawMenuAnchor(
          panel: RawMenuPanel(
            menuChildren: <Widget>[
              Text(Tag.a.text),
              RawMenuAnchor(
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    Text(Tag.b.a.text),
                    RawMenuAnchor(
                      controller: controller,
                      panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.b.b.a.text)]),
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

  testWidgets('[Node] Menu panel builder', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        alignment: AlignmentDirectional.topStart,
        RawMenuAnchor.node(
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

  testWidgets('[OverlayBuilder] Overlay builder is passed anchor rect', (
    WidgetTester tester,
  ) async {
    RawMenuAnchorOverlayPosition? overlayPosition;
    await tester.pumpWidget(
      App(
        RawMenuAnchor.overlayBuilder(
          overlayBuilder: (BuildContext context, RawMenuAnchorOverlayPosition position) {
            overlayPosition = position;
            return const SizedBox();
          },
          child: AnchorButton(Tag.anchor, onPressed: onPressed),
        ),
      ),
    );

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(overlayPosition!.anchorRect, tester.getRect(find.byType(Button)));
  });

  testWidgets('[OverlayBuilder] Overlay contents can be positioned', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor.overlayBuilder(
          overlayBuilder: (BuildContext context, RawMenuAnchorOverlayPosition position) {
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

  testWidgets('[OverlayBuilder] TapRegion group ID is passed to overlay', (
    WidgetTester tester,
  ) async {
    bool? insideTap;

    await tester.pumpWidget(
      App(
        RawMenuAnchor.overlayBuilder(
          overlayBuilder: (BuildContext context, RawMenuAnchorOverlayPosition position) {
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
            RawMenuAnchor.node(
              child: Column(
                children: <Widget>[
                  RawMenuAnchor(
                    consumeOutsideTaps: true,
                    onOpen: () => onOpen(Tag.anchor),
                    onClose: () => onClose(Tag.anchor),
                    panel: RawMenuPanel(
                      menuChildren: <Widget>[
                        RawMenuAnchor(
                          consumeOutsideTaps: true,
                          onOpen: () => onOpen(Tag.a),
                          onClose: () => onClose(Tag.a),
                          panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.a.text)]),
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

  testWidgets('[Overlays] Menus close and do not consume tap when consumesOutsideTap is false', (
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
            RawMenuAnchor.node(
              child: Column(
                children: <Widget>[
                  RawMenuAnchor(
                    onOpen: () => onOpen(Tag.anchor),
                    onClose: () => onClose(Tag.anchor),
                    // ignore: avoid_redundant_argument_values
                    consumeOutsideTaps: false,
                    panel: RawMenuPanel(
                      menuChildren: <Widget>[
                        RawMenuAnchor(
                          onOpen: () => onOpen(Tag.a),
                          onClose: () => onClose(Tag.a),
                          panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.a.text)]),
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
        RawMenuAnchor(
          controller: controller,
          onOpen: () {
            opened = true;
          },
          panel: const RawMenuPanel(menuChildren: <Widget>[]),
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
        RawMenuAnchor(
          controller: controller,
          onOpen: () {
            closed = false;
          },
          onClose: () {
            closed = true;
          },
          panel: const RawMenuPanel(menuChildren: <Widget>[]),
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

  testWidgets('panel diagnostics', (WidgetTester tester) async {
    const RawMenuPanel panel = RawMenuPanel(
      menuChildren: <Widget>[Text('1')],
      padding: EdgeInsetsDirectional.all(5),
      constrainCrossAxis: true,
      clipBehavior: Clip.hardEdge,
    );

    await tester.pumpWidget(const App(panel));
    await tester.pump();

    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    panel.debugFillProperties(builder);
    final List<String> properties =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(properties, const <String>[
      'clipBehavior: Clip.hardEdge',
      'constrainCrossAxis: true',
      'padding override: EdgeInsetsDirectional(5.0, 5.0, 5.0, 5.0)',
    ]);
  });

  testWidgets('[Default] diagnostics', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final RawMenuAnchor menuAnchor = RawMenuAnchor(
      controller: controller,
      childFocusNode: focusNode,
      alignmentOffset: const Offset(10, 10),
      panel: const Text('PANEL'),
      child: const Text('BUTTON'),
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

    expect(properties, const <String>[
      'has controller',
      'has focusNode',
      'alignmentOffset: Offset(10.0, 10.0)',
    ]);
  });

  testWidgets('[Node] diagnostics', (WidgetTester tester) async {
    final RawMenuAnchor menuNode = RawMenuAnchor.node(
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
        RawMenuAnchor(
          controller: controller,
          panel: const RawMenuPanel(menuChildren: <Widget>[Text('Button 1')]),
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
        RawMenuAnchor(
          controller: controller,
          panel: const RawMenuPanel(
            menuChildren: <Widget>[Text('Button 1')],
            clipBehavior: Clip.hardEdge,
          ),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    // Test custom clip behavior.
    expect(findMenuPanelDescendent<Container>(tester).clipBehavior, equals(Clip.hardEdge));
  });

  testWidgets('[Default] Home key from a menu item focuses first sibling', (
    WidgetTester tester,
  ) async {
    const BoxConstraints anchorConstraints = BoxConstraints.tightFor(height: 225);
    final FocusNode anchorFocusNode = FocusNode(debugLabel: Tag.anchor.focusNode);
    final FocusNode focusNode = FocusNode(debugLabel: Tag.d.c.focusNode);
    addTearDown(anchorFocusNode.dispose);
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          childFocusNode: anchorFocusNode,
          panel: RawMenuPanel(
            constraints: const BoxConstraints(maxHeight: 500),
            menuChildren: <Widget>[
              Button.tag(Tag.a),
              Button.tag(Tag.b, constraints: anchorConstraints),
              Button.tag(Tag.c, constraints: anchorConstraints),
              RawMenuAnchor(
                controller: controller,
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    Button.tag(Tag.d.a),
                    Button.tag(Tag.d.b),
                    Button.tag(Tag.d.c, focusNode: focusNode),
                  ],
                ),
                child: const AnchorButton(Tag.d, constraints: anchorConstraints),
              ),
            ],
          ),
          child: AnchorButton(Tag.anchor, focusNode: anchorFocusNode),
        ),
      ),
    );

    listenForFocusChanges();

    // Have to focus a menu item to get things started.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(focusedMenu, equals(Tag.anchor.focusNode));

    // Test that root anchor is not affected by home key.
    await tester.sendKeyEvent(LogicalKeyboardKey.home);

    expect(focusedMenu, equals(Tag.anchor.focusNode));

    // Test from adjacent menu item sibling.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);

    expect(focusedMenu, Tag.c.focusNode);

    await tester.sendKeyEvent(LogicalKeyboardKey.home);

    expect(focusedMenu, equals(Tag.a.focusNode));

    // Move to the nested anchor and open it.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();

    // Test from opened anchor sibling to check that the event doesn't affect
    // the attached submenu. The menu should scroll to the first menu item.
    expect(focusedMenu, Tag.d.focusNode);
    expect(find.text(Tag.d.a.text), findsOneWidget);
    expect(find.text(Tag.a.text).hitTestable(), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.home);

    expect(focusedMenu, equals(Tag.a.focusNode));
    expect(find.text(Tag.a.text).hitTestable(), findsOneWidget);

    // Test from nested overlay.
    await tester.ensureVisible(find.text(Tag.d.text));
    await tester.tap(find.text(Tag.d.text));
    await tester.pump();
    focusNode.requestFocus();
    await tester.pump();

    expect(find.text(Tag.d.c.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.d.c.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.home);

    expect(focusedMenu, equals(Tag.d.a.focusNode));
  });

  testWidgets('[Default] End key from a menu item focuses last sibling', (
    WidgetTester tester,
  ) async {
    const BoxConstraints anchorConstraints = BoxConstraints.tightFor(height: 200, width: 225);
    final FocusNode anchorFocusNode = FocusNode(debugLabel: Tag.anchor.focusNode);
    final FocusNode bFocusNode = FocusNode(debugLabel: Tag.b.focusNode);
    final FocusNode baFocusNode = FocusNode(debugLabel: Tag.b.a.focusNode);

    addTearDown(anchorFocusNode.dispose);
    addTearDown(bFocusNode.dispose);
    addTearDown(baFocusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          childFocusNode: anchorFocusNode,
          controller: controller,
          panel: RawMenuPanel(
            constraints: const BoxConstraints(maxHeight: 500),
            menuChildren: <Widget>[
              Button.tag(Tag.a, constraints: anchorConstraints),
              RawMenuAnchor(
                childFocusNode: bFocusNode,
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    Button.tag(Tag.b.a, focusNode: baFocusNode),
                    Button.tag(Tag.b.b),
                    Button.tag(Tag.b.c),
                  ],
                ),
                child: AnchorButton(Tag.b, focusNode: bFocusNode, constraints: anchorConstraints),
              ),
              Button.tag(Tag.c, constraints: anchorConstraints),
              Button.tag(Tag.d, constraints: anchorConstraints),
            ],
          ),
          child: AnchorButton(Tag.anchor, focusNode: anchorFocusNode),
        ),
      ),
    );

    listenForFocusChanges();

    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    expect(focusedMenu, equals(Tag.anchor.focusNode));

    // Test that root anchor is not affected by end key.
    await tester.sendKeyEvent(LogicalKeyboardKey.end);

    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);

    expect(focusedMenu, Tag.a.focusNode);
    expect(find.text(Tag.d.text).hitTestable(), findsNothing);

    // Test from menu item sibling.
    await tester.sendKeyEvent(LogicalKeyboardKey.end);

    expect(focusedMenu, equals(Tag.d.focusNode));
    expect(find.text(Tag.d.text).hitTestable(), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();

    expect(focusedMenu, Tag.b.focusNode);
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Test from opened anchor sibling to check that the event doesn't affect
    // attached submenu.
    await tester.sendKeyEvent(LogicalKeyboardKey.end);
    await tester.pump();

    expect(focusedMenu, equals(Tag.d.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.tap(find.text(Tag.b.text));
    await tester.pump();
    baFocusNode.requestFocus();
    await tester.pump();

    expect(find.text(Tag.b.a.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.b.a.focusNode));

    // Test from nested overlay.
    await tester.sendKeyEvent(LogicalKeyboardKey.end);

    expect(focusedMenu, equals(Tag.b.c.focusNode));
  });

  testWidgets('[Default] ArrowDown key from open root anchor focuses first menu item', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: Tag.anchor.focusNode);
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          childFocusNode: focusNode,
          panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a), Button.tag(Tag.b)]),
          child: AnchorButton(Tag.anchor, focusNode: focusNode),
        ),
      ),
    );

    listenForFocusChanges();

    controller.open();
    await tester.pump();

    expect(find.text(Tag.b.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    expect(focusedMenu, equals(Tag.a.focusNode));

    // Test that the action still works in a menu panel.
    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          child: Column(
            children: <Widget>[
              RawMenuAnchor(
                controller: controller,
                childFocusNode: focusNode,
                panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a), Button.tag(Tag.b)]),
                child: AnchorButton(Tag.anchor, focusNode: focusNode),
              ),
            ],
          ),
        ),
      ),
    );

    controller.open();
    await tester.pump();

    expect(find.text(Tag.b.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    expect(focusedMenu, equals(Tag.a.focusNode));
  });

  testWidgets('[Default] ArrowUp key from open root anchor focuses last menu item', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: Tag.anchor.focusNode);
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          childFocusNode: focusNode,
          panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a), Button.tag(Tag.b)]),
          child: AnchorButton(Tag.anchor, focusNode: focusNode),
        ),
      ),
    );

    listenForFocusChanges();

    controller.open();
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));

    // Test that the action works in a menu panel.
    await tester.pumpWidget(
      App(
        RawMenuAnchor.node(
          child: Column(
            children: <Widget>[
              RawMenuAnchor(
                controller: controller,
                childFocusNode: focusNode,
                panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a), Button.tag(Tag.b)]),
                child: AnchorButton(Tag.anchor, focusNode: focusNode),
              ),
            ],
          ),
        ),
      ),
    );

    controller.open();
    await tester.pump();

    expect(find.text(Tag.a.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.anchor.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));
  });

  testWidgets('[Default] LTR ArrowRight key opens a submenu anchor and focuses first item', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          panel: RawMenuPanel(
            menuChildren: <Widget>[
              Button.tag(Tag.a),
              Button.tag(Tag.b),
              RawMenuAnchor(
                panel: RawMenuPanel(
                  menuChildren: <Widget>[Button.tag(Tag.c.a), Button.tag(Tag.c.b)],
                ),
                child: AnchorButton(Tag.c, focusNode: focusNode),
              ),
            ],
          ),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    listenForFocusChanges();

    // Have to open a menu initially to start things going.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    focusNode.requestFocus();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, equals(focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(find.text(Tag.c.a.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.c.a.focusNode));
  });

  testWidgets('[Default] RTL ArrowLeft key opens a submenu anchor and focuses first item', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        textDirection: TextDirection.rtl,
        RawMenuAnchor(
          panel: RawMenuPanel(
            menuChildren: <Widget>[
              Button.tag(Tag.a),
              Button.tag(Tag.b),
              RawMenuAnchor(
                panel: RawMenuPanel(
                  menuChildren: <Widget>[Button.tag(Tag.c.a), Button.tag(Tag.c.b)],
                ),
                child: AnchorButton(Tag.c, focusNode: focusNode),
              ),
            ],
          ),
          child: const AnchorButton(Tag.anchor),
        ),
      ),
    );

    listenForFocusChanges();

    // Have to open a menu initially to start things going.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    focusNode.requestFocus();
    await tester.pump();

    expect(FocusManager.instance.primaryFocus, equals(focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(find.text(Tag.c.a.text), findsOneWidget);
    expect(focusedMenu, equals(Tag.c.a.focusNode));
  });

  testWidgets('[Default] LTR ArrowLeft key closes a submenu', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          panel: RawMenuPanel(
            menuChildren: <Widget>[
              Button.tag(Tag.a),
              Button.tag(Tag.b),
              RawMenuAnchor(
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    Button.tag(Tag.c.a),
                    Button.tag(Tag.c.b),
                    RawMenuAnchor(
                      panel: const RawMenuPanel(menuChildren: <Widget>[]),
                      child: AnchorButton(Tag.c.c),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.c),
              ),
            ],
          ),
          child: AnchorButton(Tag.anchor, focusNode: focusNode),
        ),
      ),
    );

    listenForFocusChanges();

    // Move into submenu.
    focusNode.requestFocus();
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.a.focusNode));

    // Arrow left from regular item should close the submenu and refocus its
    // anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    // ArrowLeft from submenu anchor should close the submenu and refocus its
    // anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    // ArrowLeft from root overlay anchor and root overlay button should do
    // nothing.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);
  });

  testWidgets('[Default] RTL ArrowRight key closes a submenu', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        textDirection: TextDirection.rtl,
        RawMenuAnchor(
          panel: RawMenuPanel(
            menuChildren: <Widget>[
              Button.tag(Tag.a),
              Button.tag(Tag.b),
              RawMenuAnchor(
                panel: RawMenuPanel(
                  menuChildren: <Widget>[
                    Button.tag(Tag.c.a),
                    Button.tag(Tag.c.b),
                    RawMenuAnchor(
                      panel: const RawMenuPanel(menuChildren: <Widget>[]),
                      child: AnchorButton(Tag.c.c),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.c),
              ),
            ],
          ),
          child: AnchorButton(Tag.anchor, focusNode: focusNode),
        ),
      ),
    );

    listenForFocusChanges();

    // Move into submenu.
    focusNode.requestFocus();
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.a.focusNode));

    // Arrow left from regular item should close the submenu and refocus its
    // anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    // arrowRight from submenu anchor should close the submenu and refocus its
    // anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    // arrowRight from root overlay anchor and root overlay button should do
    // nothing.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);
  });

  testWidgets('[Default] LTR Directional traversal', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    bool eventBubbled = false;
    await tester.pumpWidget(
      App(
        Actions(
          actions: <Type, Action<Intent>>{
            // Intents should not bubble up to the root anchor.
            DirectionalFocusIntent: CallbackAction<DirectionalFocusIntent>(
              onInvoke: (DirectionalFocusIntent intent) {
                eventBubbled = true;
                return null;
              },
            ),
          },
          child: RawMenuAnchor(
            childFocusNode: focusNode,
            panel: RawMenuPanel(
              menuChildren: <Widget>[
                Button.tag(Tag.a),
                Button.tag(Tag.b),
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[Button.tag(Tag.c.a), Button.tag(Tag.c.b)],
                  ),
                  child: const AnchorButton(Tag.c),
                ),
                Button.tag(Tag.d),
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[
                      Button.tag(Tag.e.a),
                      Button.tag(Tag.e.b),
                      RawMenuAnchor(
                        panel: RawMenuPanel(
                          menuChildren: <Widget>[
                            Button.tag(Tag.e.c.a),
                            Button.tag(Tag.e.c.b),
                            Button.tag(Tag.e.c.c),
                          ],
                        ),
                        child: AnchorButton(Tag.e.c),
                      ),
                    ],
                  ),
                  child: const AnchorButton(Tag.e),
                ),
              ],
            ),
            child: AnchorButton(Tag.anchor, focusNode: focusNode),
          ),
        ),
      ),
    );

    listenForFocusChanges();

    // Have to open a menu initially to start things going.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    // Arrow down moves to first item
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.b.focusNode));

    // Horizontal traversal on menu items without submenus shouldn't do
    // anything.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(focusedMenu, equals(Tag.b.focusNode));

    // Move to the first submenu
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.c.focusNode));

    // Arrow left should do nothing since no menu is open
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));

    // Arrow right should open the submenu and focus the first item
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.a.focusNode));

    // Arrow left should close the submenu and refocus its anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    // Enter should open the submenu without changing focus.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsOneWidget);

    // Arrow down should close the submenu and focus the next anchor sibling.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    expect(focusedMenu, equals(Tag.d.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.e.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.b.focusNode));

    // Cycle back up
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    expect(focusedMenu, equals(Tag.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    expect(focusedMenu, equals(Tag.e.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);

    expect(focusedMenu, equals(Tag.e.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    expect(focusedMenu, equals(Tag.e.c.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(focusedMenu, equals(Tag.e.c.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.e.c.b.focusNode));

    // Arrow left should close a menu item's overlay and refocus its anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.e.c.focusNode));
    expect(find.text(Tag.e.c.a.text), findsNothing);

    // Arrow left from the submenu anchor should behave the same as a regular
    // menu item.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    expect(focusedMenu, equals(Tag.e.focusNode));
    expect(find.text(Tag.e.c.text), findsNothing);
    expect(eventBubbled, isFalse);
  });

  testWidgets('[Default] RTL Directional traversal', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    bool eventBubbled = false;
    await tester.pumpWidget(
      App(
        textDirection: TextDirection.rtl,
        Actions(
          actions: <Type, Action<Intent>>{
            // Intents should not bubble up to the root anchor.
            DirectionalFocusIntent: CallbackAction<DirectionalFocusIntent>(
              onInvoke: (DirectionalFocusIntent intent) {
                eventBubbled = true;
                return null;
              },
            ),
          },
          child: RawMenuAnchor(
            childFocusNode: focusNode,
            panel: RawMenuPanel(
              menuChildren: <Widget>[
                Button.tag(Tag.a),
                Button.tag(Tag.b),
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[Button.tag(Tag.c.a), Button.tag(Tag.c.b)],
                  ),
                  child: const AnchorButton(Tag.c),
                ),
                Button.tag(Tag.d),
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[
                      Button.tag(Tag.e.a),
                      Button.tag(Tag.e.b),
                      RawMenuAnchor(
                        panel: RawMenuPanel(
                          menuChildren: <Widget>[
                            Button.tag(Tag.e.c.a),
                            Button.tag(Tag.e.c.b),
                            Button.tag(Tag.e.c.c),
                          ],
                        ),
                        child: AnchorButton(Tag.e.c),
                      ),
                    ],
                  ),
                  child: const AnchorButton(Tag.e),
                ),
              ],
            ),
            child: AnchorButton(Tag.anchor, focusNode: focusNode),
          ),
        ),
      ),
    );

    listenForFocusChanges();
    // Have to open a menu initially to start things going.
    await tester.tap(find.text(Tag.anchor.text));
    await tester.pump();

    // Arrow down moves to first item
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.b.focusNode));

    // Horizontal traversal on menu items without submenus shouldn't do
    // anything.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(focusedMenu, equals(Tag.b.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.c.focusNode));

    // Arrow right should do nothing since no menu is open
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));

    // Arrow left should open the submenu and focus the first item
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.a.focusNode));

    // Arrow right should close the submenu and refocus its anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    // Enter should open the submenu without changing focus.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.c.a.text), findsOneWidget);

    // Arrow down should close the submenu and focus the next anchor sibling.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    expect(focusedMenu, equals(Tag.d.focusNode));
    expect(find.text(Tag.c.a.text), findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.e.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.b.focusNode));

    // Cycle back up
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    expect(focusedMenu, equals(Tag.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    expect(focusedMenu, equals(Tag.e.focusNode));

    // Drill down to the nested anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);

    expect(focusedMenu, equals(Tag.e.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    expect(focusedMenu, equals(Tag.e.c.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    expect(focusedMenu, equals(Tag.e.c.a.focusNode));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    expect(focusedMenu, equals(Tag.e.c.b.focusNode));

    // Arrow right should close a menu item's overlay refocus its anchor.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.e.c.focusNode));
    expect(find.text(Tag.e.c.a.text), findsNothing);

    // Arrow right from the submenu anchor should behave the same as a menu
    // item without a submenu.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();

    expect(focusedMenu, equals(Tag.e.focusNode));
    expect(find.text(Tag.e.c.text), findsNothing);
    expect(eventBubbled, isFalse);
  });

  testWidgets('[Default] Closed RawMenuAnchor does not affect anchor tab traversal', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: Tag.b.focusNode);
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      App(
        Row(
          children: <Widget>[
            Button.tag(Tag.a),
            RawMenuAnchor(
              controller: controller,
              panel: RawMenuPanel(
                menuChildren: <Widget>[
                  Button.tag(Tag.b.a),
                  Button.tag(Tag.b.b),
                  Button.tag(Tag.b.c),
                ],
              ),
              child: AnchorButton(Tag.b, focusNode: focusNode),
            ),
            Button.tag(Tag.c),
          ],
        ),
      ),
    );

    listenForFocusChanges();

    focusNode.requestFocus();
    await tester.pump();
    expect(focusedMenu, equals(Tag.b.focusNode));

    // Tab on an unopened anchor should move focus to next widget
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    expect(focusedMenu, equals(Tag.c.focusNode));

    // Move focus back to the anchor
    focusNode.requestFocus();
    await tester.pump();
    expect(focusedMenu, equals(Tag.b.focusNode));

    // Shift+Tab on unopened anchor should move focus to previous widget
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    expect(focusedMenu, equals(Tag.a.focusNode));
  });

  // Menu implementations differ as to whether tabbing traverses a closes a
  // menu or traverses its items. By default, we let the user choose whether
  // to close the menu or traverse its items.
  testWidgets('Tab traversal is not handled.', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: Tag.b.focusNode);
    addTearDown(focusNode.dispose);
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
              child: RawMenuAnchor.node(
                child: Column(
                  children: <Widget>[
                    Button.tag(Tag.a),
                    RawMenuAnchor(
                      controller: controller,
                      childFocusNode: focusNode,
                      panel: RawMenuPanel(
                        menuChildren: <Widget>[
                          Button.tag(Tag.b.a),
                          Button.tag(Tag.b.b),
                          Button.tag(Tag.b.c),
                        ],
                      ),
                      child: AnchorButton(Tag.b, focusNode: focusNode),
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

    // Open overlay and focus first menu item
    focusNode.requestFocus();
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
    focusNode.requestFocus();
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
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

  testWidgets('[Default] Menus close when anchor and overlay are blurred', (
    WidgetTester tester,
  ) async {
    final FocusNode bFocusNode = FocusNode(debugLabel: Tag.b.focusNode);
    final FocusNode cFocusNode = FocusNode(debugLabel: Tag.c.focusNode);
    addTearDown(bFocusNode.dispose);
    addTearDown(cFocusNode.dispose);

    await tester.pumpWidget(
      App(
        Row(
          children: <Widget>[
            Button.tag(Tag.a),
            RawMenuAnchor(
              childFocusNode: bFocusNode,
              panel: RawMenuPanel(
                menuChildren: <Widget>[
                  Button.tag(Tag.b.a),
                  RawMenuAnchor(
                    panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.b.b.a)]),
                    child: AnchorButton(Tag.b.b),
                  ),
                  Button.tag(Tag.b.c),
                ],
              ),
              child: AnchorButton(Tag.b, focusNode: bFocusNode),
            ),
            Button.tag(Tag.c, focusNode: cFocusNode),
          ],
        ),
      ),
    );

    listenForFocusChanges();

    // First, test that a root anchor is closed when tabbing away from it.
    bFocusNode.requestFocus();
    await tester.tap(find.text(Tag.b.text));
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Tab moves focus to the next root anchor sibling
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    // Menu should be closed
    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.b.a.text), findsNothing);

    bFocusNode.requestFocus();
    await tester.tap(find.text(Tag.b.text));
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.focusNode));
    expect(find.text(Tag.b.a.text), findsOneWidget);

    // Move focus to the previous root anchor sibling
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pump();

    // Menu should be closed
    expect(focusedMenu, equals(Tag.a.focusNode));
    expect(find.text(Tag.b.a.text), findsNothing);

    // Next, test that a nested anchor is closed when tabbing away from it.
    // This test also checks that the presence of a focus node does not
    // affect the menu

    // Open nested menu and focus first anchor
    bFocusNode.requestFocus();
    await tester.tap(find.text(Tag.b.text));
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.b.focusNode));
    expect(find.text(Tag.b.b.a.text), findsOneWidget);

    // Tab moves focus to the next anchor sibling
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pump();

    // Nested menu should be closed
    expect(focusedMenu, equals(Tag.b.c.focusNode));
    expect(find.text(Tag.b.b.a.text), findsNothing);

    // Move focus to nested anchor and open menu.
    bFocusNode.requestFocus();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();

    expect(focusedMenu, equals(Tag.b.b.focusNode));
    expect(find.text(Tag.b.b.a.text), findsOneWidget);

    // Shift+Tab moves focus to the previous root anchor sibling
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pump();

    // Nested menu should be closed
    expect(focusedMenu, equals(Tag.b.a.focusNode));
    expect(find.text(Tag.b.b.a.text), findsNothing);

    // Finally, test that menus are closed when focus is moved
    // programmatically.
    cFocusNode.requestFocus();
    await tester.pump();
    await tester.pump();

    expect(focusedMenu, equals(Tag.c.focusNode));
    expect(find.text(Tag.b.a.text), findsNothing);
  });

  testWidgets('[Default] Light Surface Appearance', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          menuAlignment: Alignment.center,
          panel: const RawMenuPanel(menuChildren: <Widget>[SizedBox(height: 100, width: 200)]),
          child: Container(),
        ),
      ),
    );

    controller.open(position: const Offset(200, 200));
    await tester.pump();

    expect(
      findMenuPanelDescendent<Container>(tester).decoration,
      RawMenuPanel.lightSurfaceDecoration,
    );
  });

  testWidgets('[Default] Dark Surface Appearance', (WidgetTester tester) async {
    await tester.pumpWidget(
      App(
        MediaQuery(
          data: const MediaQueryData(platformBrightness: Brightness.dark),
          child: RawMenuAnchor(
            controller: controller,
            menuAlignment: Alignment.center,
            panel: const RawMenuPanel(menuChildren: <Widget>[SizedBox(height: 100, width: 200)]),
            child: Container(),
          ),
        ),
      ),
    );

    controller.open(position: const Offset(200, 200));
    await tester.pump();

    expect(
      findMenuPanelDescendent<Container>(tester).decoration,
      RawMenuPanel.darkSurfaceDecoration,
    );
  });

  testWidgets('Surface decoration can be changed', (WidgetTester tester) async {
    const BoxDecoration decoration = BoxDecoration(color: Color(0xFF0000FF));
    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          controller: controller,
          menuAlignment: Alignment.center,
          panel: const RawMenuPanel(
            decoration: decoration,
            menuChildren: <Widget>[SizedBox(height: 100, width: 200)],
          ),
          child: Container(),
        ),
      ),
    );

    controller.open(position: const Offset(200, 200));
    await tester.pump();

    expect(findMenuPanelDescendent<Container>(tester).decoration, decoration);
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
              child: RawMenuAnchor(
                onOpen: () {
                  opened = true;
                  closed = false;
                },
                onClose: () {
                  opened = false;
                  closed = true;
                },
                controller: controller,
                panel: RawMenuPanel(menuChildren: <Widget>[Text(Tag.a.text)]),
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
          child: RawMenuAnchor(
            alignment: Alignment.bottomCenter,
            menuAlignment: Alignment.topCenter,
            onOpen: () {
              onOpen(Tag.anchor);
            },
            onClose: () {
              onClose(Tag.anchor);
            },
            panel: RawMenuPanel(
              menuChildren: <Widget>[
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
            child: RawMenuAnchor(
              alignment: Alignment.bottomCenter,
              menuAlignment: Alignment.topCenter,
              onOpen: () {
                rootOpened = true;
              },
              onClose: () {
                rootOpened = false;
              },
              panel: RawMenuPanel(
                menuChildren: <Widget>[
                  RawMenuAnchor(
                    alignmentOffset: const Offset(10, 0),
                    alignment: Alignment.topRight,
                    menuAlignment: Alignment.topLeft,
                    onOpen: () {
                      onOpen(Tag.a);
                    },
                    onClose: () {
                      onClose(Tag.a);
                    },
                    panel: RawMenuPanel(
                      menuChildren: <Widget>[
                        Button.tag(Tag.a.a, constraints: largeButtonConstraints),
                      ],
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
  testWidgets('RawMenuAnchor builder rebuilds when isOpen state changes', (
    WidgetTester tester,
  ) async {
    bool isOpen = false;
    int openCount = 0;
    int closeCount = 0;

    await tester.pumpWidget(
      App(
        RawMenuAnchor(
          panel: RawMenuPanel(menuChildren: <Widget>[Button.text('Menu Item')]),
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
                  child: RawMenuAnchor(
                    useRootOverlay: true,
                    controller: controller,
                    panel: RawMenuPanel(
                      menuChildren: <Widget>[
                        RawMenuAnchor(
                          panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a.a)]),
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
                  child: RawMenuAnchor(
                    controller: controller,
                    panel: RawMenuPanel(
                      menuChildren: <Widget>[
                        // Nested menus should be rendered in the same overlay as
                        // their parent, so useRootOverlay should have no effect.
                        RawMenuAnchor(
                          useRootOverlay: true,
                          panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a.a)]),
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
          child: RawMenuAnchor.node(
            controller: controller,
            child: const RawMenuAnchor(
              panel: RawMenuPanel(menuChildren: <Widget>[]),
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

  group('[Default] Layout', () {
    final List<AlignmentGeometry> alignments = <AlignmentGeometry>[
      for (double x = -2; x <= 2; x += 1)
        for (double y = -2; y <= 2; y += 1) Alignment(x, y),
      for (double x = -2; x <= 2; x += 1)
        for (double y = -2; y <= 2; y += 1) AlignmentDirectional(x, y),
    ];

    /// Returns the rects of the menu's contents. If [clipped] is true, the
    /// rect is taken after UnconstrainedBox clips its contents.
    List<Rect> collectOverlays({bool clipped = true}) {
      final List<Rect> menuRects = <Rect>[];
      final Finder finder = clipped ? find.byType(RawMenuPanel) : findOverlayContents();
      for (final Element candidate in finder.evaluate().toList()) {
        final RenderBox box = candidate.renderObject! as RenderBox;
        final Offset topLeft = box.localToGlobal(box.size.topLeft(Offset.zero));
        menuRects.add(topLeft & box.size);
      }
      return menuRects;
    }

    testWidgets('LTR alignment', (WidgetTester tester) async {
      Widget buildApp({AlignmentGeometry? alignment}) {
        return App(
          RawMenuAnchor(
            alignment: alignment,
            menuAlignment: Alignment.center,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 100,
                  height: 50,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      // Anchor position is fixed.
      final ui.Rect anchorRect = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));

      for (final AlignmentGeometry alignment in alignments) {
        await tester.pumpWidget(buildApp(alignment: alignment));
        final ui.Rect overlay = tester.getRect(find.widgetWithText(Container, Tag.a.text).first);
        expect(
          alignment.resolve(TextDirection.ltr).withinRect(anchorRect),
          overlay.center,
          reason:
              'Anchor alignment: $alignment \n'
              'Menu rect: $overlay \n',
        );
      }
    });

    testWidgets('RTL alignment', (WidgetTester tester) async {
      Widget buildApp({AlignmentGeometry? alignment}) {
        return App(
          textDirection: TextDirection.rtl,
          RawMenuAnchor(
            alignment: alignment,
            menuAlignment: Alignment.center,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 100,
                  height: 50,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      // Anchor position is fixed.
      final ui.Rect anchorRect = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));

      for (final AlignmentGeometry alignment in alignments) {
        await tester.pumpWidget(buildApp(alignment: alignment));
        final ui.Rect overlay = tester.getRect(find.widgetWithText(Container, Tag.a.text).first);
        expect(
          alignment.resolve(TextDirection.rtl).withinRect(anchorRect),
          overlay.center,
          reason:
              'Anchor alignment: $alignment \n'
              'Menu rect: $overlay \n',
        );
      }
    });

    testWidgets('LTR menu alignment', (WidgetTester tester) async {
      const Size size = Size(800, 600);
      await changeSurfaceSize(tester, size);

      Widget buildApp({AlignmentGeometry? alignment}) {
        return App(
          RawMenuAnchor(
            alignment: Alignment.center,
            menuAlignment: alignment,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 100,
                  height: 50,
                  alignment: Alignment.center,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      for (final AlignmentGeometry alignment in alignments) {
        for (double y = -2; y <= 2; y += 1) {
          await tester.pumpWidget(buildApp(alignment: alignment));
          final ui.Rect overlay = tester.getRect(find.widgetWithText(Container, Tag.a.text).first);

          expect(
            alignment.resolve(TextDirection.ltr).withinRect(overlay),
            size.center(Offset.zero),
            reason:
                'Menu alignment: $alignment \n'
                'Menu rect: $overlay \n',
          );
        }
      }
    });

    testWidgets('RTL menu alignment', (WidgetTester tester) async {
      const Size size = Size(800, 600);
      await changeSurfaceSize(tester, size);
      Widget buildApp({AlignmentGeometry? alignment}) {
        return App(
          textDirection: TextDirection.rtl,
          RawMenuAnchor(
            alignment: Alignment.center,
            menuAlignment: alignment,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 100,
                  height: 50,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      for (final AlignmentGeometry alignment in alignments) {
        await tester.pumpWidget(buildApp(alignment: alignment));
        final ui.Rect overlay = tester.getRect(find.widgetWithText(Container, Tag.a.text).first);
        expect(
          alignment.resolve(TextDirection.rtl).withinRect(overlay),
          size.center(Offset.zero),
          reason:
              'Menu alignment: $alignment \n'
              'Menu rect: $overlay \n',
        );
      }
    });

    testWidgets('LTR menu top-start attaches to anchor bottom-start by default', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(color: Color(0xFF0000FF)),
              menuChildren: <Widget>[
                Container(width: 100, height: 100, color: const Color(0xFF00FF00)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Offset anchorBottomLeft = tester.getBottomLeft(
        find.widgetWithText(Button, Tag.anchor.text),
      );

      expect(anchorBottomLeft, equals(collectOverlays().first.topLeft));
    });

    testWidgets('RTL menu top-start attaches to anchor bottom-start by default', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(color: Color(0xFF0000FF)),
              menuChildren: <Widget>[
                Container(width: 100, height: 100, color: const Color(0xFF00FF00)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Offset anchorBottomLeft = tester.getBottomLeft(
        find.widgetWithText(Button, Tag.anchor.text),
      );

      expect(anchorBottomLeft, equals(collectOverlays().first.topLeft));
    });

    testWidgets('LTR submenu top-start attaches to anchor top-end by default', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(color: Color(0xFF0000FF)),
              menuChildren: <Widget>[
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    decoration: const BoxDecoration(),
                    menuChildren: <Widget>[
                      Container(width: 100, height: 100, color: const Color(0xFF00FF00)),
                    ],
                  ),
                  child: AnchorButton.small(Tag.a),
                ),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      final [ui.Rect menu, ui.Rect submenu] = collectOverlays();
      expect(submenu.topLeft, equals(menu.topRight));
      expect(submenu.bottomRight - menu.topRight, equals(const Offset(100, 100)));
    });

    testWidgets('RTL submenu top-start attaches to anchor top-end by default', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(color: Color(0xFF0000FF)),
              menuChildren: <Widget>[
                RawMenuAnchor(
                  panel: const RawMenuPanel(
                    decoration: BoxDecoration(color: Color(0xFFFF00FF)),
                    menuChildren: <Widget>[SizedBox.square(dimension: 100)],
                  ),
                  child: AnchorButton.small(Tag.a),
                ),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      final [ui.Rect menu, ui.Rect submenu] = collectOverlays();
      expect(submenu.topRight, equals(menu.topLeft));
      expect(submenu.bottomRight - menu.topRight, equals(const Offset(-100, 100)));
    });

    testWidgets('alignmentOffset is not directional by default', (WidgetTester tester) async {
      const ui.Offset offset = Offset(24, 33);

      Widget buildApp({
        Offset alignmentOffset = Offset.zero,
        ui.TextDirection textDirection = ui.TextDirection.ltr,
      }) {
        return App(
          textDirection: textDirection,
          RawMenuAnchor(
            alignmentOffset: alignmentOffset,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 250,
                  height: 66,
                  alignment: Alignment.center,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Rect ltrPosition = collectOverlays().first;

      await tester.pumpWidget(buildApp(alignmentOffset: offset));

      final Rect ltrPositionTwo = collectOverlays().first;

      expect(ltrPositionTwo, equals(ltrPosition.shift(offset)));

      await tester.pumpWidget(buildApp(textDirection: ui.TextDirection.rtl));

      final Rect rtlPosition = collectOverlays().first;

      await tester.pumpWidget(
        buildApp(alignmentOffset: offset, textDirection: ui.TextDirection.rtl),
      );

      final Rect rtlPositionTwo = collectOverlays().first;

      expect(rtlPositionTwo, equals(rtlPosition.shift(offset)));
    });

    testWidgets('LTR alignmentOffset', (WidgetTester tester) async {
      const ui.Offset offset = Offset(24, 33);

      Widget buildApp({
        Offset alignmentOffset = Offset.zero,
        AlignmentGeometry anchorAlignment = Alignment.center,
      }) {
        return App(
          RawMenuAnchor(
            alignment: anchorAlignment,
            menuAlignment: Alignment.center,
            alignmentOffset: alignmentOffset,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 125,
                  height: 66,
                  alignment: Alignment.center,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Rect center = collectOverlays().first;

      await tester.pumpWidget(buildApp(alignmentOffset: offset));

      expect(center.shift(offset), equals(collectOverlays().first));

      await tester.pumpWidget(buildApp(alignmentOffset: -offset));

      expect(center.shift(-offset), equals(collectOverlays().first));
    });

    testWidgets('RTL alignmentOffset', (WidgetTester tester) async {
      // Should be the same as LTR alignmentOffset test.
      const ui.Offset offset = Offset(24, 33);

      Widget buildApp({
        Offset alignmentOffset = Offset.zero,
        AlignmentGeometry anchorAlignment = Alignment.center,
      }) {
        return App(
          textDirection: ui.TextDirection.rtl,
          RawMenuAnchor(
            alignment: anchorAlignment,
            menuAlignment: Alignment.center,
            alignmentOffset: alignmentOffset,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 125,
                  height: 66,
                  alignment: Alignment.center,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Rect center = collectOverlays().first;

      await tester.pumpWidget(buildApp(alignmentOffset: offset));

      expect(center.shift(offset), equals(collectOverlays().first));

      await tester.pumpWidget(buildApp(alignmentOffset: -offset));

      expect(center.shift(-offset), equals(collectOverlays().first));
    });

    testWidgets(
      'LTR alignmentOffset.dx does not change when menuAlignment is an AlignmentDirectional',
      (WidgetTester tester) async {
        const ui.Offset offset = Offset(24, 33);

        Widget buildApp({
          AlignmentGeometry alignment = Alignment.center,
          Offset alignmentOffset = Offset.zero,
        }) {
          return App(
            RawMenuAnchor(
              alignmentOffset: alignmentOffset,
              alignment: alignment,
              menuAlignment: Alignment.center,
              panel: RawMenuPanel(
                decoration: const BoxDecoration(),
                menuChildren: <Widget>[
                  Container(
                    width: 50,
                    height: 66,
                    color: const Color(0xFF0000FF),
                    child: Text(Tag.a.text),
                  ),
                ],
              ),
              child: const AnchorButton(
                Tag.anchor,
                constraints: BoxConstraints.tightFor(width: 125, height: 66),
              ),
            ),
          );
        }

        await tester.pumpWidget(buildApp());

        await tester.tap(find.text(Tag.anchor.text));
        await tester.pump();

        final Rect center = collectOverlays().first;

        await tester.pumpWidget(buildApp(alignmentOffset: offset));

        final Rect centerOffset = collectOverlays().first;

        // Switching from Alignment.center to AlignmentDirectional.center won't
        // relayout the menu, so pump an empty offset to trigger a relayout.
        await tester.pumpWidget(buildApp());

        await tester.pumpWidget(
          buildApp(alignmentOffset: offset, alignment: AlignmentDirectional.center),
        );

        final Rect centerDirectionalOffset = collectOverlays().first;

        expect(centerOffset, equals(center.shift(offset)));
        expect(centerDirectionalOffset, equals(centerOffset));
      },
    );

    testWidgets('RTL alignmentOffset.dx is negated when alignment is an AlignmentDirectional', (
      WidgetTester tester,
    ) async {
      const ui.Offset offset = Offset(24, 33);

      Widget buildApp({
        AlignmentGeometry alignment = Alignment.center,
        Offset alignmentOffset = Offset.zero,
      }) {
        return App(
          textDirection: ui.TextDirection.rtl,
          RawMenuAnchor(
            controller: controller,
            alignmentOffset: alignmentOffset,
            alignment: alignment,
            menuAlignment: Alignment.center,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(
                  width: 50,
                  height: 66,
                  color: const Color(0xFF0000FF),
                  child: Text(Tag.a.text),
                ),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        );
      }

      await tester.pumpWidget(buildApp());
      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Rect center = collectOverlays().first;

      await tester.pumpWidget(buildApp(alignmentOffset: offset));

      final Rect centerOffset = collectOverlays().first;

      // Switching from Alignment.center to AlignmentDirectional.center won't
      // relayout the menu, so pump an empty offset to trigger a relayout.
      await tester.pumpWidget(buildApp());

      await tester.pumpWidget(
        buildApp(alignmentOffset: offset, alignment: AlignmentDirectional.center),
      );

      final Rect centerDirectionalOffset = collectOverlays().first;

      expect(centerOffset, equals(center.shift(offset)));
      expect(centerDirectionalOffset, equals(center.shift(Offset(-offset.dx, offset.dy))));
    });

    testWidgets(
      'RTL alignmentOffset.dx is not negated when menuAlignment is an AlignmentDirectional',
      (WidgetTester tester) async {
        const ui.Offset offset = Offset(24, 33);

        Widget buildApp({
          AlignmentGeometry alignment = Alignment.center,
          Offset alignmentOffset = Offset.zero,
        }) {
          return App(
            textDirection: ui.TextDirection.rtl,
            RawMenuAnchor(
              menuAlignment: alignment,
              alignmentOffset: alignmentOffset,
              alignment: Alignment.center,
              panel: RawMenuPanel(
                menuChildren: <Widget>[
                  Container(
                    width: 50,
                    height: 66,
                    color: const Color(0xFF0000FF),
                    child: Text(Tag.a.text),
                  ),
                ],
                decoration: const BoxDecoration(),
              ),
              child: AnchorButton.small(Tag.anchor),
            ),
          );
        }

        await tester.pumpWidget(buildApp());

        await tester.tap(find.text(Tag.anchor.text));
        await tester.pump();

        final Rect center = collectOverlays().first;

        await tester.pumpWidget(buildApp(alignmentOffset: offset));

        final Rect centerOffset = collectOverlays().first;

        // Switching from Alignment.center to AlignmentDirectional.center won't
        // relayout the menu, so pump an empty offset to trigger a relayout.
        await tester.pumpWidget(buildApp());

        await tester.pumpWidget(
          buildApp(alignmentOffset: offset, alignment: AlignmentDirectional.center),
        );

        final Rect centerDirectionalOffset = collectOverlays().first;

        expect(centerOffset, equals(center.shift(offset)));
        expect(centerDirectionalOffset, equals(centerOffset));
      },
    );

    testWidgets('LTR constrained and offset menu placement', (WidgetTester tester) async {
      await changeSurfaceSize(tester, const Size(200, 200));
      const BoxConstraints constraints = BoxConstraints.tightFor(width: 100, height: 100);

      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            alignmentOffset: const Offset(-100, 100),
            panel: RawMenuPanel(
              constraints: constraints,
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                RawMenuAnchor(
                  alignmentOffset: const Offset(100, -100),
                  panel: RawMenuPanel(
                    constraints: constraints,
                    decoration: const BoxDecoration(),
                    menuChildren: <Widget>[
                      Container(color: const Color(0xFF0000FF), constraints: constraints),
                    ],
                  ),
                  child: const AnchorButton(Tag.a, constraints: constraints),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor, constraints: constraints),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      expect(collectOverlays(), const <Rect>[
        Rect.fromLTRB(0.0, 100.0, 100.0, 200.0),
        Rect.fromLTRB(100.0, 0.0, 200.0, 100.0),
      ]);
    });

    testWidgets('RTL constrained and offset menu placement', (WidgetTester tester) async {
      await changeSurfaceSize(tester, const Size(200, 200));
      const BoxConstraints constraints = BoxConstraints.tightFor(width: 100, height: 100);

      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          RawMenuAnchor(
            alignmentOffset: const Offset(-100, 100),
            panel: RawMenuPanel(
              constraints: constraints,
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                RawMenuAnchor(
                  alignmentOffset: const Offset(100, -100),
                  panel: RawMenuPanel(
                    constraints: constraints,
                    decoration: const BoxDecoration(),
                    menuChildren: <Widget>[
                      Container(color: const Color(0xFF0000FF), constraints: constraints),
                    ],
                  ),
                  child: const AnchorButton(Tag.a, constraints: constraints),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor, constraints: constraints),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      expect(collectOverlays(), const <Rect>[
        Rect.fromLTRB(0.0, 100.0, 100.0, 200.0),
        Rect.fromLTRB(0.0, 0.0, 100.0, 100.0),
      ]);
    });

    testWidgets('LTR constrained menu placement with unconstrained crossaxis', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(200, 200));
      const BoxConstraints constraints = BoxConstraints.tightFor(width: 300, height: 40);

      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    decoration: const BoxDecoration(),
                    menuChildren: <Widget>[Button.tag(Tag.a.a, constraints: constraints)],
                  ),
                  child: const AnchorButton(Tag.a, constraints: constraints),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor, constraints: constraints),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      // The (unclipped) menu surface can grow beyond the screen. The left
      // edge should be 0 so that the leading edge (left when LTR) of a menu
      // item is visible.
      expect(collectOverlays(clipped: false), const <Rect>[
        Rect.fromLTRB(0.0, 120.0, 300.0, 160.0),
        Rect.fromLTRB(0.0, 160.0, 300.0, 200.0),
      ]);
    });

    testWidgets('RTL constrained menu placement with unconstrained crossaxis', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(200, 200));
      const BoxConstraints constraints = BoxConstraints.tightFor(width: 300, height: 40);

      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    decoration: const BoxDecoration(),
                    menuChildren: <Widget>[Button.tag(Tag.a.a, constraints: constraints)],
                  ),
                  child: const AnchorButton(Tag.a, constraints: constraints),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor, constraints: constraints),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      // The (unclipped) menu surface can grow beyond the screen. The left
      // edge should be negative so that the leading edge (right when RTL) of
      // a menu item is visible.
      expect(collectOverlays(clipped: false), const <Rect>[
        Rect.fromLTRB(-100.0, 120.0, 200.0, 160.0),
        Rect.fromLTRB(-100.0, 160.0, 200.0, 200.0),
      ]);
    });

    testWidgets('LTR constrained menu placement with constrained crossaxis', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(200, 200));
      const BoxConstraints constraints = BoxConstraints.tightFor(width: 300, height: 40);

      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              constrainCrossAxis: true,
              menuChildren: <Widget>[
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    constrainCrossAxis: true,
                    decoration: const BoxDecoration(),
                    menuChildren: <Widget>[Button.tag(Tag.a.a, constraints: constraints)],
                  ),
                  child: const AnchorButton(Tag.a, constraints: constraints),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor, constraints: constraints),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      // The (unclipped) menu surface will not grow beyond the screen.
      expect(collectOverlays(clipped: false), const <ui.Rect>[
        Rect.fromLTRB(0.0, 120.0, 200.0, 160.0),
        Rect.fromLTRB(0.0, 160.0, 200.0, 200.0),
      ]);
    });

    testWidgets('RTL constrained menu placement with constrained crossaxis', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(200, 200));
      const BoxConstraints constraints = BoxConstraints.tightFor(width: 300, height: 40);

      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          RawMenuAnchor(
            panel: RawMenuPanel(
              constrainCrossAxis: true,
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                RawMenuAnchor(
                  panel: RawMenuPanel(
                    constrainCrossAxis: true,
                    decoration: const BoxDecoration(),
                    menuChildren: <Widget>[Button.tag(Tag.a.a, constraints: constraints)],
                  ),
                  child: const AnchorButton(Tag.a, constraints: constraints),
                ),
              ],
            ),
            child: const AnchorButton(Tag.anchor, constraints: constraints),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      // The (unclipped) menu surface will not grow beyond the screen.
      expect(collectOverlays(clipped: false), const <Rect>[
        Rect.fromLTRB(0.0, 120.0, 200.0, 160.0),
        Rect.fromLTRB(0.0, 160.0, 200.0, 200.0),
      ]);
    });

    testWidgets('Constraints applied to anchor do not affect overlay', (WidgetTester tester) async {
      await tester.pumpWidget(
        App(
          ConstrainedBox(
            constraints: const BoxConstraints.tightFor(width: 40, height: 40),
            child: RawMenuAnchor(
              panel: RawMenuPanel(
                decoration: const BoxDecoration(),
                menuChildren: <Widget>[
                  Container(color: const Color(0xFFFF0000), height: 125, width: 200),
                ],
              ),
              child: AnchorButton.small(Tag.anchor),
            ),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      expect(collectOverlays().first, const Rect.fromLTRB(380.0, 320.0, 580.0, 445.0));
    });

    testWidgets('LTR menu position flips to left when overflowing screen right', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          alignment: const Alignment(0.5, 0),
          RawMenuAnchor(
            alignment: Alignment.topLeft,
            menuAlignment: const Alignment(-0.75, -0.75),
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(width: 350, height: 100, color: const Color(0x86FF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.pump();

      final [ui.Rect menu] = collectOverlays();
      final ui.Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
      expect(const Alignment(0.75, -0.75).withinRect(menu), equals(anchor.topRight));
    });

    testWidgets('RTL menu position flips to left when overflowing screen right', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          alignment: const Alignment(0.5, 0),
          RawMenuAnchor(
            alignment: Alignment.topLeft,
            menuAlignment: const Alignment(-0.75, -0.75),
            panel: RawMenuPanel(
              menuChildren: <Widget>[
                Container(width: 350, height: 100, color: const Color(0x86FF00FF)),
              ],
              decoration: const BoxDecoration(),
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.pump();

      final [ui.Rect menu] = collectOverlays();
      final Offset anchorTopRight = tester.getTopRight(
        find.widgetWithText(Button, Tag.anchor.text),
      );
      expect(const Alignment(0.75, -0.75).withinRect(menu), equals(anchorTopRight));
    });

    testWidgets('LTR menu position flips to right when overflowing screen left', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          alignment: const Alignment(-0.5, 0),
          RawMenuAnchor(
            alignment: Alignment.topLeft,
            menuAlignment: const Alignment(0.75, -0.75),
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(width: 350, height: 100, color: const Color(0x86FF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final [ui.Rect menu] = collectOverlays();
      final ui.Offset anchorTopRight = tester.getTopRight(
        find.widgetWithText(Button, Tag.anchor.text),
      );
      expect(const Alignment(-0.75, -0.75).withinRect(menu), equals(anchorTopRight));
    });

    testWidgets('RTL menu position flips to right when overflowing screen left', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          alignment: const Alignment(-0.5, 0),
          RawMenuAnchor(
            alignment: Alignment.topLeft,
            menuAlignment: const Alignment(0.75, -0.75),
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(width: 350, height: 100, color: const Color(0x86FF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final [ui.Rect menu] = collectOverlays();
      final ui.Offset anchorTopRight = tester.getTopRight(
        find.widgetWithText(Button, Tag.anchor.text),
      );
      expect(const Alignment(-0.75, -0.75).withinRect(menu), equals(anchorTopRight));
    });

    testWidgets(
      'Menus that overflow the same screen edge when flipped are placed against that edge',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          App(
            RawMenuAnchor(
              controller: controller,
              menuAlignment: Alignment.center,
              panel: RawMenuPanel(
                decoration: const BoxDecoration(),
                menuChildren: <Widget>[
                  Container(width: 100, height: 100, color: const Color(0x86FF00FF)),
                ],
              ),
              child: const Stack(
                children: <Widget>[Positioned.fill(child: ColoredBox(color: Color(0xff00ff00)))],
              ),
            ),
          ),
        );

        controller.open(position: const Offset(750, 50));
        await tester.pump();

        // Overflow top and right, so the menu should be placed against the top
        // right corner.
        expect(collectOverlays().first, equals(const Rect.fromLTRB(700, 0, 800, 100)));

        controller.open(position: const Offset(50, 550));
        await tester.pump();

        // Overflow bottom and left, so the menu should be placed against the bottom
        // left corner.
        expect(collectOverlays().first, equals(const Rect.fromLTRB(0, 500, 100, 600)));
      },
    );

    testWidgets(
      'Menu attaches to closest vertical edge of anchor when overflowing screen left and right',
      (WidgetTester tester) async {
        await changeSurfaceSize(tester, const Size(200, 200));

        await tester.pumpWidget(
          App(
            // Overlaps the bottom of the anchor by 4px.
            RawMenuAnchor(
              alignmentOffset: const Offset(0, -4),
              alignment: AlignmentDirectional.bottomEnd,
              menuAlignment: AlignmentDirectional.topStart,
              panel: RawMenuPanel(
                decoration: const BoxDecoration(color: Color(0xFF0000FF)),
                menuChildren: <Widget>[
                  // Overlaps the top of the anchor by 4px.
                  RawMenuAnchor(
                    alignmentOffset: const Offset(0, 4),
                    alignment: AlignmentDirectional.topStart,
                    menuAlignment: AlignmentDirectional.bottomEnd,
                    panel: RawMenuPanel(
                      decoration: const BoxDecoration(color: Color(0xFF0000FF)),
                      menuChildren: <Widget>[
                        Container(width: 125, height: 30, color: const Color(0xFFFF00FF)),
                      ],
                    ),
                    child: const AnchorButton(
                      Tag.a,
                      constraints: BoxConstraints.tightFor(width: 125, height: 30),
                    ),
                  ),
                ],
              ),
              child: const AnchorButton(
                Tag.anchor,
                constraints: BoxConstraints.tightFor(width: 125, height: 30),
              ),
            ),
          ),
        );

        await tester.tap(find.text(Tag.anchor.text));
        await tester.pump();
        await tester.tap(find.text(Tag.a.text));
        await tester.pump();

        final Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
        final Rect nestedAnchor = tester.getRect(find.widgetWithText(Button, Tag.a.text));

        final List<ui.Rect> overlays = collectOverlays();
        expect(overlays.first.top, equals(anchor.bottom));
        expect(overlays.last.bottom, equals(nestedAnchor.top));
      },
    );

    testWidgets('Menu flips above anchor when overflowing screen bottom', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          alignment: const Alignment(0, 0.5),
          RawMenuAnchor(
            alignmentOffset: const Offset(0, -8),
            panel: RawMenuPanel(
              decoration: const BoxDecoration(color: Color(0xFF0000FF)),
              menuChildren: <Widget>[
                Container(width: 225, height: 230, color: const Color(0xFFFF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
      expect(collectOverlays().first.bottom, equals(anchor.top + 8));
    });

    testWidgets('Menu flips below anchor when overflowing screen top', (WidgetTester tester) async {
      await tester.pumpWidget(
        App(
          alignment: const Alignment(0, -0.5),
          RawMenuAnchor(
            alignment: AlignmentDirectional.topStart,
            menuAlignment: AlignmentDirectional.bottomStart,
            alignmentOffset: const Offset(0, -8),
            panel: RawMenuPanel(
              decoration: const BoxDecoration(color: Color(0xFF0000FF)),
              menuChildren: <Widget>[
                Container(width: 225, height: 230, color: const Color(0xFFFF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
      expect(collectOverlays().first.top, equals(anchor.bottom + 8));
    });

    testWidgets('AlignmentOffset is reflected across anchor when menu flips', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          alignment: const Alignment(0.8, 0.8),
          RawMenuAnchor(
            alignment: Alignment.center,
            menuAlignment: Alignment.center,
            alignmentOffset: const Offset(200, 200),
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(width: 50, height: 50, color: const Color(0xFFFF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Offset anchorCenter = tester.getCenter(find.widgetWithText(Button, Tag.anchor.text));
      expect(collectOverlays().first.center, equals(anchorCenter - const Offset(200, 200)));
    });

    testWidgets('Alignment is reflected across anchor when menu flips', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          alignment: const AlignmentDirectional(0.95, 0.95),
          RawMenuAnchor(
            alignment: AlignmentDirectional.bottomEnd,
            menuAlignment: Alignment.center,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(width: 50, height: 50, color: const Color(0xFFFF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Offset anchorTopLeft = tester.getTopLeft(find.widgetWithText(Button, Tag.anchor.text));
      expect(collectOverlays().first.center, equals(anchorTopLeft));
    });

    testWidgets('MenuAlignment is reflected across anchor when menu flips', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          alignment: const AlignmentDirectional(0.95, 0.95),
          RawMenuAnchor(
            alignment: Alignment.center,
            menuAlignment: AlignmentDirectional.topStart,
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(width: 50, height: 50, color: const Color(0xFFFF00FF)),
              ],
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Offset anchorCenter = tester.getCenter(find.widgetWithText(Button, Tag.anchor.text));
      expect(collectOverlays().first.bottomLeft, equals(anchorCenter));
    });

    testWidgets(
      'Menus opened with a position apply the positional offset relative to the top left corner of the anchor',
      (WidgetTester tester) async {
        await tester.binding.setSurfaceSize(const Size(800, 600));

        Widget buildApp([TextDirection textDirection = TextDirection.ltr]) {
          return App(
            textDirection: textDirection,
            RawMenuAnchor(
              controller: controller,
              alignment: Alignment.topLeft,
              menuAlignment: Alignment.topCenter,
              panel: RawMenuPanel(
                constraints: const BoxConstraints(),
                menuChildren: <Widget>[
                  Container(color: const Color(0xFFFF0000), height: 100, width: 100),
                ],
              ),
              child: Container(width: 100, height: 100, color: const ui.Color(0xFF00FF00)),
            ),
          );
        }

        await tester.pumpWidget(buildApp());

        controller.open();
        await tester.pump();

        final ui.Rect control = collectOverlays().first;

        controller.open(position: const Offset(33, 45));
        await tester.pump();

        expect(collectOverlays().first, control.shift(const Offset(33, 45)));

        // Should not be affected by text direction.
        await tester.pumpWidget(buildApp(TextDirection.rtl));

        expect(collectOverlays().first, control.shift(const Offset(33, 45)));

        controller.open(position: const Offset(45, 75));
        await tester.pump();

        expect(collectOverlays().first, control.shift(const Offset(45, 75)));
      },
    );

    testWidgets('Menus opened with a position ignore `alignmentOffset`', (
      WidgetTester tester,
    ) async {
      await tester.binding.setSurfaceSize(const Size(800, 600));

      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            alignmentOffset: const Offset(33, 45),
            controller: controller,
            alignment: Alignment.topLeft,
            menuAlignment: Alignment.topCenter,
            panel: RawMenuPanel(
              constraints: const BoxConstraints(),
              menuChildren: <Widget>[
                Container(color: const Color(0xFFFF0000), height: 100, width: 100),
              ],
            ),
            child: Container(width: 100, height: 100, color: const ui.Color(0xFF00FF00)),
          ),
        ),
      );

      controller.open();
      await tester.pump();

      // Get position with alignmentOffset.
      final ui.Rect control = collectOverlays().first;

      controller.open(position: Offset.zero);
      await tester.pump();

      // Alignment offset should be removed.
      expect(collectOverlays().first, control.shift(const Offset(-33, -45)));
    });

    testWidgets('Menus opened with a position ignore `alignment`', (WidgetTester tester) async {
      await tester.binding.setSurfaceSize(const Size(800, 600));

      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            controller: controller,
            alignment: Alignment.bottomRight,
            menuAlignment: Alignment.topLeft,
            panel: RawMenuPanel(
              constraints: const BoxConstraints(),
              menuChildren: <Widget>[
                Container(color: const Color(0xFFFF0000), height: 100, width: 100),
              ],
            ),
            child: Container(width: 100, height: 100, color: const ui.Color(0xFF00FF00)),
          ),
        ),
      );
      controller.open();
      await tester.pump();

      // Get position with alignmentOffset.
      final ui.Rect control = collectOverlays().first;

      controller.open(position: Offset.zero);
      await tester.pump();

      // A positioned menu is placed relative to the top left corner of the
      // anchor. The anchor is 100x100, and the alignment is set to
      // bottom-right, so setting the position to
      // Offset.zero should offset the menu by -100 x -100.
      expect(collectOverlays().first, control.shift(const Offset(-100, -100)));
    });

    testWidgets('Menus opened with a position respect the menuAlignment property', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            controller: controller,
            alignment: Alignment.topLeft,
            menuAlignment: Alignment.center,
            padding: const EdgeInsets.all(25),
            panel: RawMenuPanel(
              constraints: const BoxConstraints(),
              menuChildren: <Widget>[
                Container(color: const Color(0xFFFF0000), height: 100, width: 100),
              ],
            ),
            child: Container(width: 100, height: 100, color: const ui.Color(0xFF00FF00)),
          ),
        ),
      );
      controller.open();
      await tester.pump();

      // Get position with alignmentOffset.
      final ui.Rect control = collectOverlays().first;

      controller.open(position: const Offset(100, 100));
      await tester.pump();

      // A positioned menu is placed relative to the top left corner of the
      // anchor. The anchor is 100x100, and the alignment is set to
      // bottom-right, so setting the position to
      // Offset.zero should offset the menu by -100 x -100.
      expect(collectOverlays().first, control.shift(const Offset(100, 100)));
    });

    testWidgets('Menus opened with a position flip relative to an empty rect at `position`', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            controller: controller,
            menuAlignment: Alignment.topLeft,
            panel: RawMenuPanel(
              constraints: const BoxConstraints(),
              decoration: const BoxDecoration(),
              menuChildren: <Widget>[
                Container(color: const ui.Color(0xFF2200FF), height: 100, width: 100),
              ],
            ),
            child: const Stack(
              fit: StackFit.expand,
              children: <Widget>[ColoredBox(color: ui.Color(0xFFFFC800))],
            ),
          ),
        ),
      );

      controller.open(position: const Offset(700, 500));
      await tester.pump();

      // The menu should be placed at the `position` argument, and should
      // fit within the overlay without flipping.
      expect(collectOverlays().first, equals(const Offset(700, 500) & const Size(100, 100)));

      // Overflow right and bottom by 50 pixels.
      controller.open(position: const Offset(750, 550));
      await tester.pump();

      // The menu should horizontally and vertically overflow the overlay,
      // leading to the menu surface flipping across the menu position.
      expect(collectOverlays().first, equals(const Offset(650, 450) & const Size(100, 100)));
    });

    testWidgets('Menu vertical padding', (WidgetTester tester) async {
      const Color paddingColor = Color(0x62000DFF);
      const ui.Color childColor = Color(0xACFF0080);
      final RawMenuAnchor child = RawMenuAnchor(
        controller: controller,
        padding: const EdgeInsets.fromLTRB(0, 5, 0, 3),
        alignment: AlignmentDirectional.bottomStart,
        menuAlignment: AlignmentDirectional.topStart,
        panel: RawMenuPanel(
          padding: const EdgeInsets.fromLTRB(0, 5, 0, 3),
          decoration: const BoxDecoration(color: paddingColor),
          menuChildren: <Widget>[
            ColoredBox(
              color: childColor,
              child: RawMenuAnchor(
                alignment: AlignmentDirectional.topEnd,
                menuAlignment: AlignmentDirectional.topStart,
                padding: const EdgeInsets.fromLTRB(0, 11, 0, 17),
                panel: RawMenuPanel(
                  padding: const EdgeInsets.fromLTRB(0, 11, 0, 17),
                  decoration: const BoxDecoration(color: paddingColor),
                  menuChildren: <Widget>[
                    Container(
                      key: ValueKey<String>(Tag.a.a.text),
                      color: childColor,
                      height: 100,
                      width: 100,
                      child: Text(Tag.a.a.text),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.a),
              ),
            ),
          ],
        ),
        child: AnchorButton.small(Tag.anchor),
      );
      // First, collect measurements without padding.
      await tester.pumpWidget(App(child));

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      final Finder anchorFinder = find.widgetWithText(Button, Tag.anchor.text);
      final Finder aFinder = find.widgetWithText(Button, Tag.a.text);
      final Finder aaFinder = find.widgetWithText(Container, Tag.a.a.text);

      var [Rect menu, Rect sub] = collectOverlays();
      ui.Rect anchor = tester.getRect(anchorFinder);
      ui.Rect a = tester.getRect(aFinder);
      ui.Rect aa = tester.getRect(aaFinder.first);

      // Menu padding - top: 5 bottom: 3
      // Submenu padding - top: 11 bottom: 17

      expect(a.top, equals(anchor.bottom));
      expect(a.top - 5, equals(menu.top));
      expect(a.bottom + 3, equals(menu.bottom));

      expect(a.top, equals(aa.top));
      expect(aa.top - 11, equals(sub.top));
      expect(aa.bottom + 17, equals(sub.bottom));

      controller.close();
      await tester.pump();

      // Test flipped menu padding.
      await tester.pumpWidget(App(alignment: const Alignment(0, 0.9), child));

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      [menu, sub] = collectOverlays();
      anchor = tester.getRect(anchorFinder);
      a = tester.getRect(aFinder);
      aa = tester.getRect(aaFinder.first);

      expect(a.bottom, equals(anchor.top));
      expect(a.bottom + 3, equals(menu.bottom));
      expect(a.top - 5, equals(menu.top));

      expect(a.bottom, equals(aa.bottom));
      expect(aa.bottom + 17, equals(sub.bottom));
      expect(aa.top - 11, equals(sub.top));
    });

    testWidgets('LTR menu horizontal padding', (WidgetTester tester) async {
      final Finder anchorFinder = find.widgetWithText(Button, Tag.anchor.text);
      final Finder aFinder = find.widgetWithText(Button, Tag.a.text);
      final Finder aaFinder = find.widgetWithText(Container, Tag.a.a.text);

      const Color paddingColor = Color(0x62000DFF);
      const ui.Color childColor = Color(0xACFF0080);
      final RawMenuAnchor child = RawMenuAnchor(
        controller: controller,
        padding: const EdgeInsetsDirectional.fromSTEB(5, 0, 3, 0),
        alignment: AlignmentDirectional.bottomStart,
        menuAlignment: AlignmentDirectional.topStart,
        panel: RawMenuPanel(
          decoration: const BoxDecoration(color: paddingColor),
          menuChildren: <Widget>[
            ColoredBox(
              color: childColor,
              child: RawMenuAnchor(
                alignment: AlignmentDirectional.topEnd,
                menuAlignment: AlignmentDirectional.topStart,
                padding: const EdgeInsetsDirectional.fromSTEB(11, 0, 17, 0),
                panel: RawMenuPanel(
                  decoration: const BoxDecoration(color: paddingColor),
                  menuChildren: <Widget>[
                    Container(
                      key: ValueKey<String>(Tag.a.a.text),
                      color: childColor,
                      height: 100,
                      width: 100,
                      child: Text(Tag.a.a.text),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.a),
              ),
            ),
          ],
        ),
        child: AnchorButton.small(Tag.anchor),
      );
      // First, collect measurements without padding.
      await tester.pumpWidget(App(child));

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      var [Rect menu, Rect sub] = collectOverlays();
      ui.Rect anchor = tester.getRect(anchorFinder);
      ui.Rect a = tester.getRect(aFinder);
      ui.Rect aa = tester.getRect(aaFinder.first);

      // Menu padding - top: 5 bottom: 3
      // Submenu padding - top: 11 bottom: 17

      expect(a.left, equals(anchor.left));
      expect(a.left - 5, equals(menu.left));
      expect(a.right + 3, equals(menu.right));

      expect(a.right, equals(aa.left));
      expect(aa.left - 11, equals(sub.left));
      expect(aa.right + 17, equals(sub.right));

      controller.close();
      await tester.pump();

      // Test flipped menu padding.
      await tester.pumpWidget(App(alignment: const AlignmentDirectional(0.9, 0.0), child));

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      [menu, sub] = collectOverlays();
      anchor = tester.getRect(anchorFinder);
      a = tester.getRect(aFinder);
      aa = tester.getRect(aaFinder.first);

      expect(a.right, equals(anchor.right));
      expect(a.right + 3, equals(menu.right));
      expect(a.left - 5, equals(menu.left));

      expect(a.left, equals(aa.right));
      expect(aa.right + 17, equals(sub.right));
      expect(aa.left - 11, equals(sub.left));
    });

    testWidgets('RTL menu horizontal padding', (WidgetTester tester) async {
      const Color paddingColor = Color(0x62000DFF);
      const ui.Color childColor = Color(0xACFF0080);

      final Finder anchorFinder = find.widgetWithText(Button, Tag.anchor.text);
      final Finder aFinder = find.widgetWithText(Button, Tag.a.text);
      final Finder aaFinder = find.widgetWithText(Container, Tag.a.a.text);

      final RawMenuAnchor child = RawMenuAnchor(
        controller: controller,
        padding: const EdgeInsetsDirectional.fromSTEB(5, 0, 3, 0),
        alignment: AlignmentDirectional.bottomStart,
        menuAlignment: AlignmentDirectional.topStart,
        panel: RawMenuPanel(
          decoration: const BoxDecoration(color: paddingColor),
          menuChildren: <Widget>[
            ColoredBox(
              color: childColor,
              child: RawMenuAnchor(
                alignment: AlignmentDirectional.topEnd,
                menuAlignment: AlignmentDirectional.topStart,
                padding: const EdgeInsetsDirectional.fromSTEB(11, 0, 17, 0),
                panel: RawMenuPanel(
                  decoration: const BoxDecoration(color: paddingColor),
                  menuChildren: <Widget>[
                    Container(
                      key: ValueKey<String>(Tag.a.a.text),
                      color: childColor,
                      height: 100,
                      width: 100,
                      child: Text(Tag.a.a.text),
                    ),
                  ],
                ),
                child: const AnchorButton(Tag.a),
              ),
            ),
          ],
        ),
        child: AnchorButton.small(Tag.anchor),
      );
      // First, collect measurements without padding.
      await tester.pumpWidget(App(textDirection: TextDirection.rtl, child));

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      var [Rect menu, Rect sub] = collectOverlays();
      ui.Rect anchor = tester.getRect(anchorFinder);
      ui.Rect a = tester.getRect(aFinder);
      ui.Rect aa = tester.getRect(aaFinder.first);

      // Menu padding    - left: 3 right: 5
      // Submenu padding - left: 17 right: 11

      expect(a.right, equals(anchor.right));
      expect(a.right + 5, equals(menu.right));
      expect(a.left - 3, equals(menu.left));

      expect(a.left, equals(aa.right));
      expect(aa.right + 11, equals(sub.right));
      expect(aa.left - 17, equals(sub.left));

      controller.close();
      await tester.pump();

      // Test flipped menu padding.
      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          alignment: const AlignmentDirectional(0.9, 0.0),
          child,
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      [menu, sub] = collectOverlays();
      anchor = tester.getRect(anchorFinder);
      a = tester.getRect(aFinder);
      aa = tester.getRect(aaFinder.first);

      expect(a.left, equals(anchor.left));
      expect(a.left - 3, equals(menu.left));
      expect(a.right + 5, equals(menu.right));

      expect(a.right, equals(aa.left));
      expect(aa.left - 17, equals(sub.left));
      expect(aa.right + 11, equals(sub.right));
    });

    testWidgets('Menu padding should not overflow screen', (WidgetTester tester) async {
      final Widget menu = RawMenuAnchor(
        controller: controller,
        padding: const EdgeInsets.only(right: 50, top: 30),
        alignment: AlignmentDirectional.topEnd,
        menuAlignment: AlignmentDirectional.topStart,
        panel: RawMenuPanel(
          decoration: const BoxDecoration(color: Color(0x62000DFF)),
          menuChildren: <Widget>[
            Container(
              key: ValueKey<String>(Tag.a.text),
              color: const Color(0xACFF0080),
              height: 100,
              width: 100,
            ),
          ],
        ),
        child: AnchorButton.small(Tag.anchor),
      );

      // The menu should fit in the top-right corner of the screen, with no
      // additional space to the right or top.
      await tester.pumpWidget(
        App(Stack(children: <Widget>[Positioned(top: 30, right: 150, child: menu)])),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Offset anchorTopRight = tester.getTopRight(
        find.widgetWithText(Button, Tag.anchor.text),
      );
      final Offset menuTopLeft = tester.getTopLeft(find.byKey(ValueKey<String>(Tag.a.text)));

      // Menu should not overflow the screen
      expect(menuTopLeft, equals(anchorTopRight));

      controller.close();
      await tester.pump();

      // Reduce the amount of space available to the menu by (1px, 1px).
      await tester.pumpWidget(
        App(Stack(children: <Widget>[Positioned(top: 29, right: 149, child: menu)])),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final Offset anchorTopLeft = tester.getTopLeft(find.widgetWithText(Button, Tag.anchor.text));
      final Offset menuTopRight = tester.getTopRight(find.byKey(ValueKey<String>(Tag.a.text)));

      // Menu overflowed the screen, so it should be placed at the top left
      // corner of the anchor.
      expect(menuTopRight - const Offset(0, 1), equals(anchorTopLeft));
    });

    testWidgets('LTR app and anchor padding', (WidgetTester tester) async {
      // Out of App:
      //    - overlay position affected
      //    - anchor position affected
      // In App:
      //    - anchor position affected
      //
      // Padding inside App DOES NOT affect the overlay position but
      // DOES affect the anchor position.
      await changeSurfaceSize(tester, const Size(400, 400));

      Widget buildApp({
        required EdgeInsetsGeometry appPadding,
        required EdgeInsetsGeometry anchorPadding,
      }) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: Padding(
            padding: appPadding,
            child: App(
              alignment: AlignmentDirectional.topStart,
              Padding(
                padding: anchorPadding,
                child: RawMenuAnchor(
                  alignment: AlignmentDirectional.topStart,
                  menuAlignment: AlignmentDirectional.bottomEnd,
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[
                      RawMenuAnchor(
                        panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a.a)]),
                        child: AnchorButton.small(Tag.a),
                      ),
                    ],
                  ),
                  child: AnchorButton.small(Tag.anchor),
                ),
              ),
            ),
          ),
        );
      }

      // First, collect measurements without padding.
      await tester.pumpWidget(
        buildApp(appPadding: EdgeInsets.zero, anchorPadding: EdgeInsets.zero),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.a.text));
      await tester.pump();

      final Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
      final [Rect first, Rect second] = collectOverlays();

      await tester.pumpWidget(
        buildApp(
          appPadding: const EdgeInsetsDirectional.fromSTEB(31, 7, 43, 0),
          anchorPadding: const EdgeInsetsDirectional.fromSTEB(64, 50, 17, 0),
        ),
      );

      final [Rect firstPadded, Rect secondPadded] = collectOverlays();
      final Rect paddedAnchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));

      expect(paddedAnchor, equals(anchor.shift(const Offset(31 + 64, 7 + 50))));

      // Hits padding on top/left
      expect(firstPadded, equals(first.shift(const Offset(31, 7))));

      // Hits padding on top/right
      expect(secondPadded, equals(second.shift(const Offset(-43, 7))));
    });

    testWidgets('RTL app and anchor padding', (WidgetTester tester) async {
      // Out of App:
      //    - overlay position affected
      //    - anchor position affected
      // In App:
      //    - anchor position affected
      //
      // Padding inside App DOES NOT affect the overlay position but
      // DOES affect the anchor position.

      // First, collect measurements without padding.
      Widget buildApp({
        required EdgeInsetsGeometry appPadding,
        required EdgeInsetsGeometry anchorPadding,
      }) {
        return Directionality(
          textDirection: TextDirection.rtl,
          child: Padding(
            padding: appPadding,
            child: App(
              alignment: AlignmentDirectional.topStart,
              Padding(
                padding: anchorPadding,
                child: RawMenuAnchor(
                  alignment: AlignmentDirectional.topStart,
                  menuAlignment: AlignmentDirectional.bottomEnd,
                  panel: RawMenuPanel(
                    menuChildren: <Widget>[
                      RawMenuAnchor(
                        panel: RawMenuPanel(menuChildren: <Widget>[Button.tag(Tag.a.a)]),
                        child: AnchorButton.small(Tag.a),
                      ),
                    ],
                  ),
                  child: AnchorButton.small(Tag.anchor),
                ),
              ),
            ),
          ),
        );
      }

      // First, collect measurements without padding.
      await tester.pumpWidget(
        buildApp(appPadding: EdgeInsets.zero, anchorPadding: EdgeInsets.zero),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.a.text));
      await tester.pump();

      final Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
      final [Rect first, Rect second] = collectOverlays();

      // Next, collect measurements with padding.
      await tester.pumpWidget(
        buildApp(
          appPadding: const EdgeInsetsDirectional.fromSTEB(31, 7, 43, 0),
          anchorPadding: const EdgeInsetsDirectional.fromSTEB(64, 50, 17, 0),
        ),
      );

      final [Rect menuPadded, Rect subPadded] = collectOverlays();
      final Rect anchorPadded = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));

      expect(anchorPadded, equals(anchor.shift(const Offset(-31 - 64, 7 + 50))));
      expect(menuPadded, equals(first.shift(const Offset(43, 7))));
      expect(subPadded, equals(second.shift(const Offset(43, 7))));
    });

    testWidgets('LTR nested menu placement', (WidgetTester tester) async {
      List<Widget> children = <Widget>[
        Container(height: 600, width: 50, color: const Color(0xFF0000FF)),
      ];
      int layers = 5;
      while (layers-- > 0) {
        children = <Widget>[
          for (int index = 0; index < 4; index++)
            Button.text(
              "${'Sub' * layers}menu $index",
              constraints: const BoxConstraints(maxHeight: 30),
            ),
          RawMenuAnchor(
            padding: const EdgeInsetsDirectional.fromSTEB(0.5, 4, 1, 6),
            alignmentOffset: const Offset(-1, 0),
            alignment: AlignmentDirectional.topEnd,
            panel: RawMenuPanel(
              constraints: BoxConstraints(minWidth: 125 + 75.0 * layers),
              menuChildren: children,
            ),
            child: AnchorButton(
              Tag.values[layers % Tag.values.length],
              constraints: const BoxConstraints(maxHeight: 30),
            ),
          ),
        ];
      }
      await tester.pumpWidget(
        App(
          alignment: AlignmentDirectional.topStart,
          RawMenuAnchor(
            panel: RawMenuPanel(
              constraints: const BoxConstraints(maxWidth: 150),
              menuChildren: children,
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();
      await tester.tap(find.text(Tag.b.text));
      await tester.pump();
      await tester.tap(find.text(Tag.c.text));
      await tester.pump();
      await tester.tap(find.text(Tag.d.text));
      await tester.pump();
      await tester.tap(find.text(Tag.e.text));
      await tester.pump();

      expect(collectOverlays(), const <Rect>[
        Rect.fromLTRB(0.0, 30.0, 109.0, 181.0),
        Rect.fromLTRB(107.0, 146.5, 259.5, 307.5),
        Rect.fromLTRB(256.5, 267.0, 456.5, 428.0),
        Rect.fromLTRB(453.5, 387.5, 728.5, 548.5),
        Rect.fromLTRB(106.5, 387.0, 456.5, 548.0),
        Rect.fromLTRB(375.0, 0.0, 800.0, 600.0),
      ]);
    });

    testWidgets('RTL nested menu placement', (WidgetTester tester) async {
      List<Widget> children = <Widget>[
        Container(height: 600, width: 50, color: const Color(0xFF0000FF)),
      ];
      int layers = 5;
      while (layers-- > 0) {
        children = <Widget>[
          for (int index = 0; index < 4; index++)
            Button.text(
              "${'Sub' * layers}menu $index",
              constraints: const BoxConstraints(maxHeight: 30),
            ),
          RawMenuAnchor(
            padding: const EdgeInsetsDirectional.fromSTEB(0.5, 4, 1, 6),
            alignmentOffset: const Offset(-1, 0),
            alignment: AlignmentDirectional.topEnd,
            panel: RawMenuPanel(
              constraints: BoxConstraints(minWidth: 125 + 75.0 * layers),
              menuChildren: children,
            ),
            child: AnchorButton(
              Tag.values[layers % Tag.values.length],
              constraints: const BoxConstraints(maxHeight: 30),
            ),
          ),
        ];
      }
      await tester.pumpWidget(
        App(
          textDirection: TextDirection.rtl,
          alignment: AlignmentDirectional.topStart,
          RawMenuAnchor(
            panel: RawMenuPanel(
              constraints: const BoxConstraints(maxWidth: 150),
              menuChildren: children,
            ),
            child: AnchorButton.small(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();
      await tester.tap(find.text(Tag.b.text));
      await tester.pump();
      await tester.tap(find.text(Tag.c.text));
      await tester.pump();
      await tester.tap(find.text(Tag.d.text));
      await tester.pump();
      await tester.tap(find.text(Tag.e.text));
      await tester.pump();

      expect(collectOverlays(), const <Rect>[
        Rect.fromLTRB(691.0, 30.0, 800.0, 181.0),
        Rect.fromLTRB(540.5, 146.5, 693.0, 307.5),
        Rect.fromLTRB(343.5, 267.0, 543.5, 428.0),
        Rect.fromLTRB(71.5, 387.5, 346.5, 548.5),
        Rect.fromLTRB(343.5, 387.0, 693.5, 548.0),
        Rect.fromLTRB(0.0, 0.0, 425.0, 600.0),
      ]);
    });

    testWidgets('Menu is positioned around display features', (WidgetTester tester) async {
      await tester.pumpWidget(
        App(
          MediaQuery(
            data: const MediaQueryData(
              platformBrightness: Brightness.dark,
              size: Size(800, 600),
              displayFeatures: <ui.DisplayFeature>[
                // A 20-pixel wide vertical display feature, similar to a
                // foldable with a visible hinge. Splits the display into two
                // "virtual screens".
                ui.DisplayFeature(
                  bounds: Rect.fromLTRB(390, 0, 410, 600),
                  type: ui.DisplayFeatureType.cutout,
                  state: ui.DisplayFeatureState.unknown,
                ),
              ],
            ),
            child: ColoredBox(
              color: const Color(0xFF004CFF),
              child: Stack(
                children: <Widget>[
                  // Pink box for visualizing the display feature.
                  Positioned.fromRect(
                    rect: const Rect.fromLTRB(390, 0, 410, 600),
                    child: const ColoredBox(color: Color(0xF7FF2190)),
                  ),
                  Positioned(
                    left: 500,
                    top: 300,
                    child: RawMenuAnchor(
                      alignment: Alignment.topLeft,
                      menuAlignment: Alignment.topRight,
                      panel: const RawMenuPanel(
                        menuChildren: <Widget>[SizedBox(width: 150, height: 50)],
                      ),
                      child: AnchorButton.small(Tag.anchor),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final double menuLeft = collectOverlays().first.left;
      final ui.Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));

      // Since the display feature splits the display into 2 sub-screens, the
      // menu should be positioned to fit in the second virtual screen.
      expect(menuLeft, equals(anchor.right));
    });

    testWidgets('Menu constraints are applied to menu surface', (WidgetTester tester) async {
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            panel: RawMenuPanel(
              decoration: const BoxDecoration(),
              constraints: const BoxConstraints(minWidth: 75, maxHeight: 100),
              menuChildren: <Widget>[
                Container(key: Tag.a.key, color: const Color(0xFFFF0000), height: 150, width: 50),
              ],
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      expect(collectOverlays().first.size, equals(const Size(75, 100)));

      // Width should expand to 75, but height will remain 150 since it's
      // located inside a scrollable.
      expect(tester.getSize(find.byKey(Tag.a.key)), equals(const Size(75, 150)));
    });

    testWidgets('Padding is applied before constraints', (WidgetTester tester) async {
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            panel: ColoredBox(
              color: const Color(0xFF0000FF),
              child: RawMenuPanel(
                decoration: const BoxDecoration(),
                constraints: BoxConstraints.tight(const Size(100, 100)),
                padding: const EdgeInsets.symmetric(vertical: 50, horizontal: 35),
                menuChildren: <Widget>[
                  Container(key: Tag.a.key, color: const Color(0xFFFF0000), height: 50, width: 50),
                ],
              ),
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      expect(tester.getSize(find.byType(RawMenuPanel)), equals(const Size(100, 100)));
    });

    testWidgets('Padding applied to RawMenuPanel overrides inherited padding from RawMenuAnchor', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            padding: const EdgeInsets.all(50),
            panel: ColoredBox(
              color: const Color(0xFF0000FF),
              child: RawMenuPanel(
                decoration: const BoxDecoration(),
                menuChildren: <Widget>[
                  Container(key: Tag.a.key, color: const Color(0xFFFF0000), height: 50, width: 50),
                ],
              ),
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();

      final ui.Rect inheritedRect = tester.getRect(find.byType(RawMenuPanel));
      await tester.pumpWidget(
        App(
          RawMenuAnchor(
            padding: const EdgeInsets.all(50),
            panel: ColoredBox(
              color: const Color(0xFF0000FF),
              child: RawMenuPanel(
                padding: const EdgeInsets.all(5),
                decoration: const BoxDecoration(),
                menuChildren: <Widget>[
                  Container(key: Tag.a.key, color: const Color(0xFFFF0000), height: 50, width: 50),
                ],
              ),
            ),
            child: const AnchorButton(Tag.anchor),
          ),
        ),
      );

      final ui.Rect overrideRect = tester.getRect(find.byType(RawMenuPanel));

      expect(inheritedRect.topLeft - overrideRect.topLeft, equals(Offset.zero));
      expect(inheritedRect.bottomRight - overrideRect.bottomRight, equals(const Offset(90, 90)));
    });

    testWidgets('Menu is positioned in the root overlay when useRootOverlay is true', (
      WidgetTester tester,
    ) async {
      // The menu should not overflow the bottom of the root overlay, so the
      // menu should be placed below the anchor button.
      final OverlayEntry entry = OverlayEntry(
        builder: (BuildContext context) {
          return const Positioned(
            bottom: 0,
            child: RawMenuAnchor(
              useRootOverlay: true,
              panel: RawMenuPanel(
                decoration: BoxDecoration(color: Color(0xFF0000FF)),
                menuChildren: <Widget>[
                  RawMenuAnchor(
                    menuAlignment: AlignmentDirectional.topStart,
                    alignment: AlignmentDirectional.bottomStart,
                    panel: RawMenuPanel(menuChildren: <Widget>[SizedBox.square(dimension: 100)]),
                    child: AnchorButton(Tag.a),
                  ),
                ],
              ),
              child: AnchorButton(Tag.anchor),
            ),
          );
        },
      );

      // Overlay entries leak if they are not disposed.
      addTearDown(() {
        entry.remove();
        entry.dispose();
      });

      await tester.pumpWidget(
        App(
          Stack(
            children: <Widget>[
              Positioned(
                left: 200,
                top: 200,
                height: 200,
                width: 200,
                child: ColoredBox(
                  color: const Color(0xFFFF0000),
                  child: Overlay(initialEntries: <OverlayEntry>[entry]),
                ),
              ),
            ],
          ),
        ),
      );

      await tester.tap(find.text(Tag.anchor.text));
      await tester.pump();
      await tester.tap(find.text(Tag.a.text));
      await tester.pump();

      final [ui.Rect menu, ui.Rect subMenu] = collectOverlays();
      final Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
      final Rect subAnchor = tester.getRect(find.widgetWithText(Button, Tag.a.text));

      expect(menu.topLeft, equals(anchor.bottomLeft));
      expect(subMenu.topLeft, equals(subAnchor.bottomLeft));
    });

    testWidgets(
      'Menu is positioned within the closest ancestor overlay when useRootOverlay is false',
      (WidgetTester tester) async {
        // The menu should overflow the bottom of the nearest ancestor overlay, so
        // the menu should be placed above the anchor button.
        final OverlayEntry entry = OverlayEntry(
          builder: (BuildContext context) {
            return const Positioned(
              bottom: 0,
              child: RawMenuAnchor(
                panel: RawMenuPanel(
                  decoration: BoxDecoration(color: Color(0xFF0000FF)),
                  menuChildren: <Widget>[
                    // Nested menus should be placed in the same overlay as their
                    // parent menu, so this menu should be placed in the nearest
                    // overlay instead of the root overlay.
                    RawMenuAnchor(
                      useRootOverlay: true,
                      menuAlignment: AlignmentDirectional.topStart,
                      alignment: AlignmentDirectional.bottomStart,
                      panel: RawMenuPanel(menuChildren: <Widget>[SizedBox.square(dimension: 100)]),
                      child: AnchorButton(Tag.a),
                    ),
                  ],
                ),
                child: AnchorButton(Tag.anchor),
              ),
            );
          },
        );

        // Overlay entries leak if they are not disposed.
        addTearDown(() {
          entry.remove();
          entry.dispose();
        });

        await tester.pumpWidget(
          App(
            Stack(
              children: <Widget>[
                Positioned(
                  left: 200,
                  top: 200,
                  height: 200,
                  width: 200,
                  child: Overlay(initialEntries: <OverlayEntry>[entry]),
                ),
              ],
            ),
          ),
        );

        await tester.tap(find.text(Tag.anchor.text));
        await tester.pump();
        await tester.tap(find.text(Tag.a.text));
        await tester.pump();

        final [ui.Rect menu, ui.Rect subMenu] = collectOverlays();
        final Rect anchor = tester.getRect(find.widgetWithText(Button, Tag.anchor.text));
        final Rect subAnchor = tester.getRect(find.widgetWithText(Button, Tag.a.text));

        expect(menu.bottomLeft, equals(anchor.topLeft));
        expect(subMenu.bottomLeft, equals(subAnchor.topLeft));
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
  static const NestedTag e = NestedTag('e');

  static const List<NestedTag> values = <NestedTag>[a, b, c, d, e];

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

class AnchorButton extends StatelessWidget {
  const AnchorButton(
    this.tag, {
    super.key,
    this.onPressed,
    this.constraints,
    this.autofocus = false,
    this.focusNode,
  });

  factory AnchorButton.small(Tag tag) {
    return AnchorButton(tag, constraints: BoxConstraints.tight(const Size(100, 30)));
  }

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
          if (controller.isOpen) {
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
