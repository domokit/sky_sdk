// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'multi_view_testing.dart';

void main() {
  testWidgets('Providing a RenderObjectWidget directly to the RootWidget fails', (
    WidgetTester tester,
  ) async {
    // No render tree exists to attach the RenderObjectWidget to.
    await tester.pumpWidget(wrapWithView: false, const ColoredBox(color: Colors.red));

    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError error) => error.message,
        'message',
        startsWith(
          'The render object for ColoredBox cannot find ancestor render object to attach to.',
        ),
      ),
    );
  });

  testWidgets('Moving a RenderObjectWidget to the RootWidget via GlobalKey fails', (
    WidgetTester tester,
  ) async {
    final Widget globalKeyedWidget = ColoredBox(key: GlobalKey(), color: Colors.red);

    await tester.pumpWidget(wrapWithView: false, View(view: tester.view, child: globalKeyedWidget));
    expect(tester.takeException(), isNull);

    await tester.pumpWidget(wrapWithView: false, globalKeyedWidget);

    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError error) => error.message,
        'message',
        contains('cannot find ancestor render object to attach to.'),
      ),
    );
  });

  testWidgets(
    'A View cannot be a child of a render object widget',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(Center(child: View(view: FakeView(tester.view), child: Container())));

      expect(
        tester.takeException(),
        isFlutterError.having(
          (FlutterError error) => error.message,
          'message',
          contains('cannot maintain an independent render tree at its current location.'),
        ),
      );
    },
  );

  testWidgets(
    'The child of a ViewAnchor cannot be a View',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(
        ViewAnchor(child: View(view: FakeView(tester.view), child: Container())),
      );

      expect(
        tester.takeException(),
        isFlutterError.having(
          (FlutterError error) => error.message,
          'message',
          contains('cannot maintain an independent render tree at its current location.'),
        ),
      );
    },
  );

  testWidgets('A View can not be moved via GlobalKey to be a child of a RenderObject', (
    WidgetTester tester,
  ) async {
    final Widget globalKeyedView = View(
      key: GlobalKey(),
      view: FakeView(tester.view),
      child: const ColoredBox(color: Colors.red),
    );

    await tester.pumpWidget(wrapWithView: false, globalKeyedView);
    expect(tester.takeException(), isNull);

    await tester.pumpWidget(wrapWithView: false, View(view: tester.view, child: globalKeyedView));

    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError error) => error.message,
        'message',
        contains('cannot maintain an independent render tree at its current location.'),
      ),
    );
  });

  testWidgets('The view property of a ViewAnchor cannot be a render object widget', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      ViewAnchor(view: const ColoredBox(color: Colors.red), child: Container()),
    );

    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError error) => error.message,
        'message',
        startsWith(
          'The render object for ColoredBox cannot find ancestor render object to attach to.',
        ),
      ),
    );
  });

  testWidgets(
    'A RenderObject cannot be moved into the view property of a ViewAnchor via GlobalKey',
    (WidgetTester tester) async {
      final Widget globalKeyedWidget = ColoredBox(key: GlobalKey(), color: Colors.red);

      await tester.pumpWidget(ViewAnchor(child: globalKeyedWidget));
      expect(tester.takeException(), isNull);

      await tester.pumpWidget(ViewAnchor(view: globalKeyedWidget, child: const SizedBox()));

      expect(
        tester.takeException(),
        isFlutterError.having(
          (FlutterError error) => error.message,
          'message',
          contains('cannot find ancestor render object to attach to.'),
        ),
      );
    },
  );

  testWidgets('ViewAnchor cannot be used at the top of the widget tree (outside of View)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(wrapWithView: false, const ViewAnchor(child: SizedBox()));

    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError error) => error.message,
        'message',
        startsWith(
          'The render object for SizedBox cannot find ancestor render object to attach to.',
        ),
      ),
    );
  });

  testWidgets(
    'ViewAnchor cannot be moved to the top of the widget tree (outside of View) via GlobalKey',
    (WidgetTester tester) async {
      final Widget globalKeyedViewAnchor = ViewAnchor(key: GlobalKey(), child: const SizedBox());

      await tester.pumpWidget(
        wrapWithView: false,
        View(view: tester.view, child: globalKeyedViewAnchor),
      );
      expect(tester.takeException(), isNull);

      await tester.pumpWidget(wrapWithView: false, globalKeyedViewAnchor);

      expect(
        tester.takeException(),
        isFlutterError.having(
          (FlutterError error) => error.message,
          'message',
          contains('cannot find ancestor render object to attach to.'),
        ),
      );
    },
  );

  testWidgets('View can be used at the top of the widget tree', (WidgetTester tester) async {
    await tester.pumpWidget(wrapWithView: false, View(view: tester.view, child: Container()));

    expect(tester.takeException(), isNull);
  });

  testWidgets('View can be moved to the top of the widget tree view GlobalKey', (
    WidgetTester tester,
  ) async {
    final Widget globalKeyView = View(
      view: FakeView(tester.view),
      child: const ColoredBox(color: Colors.red),
    );

    await tester.pumpWidget(
      wrapWithView: false,
      View(
        view: tester.view,
        child: ViewAnchor(
          view: globalKeyView, // This one has trouble when deactivating
          child: const SizedBox(),
        ),
      ),
    );
    expect(tester.takeException(), isNull);
    expect(find.byType(SizedBox), findsOneWidget);
    expect(find.byType(ColoredBox), findsOneWidget);

    await tester.pumpWidget(wrapWithView: false, globalKeyView);
    expect(tester.takeException(), isNull);
    expect(find.byType(SizedBox), findsNothing);
    expect(find.byType(ColoredBox), findsOneWidget);
  });

  testWidgets('ViewCollection can be used at the top of the widget tree', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(views: <Widget>[View(view: tester.view, child: Container())]),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('ViewCollection cannot be used inside a View', (WidgetTester tester) async {
    await tester.pumpWidget(
      ViewCollection(views: <Widget>[View(view: FakeView(tester.view), child: Container())]),
    );

    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError error) => error.message,
        'message',
        startsWith(
          'The Element for ViewCollection cannot be inserted into slot "null" of its ancestor.',
        ),
      ),
    );
  });

  testWidgets('ViewCollection can be used as ViewAnchor.view', (WidgetTester tester) async {
    await tester.pumpWidget(
      ViewAnchor(
        view: ViewCollection(
          views: <Widget>[View(view: FakeView(tester.view), child: Container())],
        ),
        child: Container(),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('ViewCollection cannot have render object widgets as children', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      wrapWithView: false,
      const ViewCollection(views: <Widget>[ColoredBox(color: Colors.red)]),
    );

    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError error) => error.message,
        'message',
        startsWith(
          'The render object for ColoredBox cannot find ancestor render object to attach to.',
        ),
      ),
    );
  });

  testWidgets('Views can be moved in and out of ViewCollections via GlobalKey', (
    WidgetTester tester,
  ) async {
    final Widget greenView = View(
      key: GlobalKey(debugLabel: 'green'),
      view: tester.view,
      child: const ColoredBox(color: Colors.green),
    );
    final Widget redView = View(
      key: GlobalKey(debugLabel: 'red'),
      view: FakeView(tester.view),
      child: const ColoredBox(color: Colors.red),
    );

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          greenView,
          ViewCollection(views: <Widget>[redView]),
        ],
      ),
    );
    expect(tester.takeException(), isNull);
    expect(find.byType(ColoredBox), findsNWidgets(2));

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          redView,
          ViewCollection(views: <Widget>[greenView]),
        ],
      ),
    );
    expect(tester.takeException(), isNull);
    expect(find.byType(ColoredBox), findsNWidgets(2));
  });

  testWidgets('Can move stuff between views via global key: viewA -> viewB', (
    WidgetTester tester,
  ) async {
    final FlutterView greenView = tester.view;
    final FlutterView redView = FakeView(tester.view);
    final Widget globalKeyChild = SizedBox(key: GlobalKey());

    Map<int, RenderObject> collectLeafRenderObjects() {
      final Map<int, RenderObject> result = <int, RenderObject>{};
      for (final RenderView renderView in RendererBinding.instance.renderViews) {
        void visit(RenderObject object) {
          result[renderView.flutterView.viewId] = object;
          object.visitChildren(visit);
        }

        visit(renderView);
      }
      return result;
    }

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(view: greenView, child: ColoredBox(color: Colors.green, child: globalKeyChild)),
          View(view: redView, child: const ColoredBox(color: Colors.red)),
        ],
      ),
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.green), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsNothing,
    );
    final RenderObject boxWithGlobalKey = tester.renderObject(find.byKey(globalKeyChild.key!));

    Map<int, RenderObject> leafRenderObject = collectLeafRenderObjects();
    expect(leafRenderObject[greenView.viewId], isA<RenderConstrainedBox>());
    expect(leafRenderObject[redView.viewId], isNot(isA<RenderConstrainedBox>()));

    // Move the child.
    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(view: greenView, child: const ColoredBox(color: Colors.green)),
          View(view: redView, child: ColoredBox(color: Colors.red, child: globalKeyChild)),
        ],
      ),
    );

    expect(
      find.descendant(of: findsColoredBox(Colors.green), matching: find.byType(SizedBox)),
      findsNothing,
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(tester.renderObject(find.byKey(globalKeyChild.key!)), equals(boxWithGlobalKey));

    leafRenderObject = collectLeafRenderObjects();
    expect(leafRenderObject[greenView.viewId], isNot(isA<RenderConstrainedBox>()));
    expect(leafRenderObject[redView.viewId], isA<RenderConstrainedBox>());
  });

  testWidgets('Can move stuff between views via global key: viewB -> viewA', (
    WidgetTester tester,
  ) async {
    final FlutterView greenView = tester.view;
    final FlutterView redView = FakeView(tester.view);
    final Widget globalKeyChild = SizedBox(key: GlobalKey());

    Map<int, RenderObject> collectLeafRenderObjects() {
      final Map<int, RenderObject> result = <int, RenderObject>{};
      for (final RenderView renderView in RendererBinding.instance.renderViews) {
        void visit(RenderObject object) {
          result[renderView.flutterView.viewId] = object;
          object.visitChildren(visit);
        }

        visit(renderView);
      }
      return result;
    }

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(view: greenView, child: const ColoredBox(color: Colors.green)),
          View(view: redView, child: ColoredBox(color: Colors.red, child: globalKeyChild)),
        ],
      ),
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.green), matching: find.byType(SizedBox)),
      findsNothing,
    );
    final RenderObject boxWithGlobalKey = tester.renderObject(find.byKey(globalKeyChild.key!));

    Map<int, RenderObject> leafRenderObject = collectLeafRenderObjects();
    expect(leafRenderObject[redView.viewId], isA<RenderConstrainedBox>());
    expect(leafRenderObject[greenView.viewId], isNot(isA<RenderConstrainedBox>()));

    // Move the child.
    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(view: greenView, child: ColoredBox(color: Colors.green, child: globalKeyChild)),
          View(view: redView, child: const ColoredBox(color: Colors.red)),
        ],
      ),
    );

    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsNothing,
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.green), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(tester.renderObject(find.byKey(globalKeyChild.key!)), equals(boxWithGlobalKey));

    leafRenderObject = collectLeafRenderObjects();
    expect(leafRenderObject[redView.viewId], isNot(isA<RenderConstrainedBox>()));
    expect(leafRenderObject[greenView.viewId], isA<RenderConstrainedBox>());
  });

  testWidgets('Can move stuff out of a view that is going away, viewA -> ViewB', (
    WidgetTester tester,
  ) async {
    final FlutterView greenView = tester.view;
    final Key greenKey = UniqueKey();
    final FlutterView redView = FakeView(tester.view);
    final Key redKey = UniqueKey();
    final Widget globalKeyChild = SizedBox(key: GlobalKey());

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(key: greenKey, view: greenView, child: const ColoredBox(color: Colors.green)),
          View(
            key: redKey,
            view: redView,
            child: ColoredBox(color: Colors.red, child: globalKeyChild),
          ),
        ],
      ),
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.green), matching: find.byType(SizedBox)),
      findsNothing,
    );
    final RenderObject boxWithGlobalKey = tester.renderObject(find.byKey(globalKeyChild.key!));

    // Move the child and remove its view.
    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(
            key: greenKey,
            view: greenView,
            child: ColoredBox(color: Colors.green, child: globalKeyChild),
          ),
        ],
      ),
    );

    expect(findsColoredBox(Colors.red), findsNothing);
    expect(
      find.descendant(of: findsColoredBox(Colors.green), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(tester.renderObject(find.byKey(globalKeyChild.key!)), equals(boxWithGlobalKey));
  });

  testWidgets('Can move stuff out of a view that is going away, viewB -> ViewA', (
    WidgetTester tester,
  ) async {
    final FlutterView greenView = tester.view;
    final Key greenKey = UniqueKey();
    final FlutterView redView = FakeView(tester.view);
    final Key redKey = UniqueKey();
    final Widget globalKeyChild = SizedBox(key: GlobalKey());

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(
            key: greenKey,
            view: greenView,
            child: ColoredBox(color: Colors.green, child: globalKeyChild),
          ),
          View(key: redKey, view: redView, child: const ColoredBox(color: Colors.red)),
        ],
      ),
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.green), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsNothing,
    );
    final RenderObject boxWithGlobalKey = tester.renderObject(find.byKey(globalKeyChild.key!));

    // Move the child and remove its view.
    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(
            key: redKey,
            view: redView,
            child: ColoredBox(color: Colors.red, child: globalKeyChild),
          ),
        ],
      ),
    );

    expect(findsColoredBox(Colors.green), findsNothing);
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(tester.renderObject(find.byKey(globalKeyChild.key!)), equals(boxWithGlobalKey));
  });

  testWidgets('Can move stuff out of a view that is moving itself, stuff ends up before view', (
    WidgetTester tester,
  ) async {
    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();
    final Key key3 = UniqueKey();
    final Key key4 = UniqueKey();

    final GlobalKey viewKey = GlobalKey();
    final GlobalKey childKey = GlobalKey();

    await tester.pumpWidget(
      Column(
        children: <Widget>[
          SizedBox(key: key1),
          ViewAnchor(
            key: key2,
            view: View(
              key: viewKey,
              view: FakeView(tester.view),
              child: SizedBox(child: ColoredBox(key: childKey, color: Colors.green)),
            ),
            child: const SizedBox(),
          ),
          ViewAnchor(key: key3, child: const SizedBox()),
          SizedBox(key: key4),
        ],
      ),
    );

    await tester.pumpWidget(
      Column(
        children: <Widget>[
          SizedBox(key: key1, child: ColoredBox(key: childKey, color: Colors.green)),
          ViewAnchor(key: key2, child: const SizedBox()),
          ViewAnchor(
            key: key3,
            view: View(key: viewKey, view: FakeView(tester.view), child: const SizedBox()),
            child: const SizedBox(),
          ),
          SizedBox(key: key4),
        ],
      ),
    );

    await tester.pumpWidget(
      Column(
        children: <Widget>[
          SizedBox(key: key1),
          ViewAnchor(
            key: key2,
            view: View(
              key: viewKey,
              view: FakeView(tester.view),
              child: SizedBox(child: ColoredBox(key: childKey, color: Colors.green)),
            ),
            child: const SizedBox(),
          ),
          ViewAnchor(key: key3, child: const SizedBox()),
          SizedBox(key: key4),
        ],
      ),
    );
  });

  testWidgets('Can move stuff out of a view that is moving itself, stuff ends up after view', (
    WidgetTester tester,
  ) async {
    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();
    final Key key3 = UniqueKey();
    final Key key4 = UniqueKey();

    final GlobalKey viewKey = GlobalKey();
    final GlobalKey childKey = GlobalKey();

    await tester.pumpWidget(
      Column(
        children: <Widget>[
          SizedBox(key: key1),
          ViewAnchor(
            key: key2,
            view: View(
              key: viewKey,
              view: FakeView(tester.view),
              child: SizedBox(child: ColoredBox(key: childKey, color: Colors.green)),
            ),
            child: const SizedBox(),
          ),
          ViewAnchor(key: key3, child: const SizedBox()),
          SizedBox(key: key4),
        ],
      ),
    );

    await tester.pumpWidget(
      Column(
        children: <Widget>[
          SizedBox(key: key1),
          ViewAnchor(key: key2, child: const SizedBox()),
          ViewAnchor(
            key: key3,
            view: View(key: viewKey, view: FakeView(tester.view), child: const SizedBox()),
            child: const SizedBox(),
          ),
          SizedBox(key: key4, child: ColoredBox(key: childKey, color: Colors.green)),
        ],
      ),
    );

    await tester.pumpWidget(
      Column(
        children: <Widget>[
          SizedBox(key: key1),
          ViewAnchor(
            key: key2,
            view: View(
              key: viewKey,
              view: FakeView(tester.view),
              child: SizedBox(child: ColoredBox(key: childKey, color: Colors.green)),
            ),
            child: const SizedBox(),
          ),
          ViewAnchor(key: key3, child: const SizedBox()),
          SizedBox(key: key4),
        ],
      ),
    );
  });

  testWidgets('Can globalkey move down the tree from a view that is going away', (
    WidgetTester tester,
  ) async {
    final FlutterView anchorView = FakeView(tester.view);
    final Widget globalKeyChild = SizedBox(key: GlobalKey());

    await tester.pumpWidget(
      ColoredBox(
        color: Colors.green,
        child: ViewAnchor(
          view: View(
            view: anchorView,
            child: ColoredBox(color: Colors.yellow, child: globalKeyChild),
          ),
          child: const ColoredBox(color: Colors.red),
        ),
      ),
    );

    expect(findsColoredBox(Colors.green), findsOneWidget);
    expect(findsColoredBox(Colors.yellow), findsOneWidget);
    expect(
      find.descendant(of: findsColoredBox(Colors.yellow), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(findsColoredBox(Colors.red), findsOneWidget);
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsNothing,
    );
    expect(find.byType(SizedBox), findsOneWidget);
    final RenderObject boxWithGlobalKey = tester.renderObject(find.byKey(globalKeyChild.key!));

    await tester.pumpWidget(
      ColoredBox(
        color: Colors.green,
        child: ViewAnchor(child: ColoredBox(color: Colors.red, child: globalKeyChild)),
      ),
    );
    expect(findsColoredBox(Colors.green), findsOneWidget);
    expect(findsColoredBox(Colors.yellow), findsNothing);
    expect(
      find.descendant(of: findsColoredBox(Colors.yellow), matching: find.byType(SizedBox)),
      findsNothing,
    );
    expect(findsColoredBox(Colors.red), findsOneWidget);
    expect(
      find.descendant(of: findsColoredBox(Colors.red), matching: find.byType(SizedBox)),
      findsOneWidget,
    );
    expect(find.byType(SizedBox), findsOneWidget);
    expect(tester.renderObject(find.byKey(globalKeyChild.key!)), boxWithGlobalKey);
  });

  testWidgets('RenderObjects are disposed when a view goes away from a ViewAnchor', (
    WidgetTester tester,
  ) async {
    final FlutterView anchorView = FakeView(tester.view);

    await tester.pumpWidget(
      ColoredBox(
        color: Colors.green,
        child: ViewAnchor(
          view: View(view: anchorView, child: const ColoredBox(color: Colors.yellow)),
          child: const ColoredBox(color: Colors.red),
        ),
      ),
    );

    final RenderObject box = tester.renderObject(findsColoredBox(Colors.yellow));

    await tester.pumpWidget(
      const ColoredBox(
        color: Colors.green,
        child: ViewAnchor(child: ColoredBox(color: Colors.red)),
      ),
    );

    expect(box.debugDisposed, isTrue);
  });

  testWidgets('RenderObjects are disposed when a view goes away from a ViewCollection', (
    WidgetTester tester,
  ) async {
    final FlutterView redView = tester.view;
    final FlutterView greenView = FakeView(tester.view);

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[
          View(view: redView, child: const ColoredBox(color: Colors.red)),
          View(view: greenView, child: const ColoredBox(color: Colors.green)),
        ],
      ),
    );

    expect(findsColoredBox(Colors.green), findsOneWidget);
    expect(findsColoredBox(Colors.red), findsOneWidget);
    final RenderObject box = tester.renderObject(findsColoredBox(Colors.green));

    await tester.pumpWidget(
      wrapWithView: false,
      ViewCollection(
        views: <Widget>[View(view: redView, child: const ColoredBox(color: Colors.red))],
      ),
    );

    expect(findsColoredBox(Colors.green), findsNothing);
    expect(findsColoredBox(Colors.red), findsOneWidget);
    expect(box.debugDisposed, isTrue);
  });

  testWidgets('View can be wrapped and unwrapped', (WidgetTester tester) async {
    final Widget view = View(view: tester.view, child: const SizedBox());

    await tester.pumpWidget(wrapWithView: false, view);

    final RenderObject renderView = tester.renderObject(find.byType(View));
    final RenderObject renderSizedBox = tester.renderObject(find.byType(SizedBox));

    await tester.pumpWidget(wrapWithView: false, ViewCollection(views: <Widget>[view]));

    expect(tester.renderObject(find.byType(View)), same(renderView));
    expect(tester.renderObject(find.byType(SizedBox)), same(renderSizedBox));

    await tester.pumpWidget(wrapWithView: false, view);

    expect(tester.renderObject(find.byType(View)), same(renderView));
    expect(tester.renderObject(find.byType(SizedBox)), same(renderSizedBox));
  });

  testWidgets('ViewAnchor with View can be wrapped and unwrapped', (WidgetTester tester) async {
    final Widget viewAnchor = ViewAnchor(
      view: View(view: FakeView(tester.view), child: const SizedBox()),
      child: const ColoredBox(color: Colors.green),
    );

    await tester.pumpWidget(viewAnchor);

    final List<RenderObject> renderViews = tester.renderObjectList(find.byType(View)).toList();
    final RenderObject renderSizedBox = tester.renderObject(find.byType(SizedBox));

    await tester.pumpWidget(ColoredBox(color: Colors.yellow, child: viewAnchor));

    expect(tester.renderObjectList(find.byType(View)), renderViews);
    expect(tester.renderObject(find.byType(SizedBox)), same(renderSizedBox));

    await tester.pumpWidget(viewAnchor);

    expect(tester.renderObjectList(find.byType(View)), renderViews);
    expect(tester.renderObject(find.byType(SizedBox)), same(renderSizedBox));
  });

  testWidgets('Moving a View keeps its semantics tree stable', (WidgetTester tester) async {
    final Widget view = View(
      // No explicit key, we rely on the implicit key of the underlying RawView.
      view: tester.view,
      child: Semantics(textDirection: TextDirection.ltr, label: 'Hello', child: const SizedBox()),
    );
    await tester.pumpWidget(wrapWithView: false, view);

    final RenderObject renderSemantics = tester.renderObject(find.bySemanticsLabel('Hello'));
    final SemanticsNode semantics = tester.getSemantics(find.bySemanticsLabel('Hello'));
    expect(semantics.id, 1);
    expect(renderSemantics.debugSemantics, same(semantics));

    await tester.pumpWidget(wrapWithView: false, ViewCollection(views: <Widget>[view]));

    final RenderObject renderSemanticsAfterMove = tester.renderObject(
      find.bySemanticsLabel('Hello'),
    );
    final SemanticsNode semanticsAfterMove = tester.getSemantics(find.bySemanticsLabel('Hello'));
    expect(renderSemanticsAfterMove, same(renderSemantics));
    expect(semanticsAfterMove.id, 1);
    expect(semanticsAfterMove, same(semantics));
  });
}

Finder findsColoredBox(Color color) {
  return find.byWidgetPredicate((Widget widget) => widget is ColoredBox && widget.color == color);
}
