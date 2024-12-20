// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'semantics_tester.dart';

class _ManyRelayoutBoundaries extends StatelessWidget {
  const _ManyRelayoutBoundaries({required this.levels, required this.child});

  final Widget child;

  final int levels;

  @override
  Widget build(BuildContext context) {
    final Widget result =
        levels <= 1 ? child : _ManyRelayoutBoundaries(levels: levels - 1, child: child);
    return SizedBox.square(dimension: 50, child: result);
  }
}

void rebuildLayoutBuilderSubtree(RenderBox descendant, WidgetTester tester) {
  assert(descendant is! RenderConstrainedLayoutBuilder<BoxConstraints, RenderBox>);

  RenderObject? node = descendant.parent;
  while (node != null) {
    if (node is! RenderConstrainedLayoutBuilder<BoxConstraints, RenderBox>) {
      node = node.parent;
    } else {
      final Element layoutBuilderElement = tester.element(
        find.byElementPredicate(
          (Element element) => element.widget is LayoutBuilder && element.renderObject == node,
        ),
      );
      layoutBuilderElement.markNeedsBuild();
      return;
    }
  }
  assert(false);
}

void verifyTreeIsClean() {
  final RenderObject renderObject = RendererBinding.instance.renderView;
  bool hasDirtyNode = renderObject.debugNeedsLayout;

  void visitor(RenderObject renderObject) {
    expect(renderObject.debugNeedsLayout, false, reason: '$renderObject is dirty');
    hasDirtyNode = hasDirtyNode || renderObject.debugNeedsLayout;
    if (!hasDirtyNode) {
      renderObject.visitChildren(visitor);
    }
  }

  visitor(renderObject);
}

void verifyOverlayChildReadyForLayout(GlobalKey overlayWidgetKey) {
  final RenderBox layoutSurrogate =
      overlayWidgetKey.currentContext!.findRenderObject()! as RenderBox;
  assert(
    layoutSurrogate.runtimeType.toString() == '_RenderLayoutSurrogateProxyBox',
    layoutSurrogate.runtimeType,
  );
  if (layoutSurrogate.debugNeedsLayout) {
    assert(layoutSurrogate.debugDoingThisLayout);
  }
  expect(!layoutSurrogate.debugNeedsLayout || layoutSurrogate.debugDoingThisLayout, true);
}

List<RenderObject> _ancestorRenderTheaters(RenderObject child) {
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

void main() {
  final OverlayPortalController controller1 = OverlayPortalController(debugLabel: 'controller1');
  final OverlayPortalController controller2 = OverlayPortalController(debugLabel: 'controller2');
  final OverlayPortalController controller3 = OverlayPortalController(debugLabel: 'controller3');
  final OverlayPortalController controller4 = OverlayPortalController(debugLabel: 'controller4');

  setUp(() {
    controller1.show();
    controller2.show();
    controller3.show();
    controller4.show();
    _PaintOrder.paintOrder.clear();
  });

  testWidgets('The overlay child sees the right inherited widgets', (WidgetTester tester) async {
    int buildCount = 0;
    TextDirection? directionSeenByOverlayChild;
    TextDirection textDirection = TextDirection.rtl;
    late StateSetter setState;
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return StatefulBuilder(
                  builder: (BuildContext context, StateSetter setter) {
                    setState = setter;
                    return Directionality(
                      textDirection: textDirection,
                      child: OverlayPortal(
                        controller: controller1,
                        overlayChildBuilder: (BuildContext context) {
                          buildCount += 1;
                          directionSeenByOverlayChild = Directionality.maybeOf(context);
                          return const SizedBox();
                        },
                        child: const SizedBox(),
                      ),
                    );
                  },
                );
              },
            ),
          ],
        ),
      ),
    );
    expect(buildCount, 1);
    expect(directionSeenByOverlayChild, textDirection);

    setState(() {
      textDirection = TextDirection.ltr;
    });
    await tester.pump();
    expect(buildCount, 2);
    expect(directionSeenByOverlayChild, textDirection);
  });

  testWidgets('The overlay portal update semantics does not dirty overlay', (
    WidgetTester tester,
  ) async {
    late StateSetter setState;
    late final OverlayEntry overlayEntry;
    final UniqueKey overlayKey = UniqueKey();
    String msg = 'msg';
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Semantics(
          container: true,
          child: Overlay(
            key: overlayKey,
            initialEntries: <OverlayEntry>[
              overlayEntry = OverlayEntry(
                builder: (BuildContext context) {
                  return Semantics(
                    container: true,
                    explicitChildNodes: true,
                    child: StatefulBuilder(
                      builder: (BuildContext context, StateSetter setter) {
                        setState = setter;
                        return OverlayPortal(
                          controller: controller1,
                          overlayChildBuilder: (BuildContext context) {
                            return Semantics(
                              label: msg,
                              child: const SizedBox(width: 100, height: 100),
                            );
                          },
                          child: const Text('overlay child'),
                        );
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
    final RenderObject renderObject = tester.renderObject(find.byKey(overlayKey));
    expect(renderObject.debugNeedsSemanticsUpdate, isFalse);
    expect(find.bySemanticsLabel(msg), findsOneWidget);
    setState(() {
      msg = 'msg2';
    });
    // stop before updating semantics.
    await tester.pump(null, EnginePhase.composite);
    expect(renderObject.debugNeedsSemanticsUpdate, isFalse);
  });

  testWidgets('Safe to deactivate and re-activate OverlayPortal', (WidgetTester tester) async {
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    final Widget widget = Directionality(
      key: GlobalKey(debugLabel: 'key'),
      textDirection: TextDirection.ltr,
      child: Overlay(
        initialEntries: <OverlayEntry>[
          overlayEntry = OverlayEntry(
            builder: (BuildContext context) {
              return OverlayPortal(
                controller: controller1,
                overlayChildBuilder: (BuildContext context) => const SizedBox(),
                child: const SizedBox(),
              );
            },
          ),
        ],
      ),
    );

    await tester.pumpWidget(widget);
    await tester.pumpWidget(SizedBox(child: widget));
  });

  testWidgets('Safe to hide overlay child and remove OverlayPortal in the same frame', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/129025.
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    final Widget widget = Directionality(
      key: GlobalKey(debugLabel: 'key'),
      textDirection: TextDirection.ltr,
      child: Overlay(
        initialEntries: <OverlayEntry>[
          overlayEntry = OverlayEntry(
            builder: (BuildContext context) {
              return OverlayPortal(
                controller: controller1,
                overlayChildBuilder: (BuildContext context) => const SizedBox(),
                child: const SizedBox(),
              );
            },
          ),
        ],
      ),
    );

    controller1.show();
    await tester.pumpWidget(widget);

    controller1.hide();
    await tester.pumpWidget(const SizedBox());
    expect(tester.takeException(), isNull);
  });

  testWidgets('Safe to hide overlay child and reparent OverlayPortal in the same frame', (
    WidgetTester tester,
  ) async {
    final OverlayPortal overlayPortal = OverlayPortal(
      key: GlobalKey(debugLabel: 'key'),
      controller: controller1,
      overlayChildBuilder: (BuildContext context) => const SizedBox(),
      child: const SizedBox(),
    );

    List<Widget> children = <Widget>[const SizedBox(), overlayPortal];

    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    late StateSetter setState;
    final Widget widget = Directionality(
      textDirection: TextDirection.ltr,
      child: Overlay(
        initialEntries: <OverlayEntry>[
          overlayEntry = OverlayStatefulEntry(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return Column(children: children);
            },
          ),
        ],
      ),
    );

    controller1.show();
    await tester.pumpWidget(widget);

    controller1.hide();
    setState(() {
      children = <Widget>[overlayPortal, const SizedBox()];
    });
    await tester.pumpWidget(widget);
    expect(tester.takeException(), isNull);
  });

  testWidgets('Safe to hide overlay child and reparent OverlayPortal in the same frame 2', (
    WidgetTester tester,
  ) async {
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    final Widget widget = Directionality(
      key: GlobalKey(debugLabel: 'key'),
      textDirection: TextDirection.ltr,
      child: Overlay(
        initialEntries: <OverlayEntry>[
          overlayEntry = OverlayEntry(
            builder: (BuildContext context) {
              return OverlayPortal(
                controller: controller1,
                overlayChildBuilder: (BuildContext context) => const SizedBox(),
                child: const SizedBox(),
              );
            },
          ),
        ],
      ),
    );

    controller1.show();
    await tester.pumpWidget(widget);

    controller1.hide();
    await tester.pumpWidget(SizedBox(child: widget));
    expect(tester.takeException(), isNull);
  });

  testWidgets(
    'Overlay child remains accessible via tree walk when there is no relayout boundary between OverlayPortal and Overlay',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/133545.
      late final OverlayEntry overlayEntry;
      addTearDown(
        () =>
            overlayEntry
              ..remove()
              ..dispose(),
      );
      final GlobalKey key = GlobalKey(debugLabel: 'key');
      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                // The Positioned widget prevents a relayout boundary from being
                // introduced between the Overlay and OverlayPortal.
                return Positioned(
                  top: 0,
                  left: 0,
                  child: OverlayPortal(
                    controller: controller1,
                    overlayChildBuilder: (BuildContext context) => SizedBox(key: key),
                    child: const SizedBox(),
                  ),
                );
              },
            ),
          ],
        ),
      );

      controller1.hide();
      await tester.pumpWidget(widget);

      controller1.show();
      await tester.pump();
      expect(find.byKey(key), findsOneWidget);
      expect(tester.takeException(), isNull);
      verifyTreeIsClean();
    },
  );

  testWidgets(
    'Throws when the same controller is attached to multiple OverlayPortal',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      final OverlayPortalController controller = OverlayPortalController(
        debugLabel: 'local controller',
      );
      late final OverlayEntry entry;
      addTearDown(() {
        entry.remove();
        entry.dispose();
      });
      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder: (BuildContext context) {
                return Column(
                  children: <Widget>[
                    OverlayPortal(
                      controller: controller,
                      overlayChildBuilder: (BuildContext context) => const SizedBox(),
                      child: const SizedBox(),
                    ),
                    OverlayPortal(
                      controller: controller,
                      overlayChildBuilder: (BuildContext context) => const SizedBox(),
                      child: const SizedBox(),
                    ),
                  ],
                );
              },
            ),
          ],
        ),
      );

      await tester.pumpWidget(widget);
      expect(
        tester.takeException().toString(),
        stringContainsInOrder(<String>['Failed to attach', 'It is already attached to']),
      );
    },
  );

  testWidgets('Properly size itself when the theater is given unbounded constraints', (
    WidgetTester tester,
  ) async {
    // Regression test https://github.com/flutter/flutter/issues/153903.
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    const Size size = Size.square(40);

    final Widget widget = Directionality(
      key: GlobalKey(debugLabel: 'key'),
      textDirection: TextDirection.ltr,
      child: UnconstrainedBox(
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              canSizeOverlay: true,
              builder: (BuildContext context) {
                return OverlayPortal(
                  controller: controller1,
                  overlayChildBuilder: (BuildContext context) => const SizedBox(),
                  child: SizedBox.fromSize(size: size),
                );
              },
            ),
          ],
        ),
      ),
    );

    controller1.show();
    await tester.pumpWidget(widget);
    expect(tester.getSize(find.byType(Overlay)), size);
    expect(tester.takeException(), isNull);
  });

  testWidgets('show/hide works', (WidgetTester tester) async {
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    final OverlayPortalController controller = OverlayPortalController(
      debugLabel: 'local controller',
    );

    const Widget target = SizedBox();
    final Widget widget = Directionality(
      textDirection: TextDirection.ltr,
      child: Overlay(
        initialEntries: <OverlayEntry>[
          overlayEntry = OverlayEntry(
            builder: (BuildContext context) {
              return OverlayPortal(
                controller: controller,
                overlayChildBuilder: (BuildContext context) => target,
              );
            },
          ),
        ],
      ),
    );

    await tester.pumpWidget(widget);
    expect(find.byWidget(target), findsNothing);

    await tester.pump();
    expect(find.byWidget(target), findsNothing);

    controller.show();
    await tester.pump();
    expect(find.byWidget(target), findsOneWidget);

    controller.hide();
    await tester.pump();
    expect(find.byWidget(target), findsNothing);

    controller.show();
    await tester.pump();
    expect(find.byWidget(target), findsOneWidget);
  });

  testWidgets('overlayChildBuilder is not evaluated until show is called', (
    WidgetTester tester,
  ) async {
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    final OverlayPortalController controller = OverlayPortalController(
      debugLabel: 'local controller',
    );

    final Widget widget = Directionality(
      textDirection: TextDirection.ltr,
      child: Overlay(
        initialEntries: <OverlayEntry>[
          overlayEntry = OverlayEntry(
            builder: (BuildContext context) {
              return OverlayPortal(
                controller: controller,
                overlayChildBuilder: (BuildContext context) => throw StateError('Unreachable'),
                child: const SizedBox(),
              );
            },
          ),
        ],
      ),
    );

    await tester.pumpWidget(widget);
    expect(tester.takeException(), isNull);
  });

  testWidgets('overlay child can use Positioned', (WidgetTester tester) async {
    double dimensions = 30;
    late StateSetter setState;
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return StatefulBuilder(
                  builder: (BuildContext context, StateSetter setter) {
                    setState = setter;
                    return OverlayPortal(
                      controller: controller1,
                      overlayChildBuilder: (BuildContext context) {
                        return Positioned(
                          width: dimensions,
                          height: dimensions,
                          child: const Placeholder(),
                        );
                      },
                      child: const SizedBox(),
                    );
                  },
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(tester.getTopLeft(find.byType(Placeholder)), Offset.zero);
    expect(tester.getSize(find.byType(Placeholder)), const Size(30, 30));

    setState(() {
      dimensions = 50;
    });
    await tester.pump();
    expect(tester.getTopLeft(find.byType(Placeholder)), Offset.zero);
    expect(tester.getSize(find.byType(Placeholder)), const Size(50, 50));
  });

  testWidgets('overlay child can be hit tested', (WidgetTester tester) async {
    double offset = 0;
    late StateSetter setState;
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    bool isHit = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return StatefulBuilder(
                  builder: (BuildContext context, StateSetter setter) {
                    setState = setter;
                    return OverlayPortal(
                      controller: controller1,
                      overlayChildBuilder: (BuildContext context) {
                        return Positioned(
                          left: offset,
                          top: offset,
                          width: 1.0,
                          height: 1.0,
                          child: GestureDetector(
                            onTap: () {
                              isHit = true;
                            },
                          ),
                        );
                      },
                      child: const SizedBox(),
                    );
                  },
                );
              },
            ),
          ],
        ),
      ),
    );
    assert(!isHit);
    await tester.tapAt(const Offset(0.5, 0.5));
    expect(isHit, true);
    isHit = false;

    setState(() {
      offset = 50;
    });
    await tester.pump();
    assert(!isHit);

    await tester.tapAt(const Offset(0.5, 0.5));
    expect(isHit, false);
    isHit = false;

    await tester.tapAt(const Offset(50.5, 50.5));
    expect(isHit, true);
  });

  testWidgets('works in a LayoutBuilder', (WidgetTester tester) async {
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return LayoutBuilder(
                  builder: (BuildContext context, BoxConstraints constraints) {
                    return OverlayPortal(
                      controller: controller1,
                      overlayChildBuilder: (BuildContext context) => const SizedBox(),
                      child: const SizedBox(),
                    );
                  },
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('works in a LayoutBuilder 2', (WidgetTester tester) async {
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    late StateSetter setState;
    bool shouldShowChild = false;

    Widget layoutBuilder(BuildContext context, BoxConstraints constraints) {
      return OverlayPortal(
        controller: controller2,
        overlayChildBuilder: (BuildContext context) => const SizedBox(),
        child: const SizedBox(),
      );
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayStatefulEntry(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                return OverlayPortal(
                  controller: controller1,
                  overlayChildBuilder: (BuildContext context) => const SizedBox(),
                  child: shouldShowChild ? LayoutBuilder(builder: layoutBuilder) : null,
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(tester.takeException(), isNull);
    setState(() {
      shouldShowChild = true;
    });

    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('works in a LayoutBuilder 3', (WidgetTester tester) async {
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    late StateSetter setState;
    bool shouldShowChild = false;

    Widget layoutBuilder(BuildContext context, BoxConstraints constraints) {
      return OverlayPortal(
        controller: controller2,
        overlayChildBuilder: (BuildContext context) => const SizedBox(),
        child: const SizedBox(),
      );
    }

    controller1.hide();
    controller2.hide();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayStatefulEntry(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                // The Positioned widget ensures there's no relayout boundary
                // between the Overlay and the OverlayPortal.
                return Positioned(
                  top: 0,
                  left: 0,
                  child: OverlayPortal(
                    controller: controller1,
                    overlayChildBuilder: (BuildContext context) => const SizedBox(),
                    child: shouldShowChild ? LayoutBuilder(builder: layoutBuilder) : null,
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );

    controller1.show();
    controller2.show();
    setState(() {
      shouldShowChild = true;
    });

    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('throws when no Overlay', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SizedBox.square(
          dimension: 50,
          child: OverlayPortal(
            controller: controller1,
            overlayChildBuilder: (BuildContext context) => const SizedBox(),
            child: const SizedBox(),
          ),
        ),
      ),
    );

    expect(
      tester.takeException().toString(),
      startsWith(
        'No Overlay widget found.\n'
        'OverlayPortal widgets require an Overlay widget ancestor.\n'
        'An overlay lets widgets float on top of other widget children.\n'
        'To introduce an Overlay widget, you can either directly include one, or use a widget '
        'that contains an Overlay itself, such as a Navigator, WidgetApp, MaterialApp, or CupertinoApp.\n'
        'The specific widget that could not find a Overlay ancestor was:\n',
      ),
    );
  });

  testWidgets('widget is laid out before overlay child', (WidgetTester tester) async {
    final GlobalKey widgetKey = GlobalKey(debugLabel: 'widget');
    final RenderBox childBox = RenderConstrainedBox(additionalConstraints: const BoxConstraints());
    final RenderBox overlayChildBox = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints(),
    );
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    int layoutCount = 0;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return _ManyRelayoutBoundaries(
                  levels: 50,
                  child: Builder(
                    builder: (BuildContext context) {
                      return OverlayPortal(
                        key: widgetKey,
                        controller: controller1,
                        overlayChildBuilder: (BuildContext context) {
                          return LayoutBuilder(
                            builder: (BuildContext context, BoxConstraints constraints) {
                              verifyOverlayChildReadyForLayout(widgetKey);
                              layoutCount += 1;
                              return WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
                            },
                          );
                        },
                        child: WidgetToRenderBoxAdapter(renderBox: childBox),
                      );
                    },
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
    expect(layoutCount, 1);

    // Make the widget's render object dirty and verifies in the LayoutBuilder's
    // callback that the widget's render object is already laid out.
    final RenderObject renderChild1 = widgetKey.currentContext!.findRenderObject()!;
    renderChild1.markNeedsLayout();
    // Dirty both render subtree branches.
    childBox.markNeedsLayout();
    rebuildLayoutBuilderSubtree(overlayChildBox, tester);

    // Make sure childBox's depth is greater than that of the overlay
    // child, and childBox's parent isn't dirty (childBox is a dirty relayout
    // boundary).
    expect(widgetKey.currentContext!.findRenderObject()!.depth, lessThan(overlayChildBox.depth));

    await tester.pump();
    expect(layoutCount, 2);
    verifyTreeIsClean();
  });

  testWidgets('adding/removing overlay child does not redirty overlay more than once', (
    WidgetTester tester,
  ) async {
    final GlobalKey widgetKey = GlobalKey(debugLabel: 'widget');
    final GlobalKey overlayKey = GlobalKey(debugLabel: 'overlay');
    final RenderBox childBox = RenderConstrainedBox(additionalConstraints: const BoxConstraints());
    final RenderBox overlayChildBox = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints(),
    );
    final _RenderLayoutCounter overlayLayoutCounter = _RenderLayoutCounter();
    late final OverlayEntry overlayEntry1;
    addTearDown(
      () =>
          overlayEntry1
            ..remove()
            ..dispose(),
    );
    late final OverlayEntry overlayEntry2;
    addTearDown(
      () =>
          overlayEntry2
            ..remove()
            ..dispose(),
    );
    int layoutCount = 0;
    controller1.hide();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          key: overlayKey,
          initialEntries: <OverlayEntry>[
            // Overlay.performLayout will call layoutCounter.layout.
            overlayEntry1 = OverlayEntry(
              builder:
                  (BuildContext context) =>
                      WidgetToRenderBoxAdapter(renderBox: overlayLayoutCounter),
            ),
            overlayEntry2 = OverlayEntry(
              builder: (BuildContext context) {
                return _ManyRelayoutBoundaries(
                  levels: 50,
                  child: Builder(
                    builder: (BuildContext context) {
                      return OverlayPortal(
                        key: widgetKey,
                        controller: controller1,
                        overlayChildBuilder: (BuildContext context) {
                          return LayoutBuilder(
                            builder: (BuildContext context, BoxConstraints constraints) {
                              layoutCount += 1;
                              expect(
                                tester.renderObject(find.byType(Overlay)).debugNeedsLayout,
                                false,
                              );
                              return WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
                            },
                          );
                        },
                        child: WidgetToRenderBoxAdapter(renderBox: childBox),
                      );
                    },
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
    expect(layoutCount, 0);
    expect(overlayLayoutCounter.layoutCount, 1);
    verifyTreeIsClean();

    // Add overlay child.
    controller1.show();
    await tester.pump();
    expect(layoutCount, 1);
    expect(overlayLayoutCounter.layoutCount, 1);
    verifyTreeIsClean();

    // Remove the overlay child.
    controller1.hide();
    await tester.pump();
    expect(layoutCount, 1);
    expect(overlayLayoutCounter.layoutCount, 1);
    verifyTreeIsClean();
  });

  group('Adding/removing overlay child causes repaint', () {
    // Regression test for https://github.com/flutter/flutter/issues/134656.
    const Key childKey = Key('child');
    final OverlayEntry overlayEntry = OverlayEntry(
      builder: (BuildContext context) {
        return RepaintBoundary(
          child: OverlayPortal(
            controller: controller1,
            overlayChildBuilder: (BuildContext context) => const SizedBox(),
            child: const SizedBox(key: childKey),
          ),
        );
      },
    );
    final Widget widget = Directionality(
      key: GlobalKey(debugLabel: 'key'),
      textDirection: TextDirection.ltr,
      child: Overlay(initialEntries: <OverlayEntry>[overlayEntry]),
    );
    tearDown(overlayEntry.remove);
    tearDownAll(overlayEntry.dispose);

    testWidgets('Adding child', (WidgetTester tester) async {
      controller1.hide();
      await tester.pumpWidget(widget);

      final RenderBox renderTheater = tester.renderObject<RenderBox>(find.byType(Overlay));
      final RenderBox renderChild = tester.renderObject<RenderBox>(find.byKey(childKey));
      assert(!renderTheater.debugNeedsPaint);
      assert(!renderChild.debugNeedsPaint);

      controller1.show();
      await tester.pump(null, EnginePhase.layout);
      expect(renderTheater.debugNeedsPaint, isTrue);
      expect(renderChild.debugNeedsPaint, isFalse);

      // Discard the dirty render tree.
      await tester.pumpWidget(const SizedBox());
    });

    testWidgets('Removing child', (WidgetTester tester) async {
      controller1.show();
      await tester.pumpWidget(widget);

      final RenderBox renderTheater = tester.renderObject<RenderBox>(find.byType(Overlay));
      final RenderBox renderChild = tester.renderObject<RenderBox>(find.byKey(childKey));
      assert(!renderTheater.debugNeedsPaint);
      assert(!renderChild.debugNeedsPaint);

      controller1.hide();
      await tester.pump(null, EnginePhase.layout);
      expect(renderTheater.debugNeedsPaint, isTrue);
      expect(renderChild.debugNeedsPaint, isFalse);

      // Discard the dirty render tree.
      await tester.pumpWidget(const SizedBox());
    });
  });

  testWidgets('Adding/Removing OverlayPortal in LayoutBuilder during layout', (
    WidgetTester tester,
  ) async {
    final GlobalKey widgetKey = GlobalKey(debugLabel: 'widget');
    final GlobalKey overlayKey = GlobalKey(debugLabel: 'overlay');
    controller1.hide();
    late StateSetter setState;
    late final OverlayEntry overlayEntry;
    addTearDown(
      () =>
          overlayEntry
            ..remove()
            ..dispose(),
    );
    Size size = Size.zero;

    final Widget overlayPortal = OverlayPortal(
      key: widgetKey,
      controller: controller1,
      overlayChildBuilder: (BuildContext context) => const Placeholder(),
      child: const Placeholder(),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          key: overlayKey,
          initialEntries: <OverlayEntry>[
            overlayEntry = OverlayEntry(
              builder: (BuildContext context) {
                return StatefulBuilder(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState = stateSetter;
                    return Center(
                      child: SizedBox.fromSize(
                        size: size,
                        child: LayoutBuilder(
                          builder: (BuildContext context, BoxConstraints constraints) {
                            // This layout callback adds/removes an OverlayPortal during layout.
                            return constraints.maxHeight > 0 ? overlayPortal : const SizedBox();
                          },
                        ),
                      ),
                    );
                  },
                );
              },
            ),
          ],
        ),
      ),
    );
    controller1.show();
    await tester.pump();
    expect(tester.takeException(), isNull);

    // Adds the OverlayPortal from within a LayoutBuilder, in a layout callback.
    setState(() {
      size = const Size(300, 300);
    });
    await tester.pump();
    expect(tester.takeException(), isNull);

    // Removes the OverlayPortal from within a LayoutBuilder, in a layout callback.
    setState(() {
      size = Size.zero;
    });
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  testWidgets('Change overlay constraints', (WidgetTester tester) async {
    final GlobalKey widgetKey = GlobalKey(debugLabel: 'widget outer');
    final GlobalKey overlayKey = GlobalKey(debugLabel: 'overlay');
    final RenderBox childBox = RenderConstrainedBox(additionalConstraints: const BoxConstraints());
    final RenderBox overlayChildBox = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints(),
    );
    final _RenderLayoutCounter overlayLayoutCounter = _RenderLayoutCounter();
    int layoutCount = 0;
    late StateSetter setState;
    double dimension = 100;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter stateSetter) {
          setState = stateSetter;
          return Center(
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: SizedBox.square(
                dimension: dimension,
                child: Overlay(
                  key: overlayKey,
                  initialEntries: <OverlayEntry>[
                    // Overlay.performLayout calls layoutCounter.layout.
                    _buildOverlayEntry(
                      (BuildContext context) =>
                          WidgetToRenderBoxAdapter(renderBox: overlayLayoutCounter),
                    ),
                    _buildOverlayEntry((BuildContext outerEntryContext) {
                      return Center(
                        child: _ManyRelayoutBoundaries(
                          levels: 50,
                          child: Builder(
                            builder: (BuildContext context) {
                              return OverlayPortal(
                                key: widgetKey,
                                controller: controller1,
                                overlayChildBuilder: (BuildContext context) {
                                  return LayoutBuilder(
                                    builder: (BuildContext context, BoxConstraints constraints) {
                                      layoutCount += 1;
                                      // Both overlays need to be clean at this point.
                                      expect(
                                        tester.renderObjectList(find.byType(Overlay)),
                                        everyElement(
                                          wrapMatcher(
                                            (RenderObject object) =>
                                                !object.debugNeedsLayout ||
                                                object.debugDoingThisLayout,
                                          ),
                                        ),
                                      );
                                      return WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
                                    },
                                  );
                                },
                                child: WidgetToRenderBoxAdapter(renderBox: childBox),
                              );
                            },
                          ),
                        ),
                      );
                    }),
                  ],
                ),
              ),
            ),
          );
        },
      ),
    );

    expect(layoutCount, 1);
    expect(overlayLayoutCounter.layoutCount, 1);
    expect(childBox.size, const Size.square(50));
    expect(overlayChildBox.size, const Size.square(100));
    verifyTreeIsClean();

    // The incoming constraints changed.
    setState(() {
      dimension = 150;
    });
    await tester.pump();
    expect(childBox.size, const Size.square(50));
    expect(overlayChildBox.size, const Size.square(150));

    expect(layoutCount, 2);
    expect(overlayLayoutCounter.layoutCount, 2);
    verifyTreeIsClean();
  });

  testWidgets('Can target the root overlay', (WidgetTester tester) async {
    final GlobalKey widgetKey = GlobalKey(debugLabel: 'widget outer');
    final GlobalKey rootOverlayKey = GlobalKey(debugLabel: 'root overlay');
    final GlobalKey localOverlayKey = GlobalKey(debugLabel: 'local overlay');
    final RenderBox childBox = RenderConstrainedBox(additionalConstraints: const BoxConstraints());
    final RenderBox overlayChildBox = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints(),
    );
    final _RenderLayoutCounter overlayLayoutCounter = _RenderLayoutCounter();
    int layoutCount = 0;
    OverlayPortal Function({
      Widget? child,
      required OverlayPortalController controller,
      Key? key,
      required WidgetBuilder overlayChildBuilder,
    })
    constructorToUse = OverlayPortal.new;
    late StateSetter setState;

    // This tree has 3 nested Overlays.
    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter stateSetter) {
          setState = stateSetter;
          return Center(
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Overlay(
                key: rootOverlayKey,
                initialEntries: <OverlayEntry>[
                  _buildOverlayEntry((BuildContext context) {
                    return Overlay(
                      initialEntries: <OverlayEntry>[
                        _buildOverlayEntry((BuildContext context) {
                          return Overlay(
                            key: localOverlayKey,
                            initialEntries: <OverlayEntry>[
                              // Overlay.performLayout calls layoutCounter.layout.
                              _buildOverlayEntry(
                                (BuildContext context) =>
                                    WidgetToRenderBoxAdapter(renderBox: overlayLayoutCounter),
                              ),
                              _buildOverlayEntry((BuildContext outerEntryContext) {
                                return Center(
                                  child: Builder(
                                    builder: (BuildContext context) {
                                      return constructorToUse(
                                        key: widgetKey,
                                        controller: controller1,
                                        overlayChildBuilder: (BuildContext context) {
                                          return LayoutBuilder(
                                            builder: (
                                              BuildContext context,
                                              BoxConstraints constraints,
                                            ) {
                                              layoutCount += 1;
                                              // Both overlays need to be clean at this point.
                                              expect(
                                                tester.renderObjectList(find.byType(Overlay)),
                                                everyElement(
                                                  wrapMatcher(
                                                    (RenderObject object) =>
                                                        !object.debugNeedsLayout ||
                                                        object.debugDoingThisLayout,
                                                  ),
                                                ),
                                              );
                                              return WidgetToRenderBoxAdapter(
                                                renderBox: overlayChildBox,
                                              );
                                            },
                                          );
                                        },
                                        child: WidgetToRenderBoxAdapter(renderBox: childBox),
                                      );
                                    },
                                  ),
                                );
                              }),
                            ],
                          );
                        }),
                      ],
                    );
                  }),
                ],
              ),
            ),
          );
        },
      ),
    );

    expect(layoutCount, 1);
    expect(overlayLayoutCounter.layoutCount, 1);
    expect(_ancestorRenderTheaters(overlayChildBox).length, 3);

    verifyTreeIsClean();

    // Now targets the root overlay.
    setState(() {
      constructorToUse = OverlayPortal.targetsRootOverlay;
    });
    await tester.pump();

    expect(layoutCount, 2);
    expect(overlayLayoutCounter.layoutCount, 1);
    expect(
      _ancestorRenderTheaters(overlayChildBox).single,
      tester.renderObject(find.byKey(rootOverlayKey)),
    );
    verifyTreeIsClean();
  });

  group('GlobalKey Reparenting', () {
    testWidgets('child is laid out before overlay child after OverlayEntry shuffle', (
      WidgetTester tester,
    ) async {
      int layoutCount = 0;

      final GlobalKey widgetKey = GlobalKey(debugLabel: 'widget');
      final RenderBox childBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final RenderBox overlayChildBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final OverlayEntry overlayEntry1 = OverlayEntry(
        builder: (BuildContext context) {
          return _ManyRelayoutBoundaries(
            levels: 50,
            child: Builder(
              builder: (BuildContext context) {
                return OverlayPortal(
                  key: widgetKey,
                  controller: controller1,
                  overlayChildBuilder: (BuildContext context) {
                    return LayoutBuilder(
                      builder: (BuildContext context, BoxConstraints constraints) {
                        layoutCount += 1;
                        verifyOverlayChildReadyForLayout(widgetKey);
                        return WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
                      },
                    );
                  },
                  child: WidgetToRenderBoxAdapter(renderBox: childBox),
                );
              },
            ),
          );
        },
      );
      addTearDown(
        () =>
            overlayEntry1
              ..remove()
              ..dispose(),
      );
      final OverlayEntry overlayEntry2 = OverlayEntry(
        builder: (BuildContext context) => const Placeholder(),
      );
      addTearDown(
        () =>
            overlayEntry2
              ..remove()
              ..dispose(),
      );
      final OverlayEntry overlayEntry3 = OverlayEntry(
        builder: (BuildContext context) => const Placeholder(),
      );
      addTearDown(
        () =>
            overlayEntry3
              ..remove()
              ..dispose(),
      );

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[overlayEntry1, overlayEntry2, overlayEntry3],
          ),
        ),
      );
      expect(layoutCount, 1);
      verifyTreeIsClean();

      widgetKey.currentContext!.findRenderObject()!.markNeedsLayout();
      childBox.markNeedsLayout();
      rebuildLayoutBuilderSubtree(overlayChildBox, tester);
      // Make sure childBox's depth is greater than that of the overlay child.
      expect(widgetKey.currentContext!.findRenderObject()!.depth, lessThan(overlayChildBox.depth));

      tester.state<OverlayState>(find.byType(Overlay)).rearrange(<OverlayEntry>[
        overlayEntry3,
        overlayEntry2,
        overlayEntry1,
      ]);
      await tester.pump();
      expect(layoutCount, 2);
      expect(widgetKey.currentContext!.findRenderObject()!.depth, lessThan(overlayChildBox.depth));
      verifyTreeIsClean();
    });

    testWidgets('widget is laid out before overlay child after reparenting', (
      WidgetTester tester,
    ) async {
      final GlobalKey targetGlobalKey = GlobalKey(debugLabel: 'target widget');
      final RenderBox childBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final RenderBox overlayChildBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );

      late StateSetter setState1, setState2;
      bool targetMovedToOverlayEntry3 = false;

      int layoutCount1 = 0;
      int layoutCount2 = 0;

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[
              _buildOverlayEntry((BuildContext context) {
                return _ManyRelayoutBoundaries(
                  levels: 50,
                  child: StatefulBuilder(
                    builder: (BuildContext context, StateSetter stateSetter) {
                      setState1 = stateSetter;
                      return targetMovedToOverlayEntry3
                          ? const SizedBox()
                          : OverlayPortal(
                            key: targetGlobalKey,
                            controller: controller1,
                            overlayChildBuilder: (BuildContext context) {
                              return LayoutBuilder(
                                builder: (BuildContext context, BoxConstraints constraints) {
                                  layoutCount1 += 1;
                                  verifyOverlayChildReadyForLayout(targetGlobalKey);
                                  return WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
                                },
                              );
                            },
                            child: WidgetToRenderBoxAdapter(renderBox: childBox),
                          );
                    },
                  ),
                );
              }),
              _buildOverlayEntry((BuildContext context) => const Placeholder()),
              _buildOverlayEntry((BuildContext context) {
                return SizedBox(
                  child: StatefulBuilder(
                    builder: (BuildContext context, StateSetter stateSetter) {
                      setState2 = stateSetter;
                      return !targetMovedToOverlayEntry3
                          ? const SizedBox()
                          : OverlayPortal(
                            key: targetGlobalKey,
                            controller: controller1,
                            overlayChildBuilder: (BuildContext context) {
                              return LayoutBuilder(
                                builder: (BuildContext context, BoxConstraints constraints) {
                                  layoutCount2 += 1;
                                  verifyOverlayChildReadyForLayout(targetGlobalKey);
                                  return WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
                                },
                              );
                            },
                            child: WidgetToRenderBoxAdapter(renderBox: childBox),
                          );
                    },
                  ),
                );
              }),
            ],
          ),
        ),
      );

      expect(layoutCount1, 1);
      expect(layoutCount2, 0);

      targetGlobalKey.currentContext!.findRenderObject()!.markNeedsLayout();
      childBox.markNeedsLayout();
      rebuildLayoutBuilderSubtree(overlayChildBox, tester);
      setState1(() {});
      setState2(() {});
      targetMovedToOverlayEntry3 = true;

      await tester.pump();

      expect(
        targetGlobalKey.currentContext!.findRenderObject()!.depth,
        lessThan(overlayChildBox.depth),
      );
      verifyTreeIsClean();
      expect(layoutCount1, 1);
      expect(layoutCount2, 1);
    });

    testWidgets('Swap child and overlayChild', (WidgetTester tester) async {
      final RenderBox childBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final RenderBox overlayChildBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );

      late StateSetter setState;
      bool swapChildAndRemoteChild = false;

      // WidgetToRenderBoxAdapter has its own builtin GlobalKey.
      final Widget child1 = WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
      final Widget child2 = WidgetToRenderBoxAdapter(renderBox: childBox);

      late final OverlayEntry overlayEntry;
      addTearDown(
        () =>
            overlayEntry
              ..remove()
              ..dispose(),
      );

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[
              overlayEntry = OverlayEntry(
                builder: (BuildContext context) {
                  return _ManyRelayoutBoundaries(
                    levels: 50,
                    child: StatefulBuilder(
                      builder: (BuildContext context, StateSetter stateSetter) {
                        setState = stateSetter;
                        return OverlayPortal(
                          controller: controller1,
                          overlayChildBuilder:
                              (BuildContext context) => swapChildAndRemoteChild ? child1 : child2,
                          child: swapChildAndRemoteChild ? child2 : child1,
                        );
                      },
                    ),
                  );
                },
              ),
            ],
          ),
        ),
      );

      setState(() {
        swapChildAndRemoteChild = true;
      });
      await tester.pump();
      verifyTreeIsClean();
    });

    testWidgets('forgetChild', (WidgetTester tester) async {
      final RenderBox childBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final RenderBox overlayChildBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );

      late StateSetter setState1;
      late StateSetter setState2;
      bool takeChildren = false;

      // WidgetToRenderBoxAdapter has its own builtin GlobalKey.
      final Widget child1 = WidgetToRenderBoxAdapter(renderBox: overlayChildBox);
      final Widget child2 = WidgetToRenderBoxAdapter(renderBox: childBox);

      late final OverlayEntry overlayEntry1;
      addTearDown(
        () =>
            overlayEntry1
              ..remove()
              ..dispose(),
      );
      late final OverlayEntry overlayEntry2;
      addTearDown(
        () =>
            overlayEntry2
              ..remove()
              ..dispose(),
      );

      controller1.hide();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[
              overlayEntry1 = OverlayEntry(
                builder: (BuildContext context) {
                  return StatefulBuilder(
                    builder: (BuildContext context, StateSetter stateSetter) {
                      setState2 = stateSetter;
                      return OverlayPortal(
                        controller: controller1,
                        overlayChildBuilder: (BuildContext context) => child2,
                        child: takeChildren ? child1 : null,
                      );
                    },
                  );
                },
              ),
              overlayEntry2 = OverlayEntry(
                builder: (BuildContext context) {
                  return _ManyRelayoutBoundaries(
                    levels: 50,
                    child: StatefulBuilder(
                      builder: (BuildContext context, StateSetter stateSetter) {
                        setState1 = stateSetter;
                        return OverlayPortal(
                          controller: controller2,
                          overlayChildBuilder: (BuildContext context) => child1,
                          child: takeChildren ? null : child2,
                        );
                      },
                    ),
                  );
                },
              ),
            ],
          ),
        ),
      );

      controller1.show();
      controller2.hide();
      setState2(() {
        takeChildren = true;
      });
      setState1(() {});

      await tester.pump();
      verifyTreeIsClean();
    });

    testWidgets('Nested overlay children: swap inner and outer', (WidgetTester tester) async {
      final GlobalKey outerKey = GlobalKey(debugLabel: 'Original Outer Widget');
      final GlobalKey innerKey = GlobalKey(debugLabel: 'Original Inner Widget');

      final RenderBox child1Box = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final RenderBox child2Box = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final RenderBox overlayChildBox = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      addTearDown(overlayChildBox.dispose);

      late StateSetter setState;
      bool swapped = false;

      // WidgetToRenderBoxAdapter has its own builtin GlobalKey.
      final Widget child1 = WidgetToRenderBoxAdapter(renderBox: child1Box);
      final Widget child2 = WidgetToRenderBoxAdapter(renderBox: child2Box);
      final Widget child3 = WidgetToRenderBoxAdapter(renderBox: overlayChildBox);

      late final OverlayEntry entry;
      addTearDown(() {
        entry.remove();
        entry.dispose();
      });

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[
              entry = OverlayEntry(
                builder: (BuildContext context) {
                  return StatefulBuilder(
                    builder: (BuildContext context, StateSetter stateSetter) {
                      setState = stateSetter;
                      return OverlayPortal(
                        key: swapped ? outerKey : innerKey,
                        controller: swapped ? controller2 : controller1,
                        overlayChildBuilder: (BuildContext context) {
                          return OverlayPortal(
                            key: swapped ? innerKey : outerKey,
                            controller: swapped ? controller1 : controller2,
                            overlayChildBuilder: (BuildContext context) {
                              return OverlayPortal(
                                controller: OverlayPortalController(),
                                overlayChildBuilder: (BuildContext context) => child3,
                              );
                            },
                            child: child2,
                          );
                        },
                        child: child1,
                      );
                    },
                  );
                },
              ),
            ],
          ),
        ),
      );

      setState(() {
        swapped = true;
      });
      await tester.pump();
      verifyTreeIsClean();
    });

    testWidgets('Paint order', (WidgetTester tester) async {
      final GlobalKey outerKey = GlobalKey(debugLabel: 'Original Outer Widget');
      final GlobalKey innerKey = GlobalKey(debugLabel: 'Original Inner Widget');

      late StateSetter setState;

      const Widget child1 = _PaintOrder();
      const Widget child2 = _PaintOrder();
      const Widget child3 = _PaintOrder();
      const Widget child4 = _PaintOrder();
      controller1.show();
      controller2.show();
      controller3.show();
      controller4.show();

      // Expected Order child1 -> innerKey -> child4.
      Widget widget = Column(
        children: <Widget>[
          OverlayPortal(
            controller: controller1,
            overlayChildBuilder: (BuildContext context) => child1,
          ),
          OverlayPortal(
            key: outerKey,
            controller: controller2,
            overlayChildBuilder: (BuildContext context) {
              return OverlayPortal(
                key: innerKey,
                controller: controller3,
                overlayChildBuilder: (BuildContext context) => child3,
                child: child2,
              );
            },
          ),
          OverlayPortal(
            controller: controller4,
            overlayChildBuilder: (BuildContext context) => child4,
          ),
        ],
      );

      late final OverlayEntry overlayEntry;
      addTearDown(
        () =>
            overlayEntry
              ..remove()
              ..dispose(),
      );

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[
              overlayEntry = OverlayEntry(
                builder: (BuildContext context) {
                  return StatefulBuilder(
                    builder: (BuildContext context, StateSetter stateSetter) {
                      setState = stateSetter;
                      return widget;
                    },
                  );
                },
              ),
            ],
          ),
        ),
      );

      expect(_PaintOrder.paintOrder, <Widget>[child1, child2, child3, child4]);
      _PaintOrder.paintOrder.clear();

      // Swap the nested OverlayPortal.
      widget = Column(
        children: <Widget>[
          OverlayPortal(
            controller: controller1,
            overlayChildBuilder: (BuildContext context) => child1,
          ),
          OverlayPortal(
            key: innerKey,
            controller: controller3,
            overlayChildBuilder: (BuildContext context) {
              return OverlayPortal(
                key: outerKey,
                controller: controller2,
                overlayChildBuilder: (BuildContext context) => child3,
                child: child2,
              );
            },
          ),
          OverlayPortal(
            controller: controller4,
            overlayChildBuilder: (BuildContext context) => child4,
          ),
        ],
      );

      setState(() {});
      await tester.pump();

      expect(_PaintOrder.paintOrder, <Widget>[child1, child3, child2, child4]);
    });

    group('Swapping', () {
      StateSetter? setState1, setState2;
      bool swapped = false;

      void setState({required bool newValue}) {
        swapped = newValue;
        setState1?.call(() {});
        setState2?.call(() {});
      }

      tearDown(() {
        swapped = false;
        setState1 = null;
        setState2 = null;
      });

      testWidgets('between OverlayEntry & overlayChild', (WidgetTester tester) async {
        final _RenderLayoutCounter counter1 = _RenderLayoutCounter();
        final _RenderLayoutCounter counter2 = _RenderLayoutCounter();

        late final OverlayEntry overlayEntry1;
        addTearDown(
          () =>
              overlayEntry1
                ..remove()
                ..dispose(),
        );
        late final OverlayEntry overlayEntry2;
        addTearDown(
          () =>
              overlayEntry2
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              initialEntries: <OverlayEntry>[
                overlayEntry1 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState1 = stateSetter;
                    // WidgetToRenderBoxAdapter is keyed by the render box.
                    return WidgetToRenderBoxAdapter(renderBox: swapped ? counter2 : counter1);
                  },
                ),
                overlayEntry2 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState2 = stateSetter;
                    return OverlayPortal(
                      controller: controller1,
                      overlayChildBuilder:
                          (BuildContext context) =>
                              WidgetToRenderBoxAdapter(renderBox: swapped ? counter1 : counter2),
                      child: const SizedBox(),
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(counter1.layoutCount, 1);
        expect(counter2.layoutCount, 1);

        setState(newValue: true);
        await tester.pump();

        expect(counter1.layoutCount, 2);
        expect(counter2.layoutCount, 2);

        setState(newValue: false);
        await tester.pump();

        expect(counter1.layoutCount, 3);
        expect(counter2.layoutCount, 3);
      });

      testWidgets('between OverlayEntry & overlayChild, featuring LayoutBuilder', (
        WidgetTester tester,
      ) async {
        final _RenderLayoutCounter counter1 = _RenderLayoutCounter();
        final _RenderLayoutCounter counter2 = _RenderLayoutCounter();

        late final OverlayEntry overlayEntry1;
        addTearDown(
          () =>
              overlayEntry1
                ..remove()
                ..dispose(),
        );
        late final OverlayEntry overlayEntry2;
        addTearDown(
          () =>
              overlayEntry2
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              initialEntries: <OverlayEntry>[
                overlayEntry1 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState1 = stateSetter;
                    return WidgetToRenderBoxAdapter(renderBox: swapped ? counter2 : counter1);
                  },
                ),
                overlayEntry2 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState2 = stateSetter;
                    return LayoutBuilder(
                      builder: (BuildContext context, BoxConstraints constraints) {
                        return OverlayPortal(
                          controller: controller1,
                          overlayChildBuilder:
                              (BuildContext context) => WidgetToRenderBoxAdapter(
                                renderBox: swapped ? counter1 : counter2,
                              ),
                          child: const SizedBox(),
                        );
                      },
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(counter1.layoutCount, 1);
        expect(counter2.layoutCount, 1);

        setState(newValue: true);
        await tester.pump();

        expect(counter1.layoutCount, 2);
        expect(counter2.layoutCount, 2);

        setState(newValue: false);
        await tester.pump();

        expect(counter1.layoutCount, 3);
        expect(counter2.layoutCount, 3);
      });

      testWidgets('between overlayChild & overlayChild', (WidgetTester tester) async {
        final _RenderLayoutCounter counter1 = _RenderLayoutCounter();
        final _RenderLayoutCounter counter2 = _RenderLayoutCounter();

        late final OverlayEntry overlayEntry1;
        addTearDown(
          () =>
              overlayEntry1
                ..remove()
                ..dispose(),
        );
        late final OverlayEntry overlayEntry2;
        addTearDown(
          () =>
              overlayEntry2
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              initialEntries: <OverlayEntry>[
                overlayEntry1 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState1 = stateSetter;
                    return OverlayPortal(
                      // WidgetToRenderBoxAdapter is keyed by the render box.
                      controller: controller1,
                      overlayChildBuilder:
                          (BuildContext context) =>
                              WidgetToRenderBoxAdapter(renderBox: swapped ? counter2 : counter1),
                      child: const SizedBox(),
                    );
                  },
                ),
                overlayEntry2 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState2 = stateSetter;
                    return OverlayPortal(
                      controller: controller2,
                      overlayChildBuilder:
                          (BuildContext context) =>
                              WidgetToRenderBoxAdapter(renderBox: swapped ? counter1 : counter2),
                      child: const SizedBox(),
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(counter1.layoutCount, 1);
        expect(counter2.layoutCount, 1);

        setState(newValue: true);
        await tester.pump();

        expect(counter1.layoutCount, 2);
        expect(counter2.layoutCount, 2);

        setState(newValue: false);
        await tester.pump();

        expect(counter1.layoutCount, 3);
        expect(counter2.layoutCount, 3);
      });

      testWidgets('between overlayChild & overlayChild, featuring LayoutBuilder', (
        WidgetTester tester,
      ) async {
        final _RenderLayoutCounter counter1 = _RenderLayoutCounter();
        final _RenderLayoutCounter counter2 = _RenderLayoutCounter();

        late final OverlayEntry overlayEntry1;
        addTearDown(
          () =>
              overlayEntry1
                ..remove()
                ..dispose(),
        );
        late final OverlayEntry overlayEntry2;
        addTearDown(
          () =>
              overlayEntry2
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              initialEntries: <OverlayEntry>[
                overlayEntry1 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState1 = stateSetter;
                    return LayoutBuilder(
                      builder: (BuildContext context, BoxConstraints constraints) {
                        return OverlayPortal(
                          controller: controller1,
                          overlayChildBuilder:
                              (BuildContext context) => WidgetToRenderBoxAdapter(
                                renderBox: swapped ? counter2 : counter1,
                              ),
                          child: const SizedBox(),
                        );
                      },
                    );
                  },
                ),
                overlayEntry2 = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState2 = stateSetter;
                    return LayoutBuilder(
                      builder: (BuildContext context, BoxConstraints constraints) {
                        return OverlayPortal(
                          controller: controller2,
                          overlayChildBuilder:
                              (BuildContext context) => WidgetToRenderBoxAdapter(
                                renderBox: swapped ? counter1 : counter2,
                              ),
                          child: const SizedBox(),
                        );
                      },
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(counter1.layoutCount, 1);
        expect(counter2.layoutCount, 1);

        setState(newValue: true);
        await tester.pump();

        expect(counter1.layoutCount, 2);
        expect(counter2.layoutCount, 2);

        setState(newValue: false);
        await tester.pump();

        expect(counter1.layoutCount, 3);
        expect(counter2.layoutCount, 3);
      });

      testWidgets('between child & overlayChild', (WidgetTester tester) async {
        final _RenderLayoutCounter counter1 = _RenderLayoutCounter();
        final _RenderLayoutCounter counter2 = _RenderLayoutCounter();

        late final OverlayEntry overlayEntry;
        addTearDown(
          () =>
              overlayEntry
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              initialEntries: <OverlayEntry>[
                overlayEntry = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState1 = stateSetter;
                    return OverlayPortal(
                      // WidgetToRenderBoxAdapter is keyed by the render box.
                      controller: controller1,
                      overlayChildBuilder:
                          (BuildContext context) =>
                              WidgetToRenderBoxAdapter(renderBox: swapped ? counter2 : counter1),
                      child: WidgetToRenderBoxAdapter(renderBox: swapped ? counter1 : counter2),
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(counter1.layoutCount, 1);
        expect(counter2.layoutCount, 1);

        setState(newValue: true);
        await tester.pump();

        expect(counter1.layoutCount, 2);
        expect(counter2.layoutCount, 2);

        setState(newValue: false);
        await tester.pump();

        expect(counter1.layoutCount, 3);
        expect(counter2.layoutCount, 3);
      });

      testWidgets('between child & overlayChild, featuring LayoutBuilder', (
        WidgetTester tester,
      ) async {
        final _RenderLayoutCounter counter1 = _RenderLayoutCounter();
        final _RenderLayoutCounter counter2 = _RenderLayoutCounter();

        late final OverlayEntry overlayEntry;
        addTearDown(
          () =>
              overlayEntry
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              initialEntries: <OverlayEntry>[
                overlayEntry = OverlayStatefulEntry(
                  builder: (BuildContext context, StateSetter stateSetter) {
                    setState1 = stateSetter;
                    return LayoutBuilder(
                      builder: (BuildContext context, BoxConstraints constraints) {
                        return OverlayPortal(
                          // WidgetToRenderBoxAdapter is keyed by the render box.
                          controller: controller1,
                          overlayChildBuilder:
                              (BuildContext context) => WidgetToRenderBoxAdapter(
                                renderBox: swapped ? counter2 : counter1,
                              ),
                          child: WidgetToRenderBoxAdapter(renderBox: swapped ? counter1 : counter2),
                        );
                      },
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(counter1.layoutCount, 1);
        expect(counter2.layoutCount, 1);

        setState(newValue: true);
        await tester.pump();

        expect(counter1.layoutCount, 2);
        expect(counter2.layoutCount, 2);

        setState(newValue: false);
        await tester.pump();

        expect(counter1.layoutCount, 3);
        expect(counter2.layoutCount, 3);
      });
    });

    testWidgets(
      'Safe to move the overlay child to a different Overlay and remove the old Overlay',
      (WidgetTester tester) async {
        controller1.show();
        final GlobalKey key = GlobalKey(debugLabel: 'key');
        final GlobalKey oldOverlayKey = GlobalKey(debugLabel: 'old overlay');
        final GlobalKey newOverlayKey = GlobalKey(debugLabel: 'new overlay');
        final GlobalKey overlayChildKey = GlobalKey(debugLabel: 'overlay child key');

        late final OverlayEntry overlayEntry1;
        addTearDown(
          () =>
              overlayEntry1
                ..remove()
                ..dispose(),
        );
        late final OverlayEntry overlayEntry2;
        addTearDown(
          () =>
              overlayEntry2
                ..remove()
                ..dispose(),
        );

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              key: oldOverlayKey,
              initialEntries: <OverlayEntry>[
                overlayEntry1 = OverlayEntry(
                  builder: (BuildContext context) {
                    return OverlayPortal(
                      key: key,
                      controller: controller1,
                      overlayChildBuilder: (BuildContext context) => SizedBox(key: overlayChildKey),
                      child: const SizedBox(),
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(find.byKey(overlayChildKey), findsOneWidget);
        expect(find.byKey(newOverlayKey), findsNothing);
        expect(find.byKey(oldOverlayKey), findsOneWidget);

        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Overlay(
              key: newOverlayKey,
              initialEntries: <OverlayEntry>[
                overlayEntry2 = OverlayEntry(
                  builder: (BuildContext context) {
                    return OverlayPortal(
                      key: key,
                      controller: controller1,
                      overlayChildBuilder: (BuildContext context) => SizedBox(key: overlayChildKey),
                      child: const SizedBox(),
                    );
                  },
                ),
              ],
            ),
          ),
        );

        expect(tester.takeException(), isNull);
        expect(find.byKey(overlayChildKey), findsOneWidget);
        expect(find.byKey(newOverlayKey), findsOneWidget);
        expect(find.byKey(oldOverlayKey), findsNothing);
      },
    );
  });

  group('Paint order', () {
    testWidgets('show bringsToTop', (WidgetTester tester) async {
      controller1.hide();

      const _PaintOrder child1 = _PaintOrder();
      const _PaintOrder child2 = _PaintOrder();

      late final OverlayEntry overlayEntry;
      addTearDown(
        () =>
            overlayEntry
              ..remove()
              ..dispose(),
      );

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[
              overlayEntry = OverlayEntry(
                builder: (BuildContext context) {
                  return Column(
                    children: <Widget>[
                      OverlayPortal(
                        controller: controller1,
                        overlayChildBuilder: (BuildContext context) => child1,
                      ),
                      OverlayPortal(
                        controller: controller2,
                        overlayChildBuilder: (BuildContext context) => child2,
                      ),
                    ],
                  );
                },
              ),
            ],
          ),
        ),
      );

      // Only child2 is visible.
      expect(_PaintOrder.paintOrder, <_PaintOrder>[child2]);

      _PaintOrder.paintOrder.clear();
      controller1.show();
      await tester.pump();
      expect(_PaintOrder.paintOrder, <_PaintOrder>[child2, child1]);

      _PaintOrder.paintOrder.clear();
      controller2.show();
      await tester.pump();
      expect(_PaintOrder.paintOrder, <_PaintOrder>[child1, child2]);

      _PaintOrder.paintOrder.clear();
      controller2.hide();
      controller1.hide();
      await tester.pump();
      expect(_PaintOrder.paintOrder, isEmpty);
    });

    testWidgets('Paint order does not change after global key reparenting', (
      WidgetTester tester,
    ) async {
      final GlobalKey key = GlobalKey();

      late StateSetter setState;
      bool reparented = false;

      // WidgetToRenderBoxAdapter has its own builtin GlobalKey.
      final RenderBox child1Box = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final RenderBox child2Box = RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(),
      );
      final Widget child1 = WidgetToRenderBoxAdapter(renderBox: child1Box);
      final Widget child2 = WidgetToRenderBoxAdapter(renderBox: child2Box);

      final Widget overlayPortal1 = OverlayPortal(
        key: key,
        controller: controller1,
        overlayChildBuilder: (BuildContext context) => child1,
        child: const SizedBox(),
      );

      final Widget overlayPortal2 = OverlayPortal(
        controller: controller2,
        overlayChildBuilder: (BuildContext context) => child2,
        child: const SizedBox(),
      );

      late final OverlayEntry overlayEntry;
      addTearDown(
        () =>
            overlayEntry
              ..remove()
              ..dispose(),
      );

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Overlay(
            initialEntries: <OverlayEntry>[
              overlayEntry = OverlayEntry(
                builder: (BuildContext context) {
                  return Column(
                    children: <Widget>[
                      StatefulBuilder(
                        builder: (BuildContext context, StateSetter stateSetter) {
                          setState = stateSetter;
                          return reparented ? SizedBox(child: overlayPortal1) : overlayPortal1;
                        },
                      ),
                      overlayPortal2,
                    ],
                  );
                },
              ),
            ],
          ),
        ),
      );

      final RenderObject theater = tester.renderObject<RenderObject>(find.byType(Overlay));
      final List<RenderObject> childrenVisited = <RenderObject>[];
      theater.visitChildren(childrenVisited.add);
      expect(childrenVisited.length, 3);
      expect(
        childrenVisited,
        containsAllInOrder(<RenderObject>[child1Box.parent!, child2Box.parent!]),
      );
      childrenVisited.clear();

      setState(() {
        reparented = true;
      });
      await tester.pump();
      theater.visitChildren(childrenVisited.add);
      // The child list stays the same.
      expect(
        childrenVisited,
        containsAllInOrder(<RenderObject>[child1Box.parent!, child2Box.parent!]),
      );
    });
  });

  group('Semantics', () {
    testWidgets('ordering and transform', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      final double rowOriginY = TestSemantics.fullScreen.height - 10;

      late final OverlayEntry entry;
      addTearDown(() {
        entry.remove();
        entry.dispose();
      });

      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder: (BuildContext context) {
                return DefaultTextStyle(
                  style: const TextStyle(fontSize: 10),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    crossAxisAlignment: CrossAxisAlignment.end,
                    children: <Widget>[
                      Semantics(
                        container: true,
                        explicitChildNodes: true,
                        child: OverlayPortal(
                          controller: controller1,
                          overlayChildBuilder:
                              (BuildContext context) =>
                                  const Positioned(left: 0.0, top: 0.0, child: Text('BBBB')),
                          child: const Text('A'),
                        ),
                      ),
                      const Text('CC'),
                    ],
                  ),
                );
              },
            ),
          ],
        ),
      );

      await tester.pumpWidget(widget);
      final Matrix4 node1Transform =
          Matrix4.identity()
            ..scale(3.0, 3.0, 1.0)
            ..translate(0.0, TestSemantics.fullScreen.height - 10.0);
      final Matrix4 node4Transform = node1Transform.clone()..translate(10.0);

      final TestSemantics expected = TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics(
            id: 1,
            rect: Offset.zero & const Size(10, 10),
            transform: node1Transform,
            children: <TestSemantics>[
              TestSemantics(id: 2, label: 'A', rect: Offset.zero & const Size(10, 10)),
              // The crossAxisAlignment is set to `end`. The size of node 1 is 30 x 10.
              TestSemantics(
                id: 3,
                label: 'BBBB',
                rect: Offset.zero & const Size(40, 10),
                transform: Matrix4.translationValues(0, -rowOriginY, 0),
              ),
            ],
          ),
          TestSemantics(
            id: 4,
            label: 'CC',
            rect: Offset.zero & const Size(20, 10),
            transform: node4Transform,
          ),
        ],
      );

      expect(semantics, hasSemantics(expected));
      semantics.dispose();
    });

    testWidgets('OverlayPortal overlay child clipping', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      late final OverlayEntry entry;
      addTearDown(() {
        entry.remove();
        entry.dispose();
      });

      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder: (BuildContext context) {
                return DefaultTextStyle(
                  style: const TextStyle(fontSize: 10),
                  child: ListView(
                    children: <Widget>[
                      // Clips OverlayPortal, making it only half visible.
                      SizedBox(height: TestSemantics.fullScreen.height - 5),
                      Semantics(
                        container: true,
                        explicitChildNodes: true,
                        child: OverlayPortal(
                          controller: controller1,
                          overlayChildBuilder: (BuildContext context) {
                            return Positioned(
                              left: 0,
                              right: 0,
                              top: 0,
                              height: 10,
                              child: ListView(
                                children: const <Widget>[
                                  SizedBox(height: 3), // Clips B so it's only 7 pixels tall.
                                  Text('B'),
                                ],
                              ),
                            );
                          },
                          child: const Text('A'),
                        ),
                      ),
                    ],
                  ),
                );
              },
            ),
          ],
        ),
      );
      await tester.pumpWidget(widget);

      final SemanticsNode clippedOverlayPortal = semantics.nodesWith(label: 'A').single;
      final SemanticsNode clippedOverlayChild = semantics.nodesWith(label: 'B').single;

      expect(clippedOverlayPortal.rect, Offset.zero & const Size(800, 5));
      expect(clippedOverlayChild.rect, Offset.zero & const Size(800, 7));

      expect(clippedOverlayPortal.transform, isNull);
      // The parent SemanticsNode is created by the ListView.
      expect(clippedOverlayChild.transform, Matrix4.translationValues(0.0, 3.0, 0.0));

      semantics.dispose();
    });

    testWidgets("OverlayPortal's semantics node is hidden", (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      late final OverlayEntry entry;
      addTearDown(() {
        entry.remove();
        entry.dispose();
      });

      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder: (BuildContext context) {
                return DefaultTextStyle(
                  style: const TextStyle(fontSize: 10),
                  child: ListView(
                    children: <Widget>[
                      // Clips OverlayPortal, making it completely invisible.
                      SizedBox(height: TestSemantics.fullScreen.height),
                      Semantics(
                        container: true,
                        explicitChildNodes: true,
                        child: OverlayPortal(
                          controller: controller1,
                          overlayChildBuilder: (BuildContext context) {
                            return const Positioned(left: 0, top: 0, child: Text('B'));
                          },
                          child: const Text('A'),
                        ),
                      ),
                    ],
                  ),
                );
              },
            ),
          ],
        ),
      );
      await tester.pumpWidget(widget);

      final SemanticsNode clippedOverlayPortal = semantics.nodesWith(label: 'A').single;
      final SemanticsNode clippedOverlayChild = semantics.nodesWith(label: 'B').single;

      expect(clippedOverlayPortal.rect, Offset.zero & const Size(800, 10));
      expect(clippedOverlayChild.rect, Offset.zero & const Size(10, 10));

      expect(clippedOverlayPortal.transform, isNull);
      // The parent SemanticsNode is created by OverlayPortal.
      expect(clippedOverlayChild.transform, Matrix4.translationValues(0.0, -600.0, 0.0));

      semantics.dispose();
    });

    testWidgets("OverlayPortal's semantics node is dropped but the element is kept alive", (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      final ScrollController controller = ScrollController(initialScrollOffset: 10);
      addTearDown(controller.dispose);

      late final OverlayEntry entry;
      addTearDown(() {
        entry.remove();
        entry.dispose();
      });

      final Widget widget = Directionality(
        textDirection: TextDirection.ltr,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder: (BuildContext context) {
                return DefaultTextStyle(
                  style: const TextStyle(fontSize: 10),
                  child: ListView(
                    controller: controller,
                    cacheExtent: 0,
                    addAutomaticKeepAlives: false,
                    addRepaintBoundaries: false,
                    addSemanticIndexes: false,
                    children: <Widget>[
                      // Clips OverlayPortal, making it completely invisible.
                      SizedBox(height: TestSemantics.fullScreen.height),
                      KeepAlive(
                        keepAlive: true,
                        child: Semantics(
                          container: true,
                          explicitChildNodes: true,
                          child: OverlayPortal(
                            controller: controller1,
                            overlayChildBuilder: (BuildContext context) {
                              return const Positioned(left: 0, top: 0, child: Text('B'));
                            },
                            child: const Text('A'),
                          ),
                        ),
                      ),
                    ],
                  ),
                );
              },
            ),
          ],
        ),
      );
      await tester.pumpWidget(widget);
      expect(semantics.nodesWith(label: 'A'), isNotEmpty);
      expect(semantics.nodesWith(label: 'B'), isNotEmpty);

      controller.jumpTo(0);
      await tester.pump();

      expect(semantics.nodesWith(label: 'A'), isEmpty);
      expect(semantics.nodesWith(label: 'B'), isEmpty);
      semantics.dispose();

      final RenderObject overlayRenderObject = tester.renderObject(find.byType(Overlay));
      // Paints 'B' but not both 'A' and 'B'.
      expect(overlayRenderObject, paints..paragraph());
      expect(
        overlayRenderObject,
        isNot(
          paints
            ..paragraph()
            ..paragraph(),
        ),
      );
    });
  });
}

class OverlayStatefulEntry extends OverlayEntry {
  OverlayStatefulEntry({required StatefulWidgetBuilder builder})
    : super(builder: (BuildContext context) => StatefulBuilder(builder: builder));
}

class _RenderLayoutCounter extends RenderProxyBox {
  int layoutCount = 0;
  bool _parentDoingLayout = false;
  @override
  void layout(Constraints constraints, {bool parentUsesSize = false}) {
    assert(!_parentDoingLayout);
    _parentDoingLayout = true;
    layoutCount += 1;
    super.layout(constraints, parentUsesSize: parentUsesSize);
    _parentDoingLayout = false;
  }

  @override
  void performLayout() {
    super.performLayout();
    if (!_parentDoingLayout) {
      layoutCount += 1;
    }
  }
}

/// This helper makes leak tracker forgiving the entry is not disposed.
OverlayEntry _buildOverlayEntry(WidgetBuilder builder) => OverlayEntry(builder: builder);

class _PaintOrder extends SingleChildRenderObjectWidget {
  const _PaintOrder();
  static List<_PaintOrder> paintOrder = <_PaintOrder>[];

  void onPaint() => paintOrder.add(this);

  @override
  _RenderPaintRecorder createRenderObject(BuildContext context) =>
      _RenderPaintRecorder()..onPaint = onPaint;
  @override
  void updateRenderObject(BuildContext context, _RenderPaintRecorder renderObject) =>
      renderObject.onPaint = onPaint;
}

class _RenderPaintRecorder extends RenderProxyBox {
  VoidCallback? onPaint;

  @override
  void paint(PaintingContext context, Offset offset) {
    onPaint?.call();
    super.paint(context, offset);
  }
}
