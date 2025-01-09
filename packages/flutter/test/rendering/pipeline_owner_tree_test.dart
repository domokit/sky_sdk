// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  FlutterError.presentError = (FlutterErrorDetails details) {
    // Make tests fail on FlutterErrors.
    throw details.exception;
  };

  test('onNeedVisualUpdate takes precedence over manifold', () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    int rootOnNeedVisualUpdateCallCount = 0;
    final TestRenderObject rootRenderObject = TestRenderObject();
    final PipelineOwner root = PipelineOwner(
      onNeedVisualUpdate: () {
        rootOnNeedVisualUpdateCallCount += 1;
      },
    );
    root.rootNode = rootRenderObject;
    rootRenderObject.scheduleInitialLayout();

    int child1OnNeedVisualUpdateCallCount = 0;
    final TestRenderObject child1RenderObject = TestRenderObject();
    final PipelineOwner child1 = PipelineOwner(
      onNeedVisualUpdate: () {
        child1OnNeedVisualUpdateCallCount += 1;
      },
    );
    child1.rootNode = child1RenderObject;
    child1RenderObject.scheduleInitialLayout();

    final TestRenderObject child2RenderObject = TestRenderObject();
    final PipelineOwner child2 = PipelineOwner();
    child2.rootNode = child2RenderObject;
    child2RenderObject.scheduleInitialLayout();

    root.adoptChild(child1);
    root.adoptChild(child2);
    root.attach(manifold);
    root.flushLayout();
    manifold.requestVisualUpdateCount = 0;

    rootRenderObject.markNeedsLayout();
    expect(manifold.requestVisualUpdateCount, 0);
    expect(rootOnNeedVisualUpdateCallCount, 1);
    expect(child1OnNeedVisualUpdateCallCount, 0);

    child1RenderObject.markNeedsLayout();
    expect(manifold.requestVisualUpdateCount, 0);
    expect(rootOnNeedVisualUpdateCallCount, 1);
    expect(child1OnNeedVisualUpdateCallCount, 1);

    child2RenderObject.markNeedsLayout();
    expect(manifold.requestVisualUpdateCount, 1);
    expect(rootOnNeedVisualUpdateCallCount, 1);
    expect(child1OnNeedVisualUpdateCallCount, 1);
  });

  test("parent's render objects are laid out before child's render objects", () {
    final TestPipelineManifold manifold = TestPipelineManifold();
    final List<String> log = <String>[];

    final TestRenderObject rootRenderObject = TestRenderObject(
      onLayout: () {
        log.add('layout parent');
      },
    );
    final PipelineOwner root = PipelineOwner();
    root.rootNode = rootRenderObject;
    rootRenderObject.scheduleInitialLayout();

    final TestRenderObject childRenderObject = TestRenderObject(
      onLayout: () {
        log.add('layout child');
      },
    );
    final PipelineOwner child = PipelineOwner();
    child.rootNode = childRenderObject;
    childRenderObject.scheduleInitialLayout();

    root.adoptChild(child);
    root.attach(manifold);
    expect(log, isEmpty);

    root.flushLayout();
    expect(log, <String>['layout parent', 'layout child']);
  });

  test("child cannot dirty parent's render object during flushLayout", () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    final TestRenderObject rootRenderObject = TestRenderObject();
    final PipelineOwner root = PipelineOwner();
    root.rootNode = rootRenderObject;
    rootRenderObject.scheduleInitialLayout();

    bool childLayoutExecuted = false;
    final TestRenderObject childRenderObject = TestRenderObject(
      onLayout: () {
        childLayoutExecuted = true;
        expect(() => rootRenderObject.markNeedsLayout(), throwsFlutterError);
      },
    );
    final PipelineOwner child = PipelineOwner();
    child.rootNode = childRenderObject;
    childRenderObject.scheduleInitialLayout();

    root.adoptChild(child);
    root.attach(manifold);

    root.flushLayout();
    expect(childLayoutExecuted, isTrue);
  });

  test('updates compositing bits on children', () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    final TestRenderObject rootRenderObject = TestRenderObject();
    final PipelineOwner root = PipelineOwner();
    root.rootNode = rootRenderObject;
    rootRenderObject.markNeedsCompositingBitsUpdate();

    final TestRenderObject childRenderObject = TestRenderObject();
    final PipelineOwner child = PipelineOwner();
    child.rootNode = childRenderObject;
    childRenderObject.markNeedsCompositingBitsUpdate();

    root.adoptChild(child);
    root.attach(manifold);
    expect(() => rootRenderObject.needsCompositing, throwsAssertionError);
    expect(() => childRenderObject.needsCompositing, throwsAssertionError);

    root.flushCompositingBits();
    expect(rootRenderObject.needsCompositing, isTrue);
    expect(childRenderObject.needsCompositing, isTrue);
  });

  test("parent's render objects are painted before child's render objects", () {
    final TestPipelineManifold manifold = TestPipelineManifold();
    final List<String> log = <String>[];

    final TestRenderObject rootRenderObject = TestRenderObject(
      onPaint: () {
        log.add('paint parent');
      },
    );
    final PipelineOwner root = PipelineOwner();
    root.rootNode = rootRenderObject;
    final OffsetLayer rootLayer = OffsetLayer();
    rootLayer.attach(rootRenderObject);
    rootRenderObject.scheduleInitialLayout();
    rootRenderObject.scheduleInitialPaint(rootLayer);

    final TestRenderObject childRenderObject = TestRenderObject(
      onPaint: () {
        log.add('paint child');
      },
    );
    final PipelineOwner child = PipelineOwner();
    child.rootNode = childRenderObject;
    final OffsetLayer childLayer = OffsetLayer();
    childLayer.attach(childRenderObject);
    childRenderObject.scheduleInitialLayout();
    childRenderObject.scheduleInitialPaint(childLayer);

    root.adoptChild(child);
    root.attach(manifold);
    root.flushLayout(); // Can't paint with invalid layout.
    expect(log, isEmpty);

    root.flushPaint();
    expect(log, <String>['paint parent', 'paint child']);
  });

  test("child paint cannot dirty parent's render object", () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    final TestRenderObject rootRenderObject = TestRenderObject();
    final PipelineOwner root = PipelineOwner();
    root.rootNode = rootRenderObject;
    final OffsetLayer rootLayer = OffsetLayer();
    rootLayer.attach(rootRenderObject);
    rootRenderObject.scheduleInitialLayout();
    rootRenderObject.scheduleInitialPaint(rootLayer);

    bool childPaintExecuted = false;
    final TestRenderObject childRenderObject = TestRenderObject(
      onPaint: () {
        childPaintExecuted = true;
        expect(() => rootRenderObject.markNeedsPaint(), throwsAssertionError);
      },
    );
    final PipelineOwner child = PipelineOwner();
    child.rootNode = childRenderObject;
    final OffsetLayer childLayer = OffsetLayer();
    childLayer.attach(childRenderObject);
    childRenderObject.scheduleInitialLayout();
    childRenderObject.scheduleInitialPaint(childLayer);

    root.adoptChild(child);
    root.attach(manifold);
    root.flushLayout(); // Can't paint with invalid layout.
    root.flushPaint();
    expect(childPaintExecuted, isTrue);
  });

  test("parent's render objects do semantics before child's render objects", () {
    final TestPipelineManifold manifold = TestPipelineManifold()..semanticsEnabled = true;
    final List<String> log = <String>[];

    final TestRenderObject rootRenderObject = TestRenderObject(
      onSemantics: () {
        log.add('semantics parent');
      },
    );
    final PipelineOwner root = PipelineOwner(
      onSemanticsOwnerCreated: () {
        rootRenderObject.scheduleInitialSemantics();
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
    );
    root.rootNode = rootRenderObject;

    final TestRenderObject childRenderObject = TestRenderObject(
      onSemantics: () {
        log.add('semantics child');
      },
    );
    final PipelineOwner child = PipelineOwner(
      onSemanticsOwnerCreated: () {
        childRenderObject.scheduleInitialSemantics();
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
    );
    child.rootNode = childRenderObject;

    root.adoptChild(child);
    root.attach(manifold);
    log.clear();

    rootRenderObject.markNeedsSemanticsUpdate();
    childRenderObject.markNeedsSemanticsUpdate();
    root.flushSemantics();
    expect(log, <String>['semantics parent', 'semantics child']);
  });

  test("child cannot mark parent's render object dirty during flushSemantics", () {
    final TestPipelineManifold manifold = TestPipelineManifold()..semanticsEnabled = true;

    final TestRenderObject rootRenderObject = TestRenderObject();
    final PipelineOwner root = PipelineOwner(
      onSemanticsOwnerCreated: () {
        rootRenderObject.scheduleInitialSemantics();
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
    );
    root.rootNode = rootRenderObject;

    bool childSemanticsCalled = false;
    final TestRenderObject childRenderObject = TestRenderObject(
      onSemantics: () {
        childSemanticsCalled = true;
        rootRenderObject.markNeedsSemanticsUpdate();
      },
    );
    final PipelineOwner child = PipelineOwner(
      onSemanticsOwnerCreated: () {
        childRenderObject.scheduleInitialSemantics();
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
    );
    child.rootNode = childRenderObject;

    root.adoptChild(child);
    root.attach(manifold);
    rootRenderObject.markNeedsSemanticsUpdate();
    childRenderObject.markNeedsSemanticsUpdate();
    root.flushSemantics();

    expect(childSemanticsCalled, isTrue);
  });

  test('when manifold enables semantics all PipelineOwners in tree create SemanticsOwner', () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    int rootOnSemanticsOwnerCreatedCount = 0;
    int rootOnSemanticsOwnerDisposed = 0;
    final PipelineOwner root = PipelineOwner(
      onSemanticsOwnerCreated: () {
        rootOnSemanticsOwnerCreatedCount++;
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
      onSemanticsOwnerDisposed: () {
        rootOnSemanticsOwnerDisposed++;
      },
    );

    int childOnSemanticsOwnerCreatedCount = 0;
    int childOnSemanticsOwnerDisposed = 0;
    final PipelineOwner child = PipelineOwner(
      onSemanticsOwnerCreated: () {
        childOnSemanticsOwnerCreatedCount++;
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
      onSemanticsOwnerDisposed: () {
        childOnSemanticsOwnerDisposed++;
      },
    );

    root.adoptChild(child);
    root.attach(manifold);
    expect(rootOnSemanticsOwnerCreatedCount, 0);
    expect(childOnSemanticsOwnerCreatedCount, 0);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNull);

    manifold.semanticsEnabled = true;

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);

    manifold.semanticsEnabled = false;

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 1);
    expect(childOnSemanticsOwnerDisposed, 1);
    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNull);
  });

  test(
    'when manifold enables semantics all PipelineOwners in tree that did not have a SemanticsOwner create one',
    () {
      final TestPipelineManifold manifold = TestPipelineManifold();

      int rootOnSemanticsOwnerCreatedCount = 0;
      int rootOnSemanticsOwnerDisposed = 0;
      final PipelineOwner root = PipelineOwner(
        onSemanticsOwnerCreated: () {
          rootOnSemanticsOwnerCreatedCount++;
        },
        onSemanticsUpdate: (SemanticsUpdate update) {},
        onSemanticsOwnerDisposed: () {
          rootOnSemanticsOwnerDisposed++;
        },
      );

      int childOnSemanticsOwnerCreatedCount = 0;
      int childOnSemanticsOwnerDisposed = 0;
      final PipelineOwner child = PipelineOwner(
        onSemanticsOwnerCreated: () {
          childOnSemanticsOwnerCreatedCount++;
        },
        onSemanticsUpdate: (SemanticsUpdate update) {},
        onSemanticsOwnerDisposed: () {
          childOnSemanticsOwnerDisposed++;
        },
      );

      root.adoptChild(child);
      root.attach(manifold);

      final SemanticsHandle childSemantics = child.ensureSemantics();
      expect(rootOnSemanticsOwnerCreatedCount, 0);
      expect(childOnSemanticsOwnerCreatedCount, 1);
      expect(rootOnSemanticsOwnerDisposed, 0);
      expect(childOnSemanticsOwnerDisposed, 0);
      expect(root.semanticsOwner, isNull);
      expect(child.semanticsOwner, isNotNull);

      manifold.semanticsEnabled = true;

      expect(rootOnSemanticsOwnerCreatedCount, 1);
      expect(childOnSemanticsOwnerCreatedCount, 1);
      expect(rootOnSemanticsOwnerDisposed, 0);
      expect(childOnSemanticsOwnerDisposed, 0);
      expect(root.semanticsOwner, isNotNull);
      expect(child.semanticsOwner, isNotNull);

      manifold.semanticsEnabled = false;

      expect(rootOnSemanticsOwnerCreatedCount, 1);
      expect(childOnSemanticsOwnerCreatedCount, 1);
      expect(rootOnSemanticsOwnerDisposed, 1);
      expect(childOnSemanticsOwnerDisposed, 0);
      expect(root.semanticsOwner, isNull);
      expect(child.semanticsOwner, isNotNull);

      childSemantics.dispose();

      expect(rootOnSemanticsOwnerCreatedCount, 1);
      expect(childOnSemanticsOwnerCreatedCount, 1);
      expect(rootOnSemanticsOwnerDisposed, 1);
      expect(childOnSemanticsOwnerDisposed, 1);
      expect(root.semanticsOwner, isNull);
      expect(child.semanticsOwner, isNull);
    },
  );

  test('PipelineOwner can dispose local handle even when manifold forces semantics to on', () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    int rootOnSemanticsOwnerCreatedCount = 0;
    int rootOnSemanticsOwnerDisposed = 0;
    final PipelineOwner root = PipelineOwner(
      onSemanticsOwnerCreated: () {
        rootOnSemanticsOwnerCreatedCount++;
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
      onSemanticsOwnerDisposed: () {
        rootOnSemanticsOwnerDisposed++;
      },
    );

    int childOnSemanticsOwnerCreatedCount = 0;
    int childOnSemanticsOwnerDisposed = 0;
    final PipelineOwner child = PipelineOwner(
      onSemanticsOwnerCreated: () {
        childOnSemanticsOwnerCreatedCount++;
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
      onSemanticsOwnerDisposed: () {
        childOnSemanticsOwnerDisposed++;
      },
    );

    root.adoptChild(child);
    root.attach(manifold);

    final SemanticsHandle childSemantics = child.ensureSemantics();
    expect(rootOnSemanticsOwnerCreatedCount, 0);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNotNull);

    manifold.semanticsEnabled = true;

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);

    childSemantics.dispose();

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);

    manifold.semanticsEnabled = false;

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 1);
    expect(childOnSemanticsOwnerDisposed, 1);
    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNull);
  });

  test('can hold on to local handle when manifold turns off semantics', () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    int rootOnSemanticsOwnerCreatedCount = 0;
    int rootOnSemanticsOwnerDisposed = 0;
    final PipelineOwner root = PipelineOwner(
      onSemanticsOwnerCreated: () {
        rootOnSemanticsOwnerCreatedCount++;
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
      onSemanticsOwnerDisposed: () {
        rootOnSemanticsOwnerDisposed++;
      },
    );

    int childOnSemanticsOwnerCreatedCount = 0;
    int childOnSemanticsOwnerDisposed = 0;
    final PipelineOwner child = PipelineOwner(
      onSemanticsOwnerCreated: () {
        childOnSemanticsOwnerCreatedCount++;
      },
      onSemanticsUpdate: (SemanticsUpdate update) {},
      onSemanticsOwnerDisposed: () {
        childOnSemanticsOwnerDisposed++;
      },
    );

    root.adoptChild(child);
    root.attach(manifold);

    expect(rootOnSemanticsOwnerCreatedCount, 0);
    expect(childOnSemanticsOwnerCreatedCount, 0);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNull);

    manifold.semanticsEnabled = true;

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);

    final SemanticsHandle childSemantics = child.ensureSemantics();

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 0);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);

    manifold.semanticsEnabled = false;

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 1);
    expect(childOnSemanticsOwnerDisposed, 0);
    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNotNull);

    childSemantics.dispose();

    expect(rootOnSemanticsOwnerCreatedCount, 1);
    expect(childOnSemanticsOwnerCreatedCount, 1);
    expect(rootOnSemanticsOwnerDisposed, 1);
    expect(childOnSemanticsOwnerDisposed, 1);
    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNull);
  });

  test('cannot attach when already attached', () {
    final TestPipelineManifold manifold = TestPipelineManifold();
    final PipelineOwner owner = PipelineOwner();

    owner.attach(manifold);
    expect(() => owner.attach(manifold), throwsAssertionError);
  });

  test('attach update semanticsOwner', () {
    final TestPipelineManifold manifold = TestPipelineManifold()..semanticsEnabled = true;
    final PipelineOwner owner = PipelineOwner(onSemanticsUpdate: (_) {});

    expect(owner.semanticsOwner, isNull);
    owner.attach(manifold);
    expect(owner.semanticsOwner, isNotNull);
  });

  test('attach does not request visual update if nothing is dirty', () {
    final TestPipelineManifold manifold = TestPipelineManifold();
    final TestRenderObject renderObject = TestRenderObject();
    final PipelineOwner owner = PipelineOwner();
    owner.rootNode = renderObject;

    expect(manifold.requestVisualUpdateCount, 0);
    owner.attach(manifold);
    expect(manifold.requestVisualUpdateCount, 0);
  });

  test('cannot detach when not attached', () {
    final PipelineOwner owner = PipelineOwner();

    expect(() => owner.detach(), throwsAssertionError);
  });

  test('cannot adopt twice', () {
    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child = PipelineOwner();
    root.adoptChild(child);
    expect(() => root.adoptChild(child), throwsAssertionError);
  });

  test('cannot adopt child of other parent', () {
    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child = PipelineOwner();
    final PipelineOwner otherRoot = PipelineOwner();
    root.adoptChild(child);
    expect(() => otherRoot.adoptChild(child), throwsAssertionError);
  });

  test('adopting creates semantics owner if necessary', () {
    final TestPipelineManifold manifold = TestPipelineManifold();
    final PipelineOwner root = PipelineOwner(onSemanticsUpdate: (_) {});
    final PipelineOwner child = PipelineOwner(onSemanticsUpdate: (_) {});
    final PipelineOwner childOfChild = PipelineOwner(onSemanticsUpdate: (_) {});
    root.attach(manifold);

    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNull);
    expect(childOfChild.semanticsOwner, isNull);

    root.adoptChild(child);

    expect(root.semanticsOwner, isNull);
    expect(child.semanticsOwner, isNull);
    expect(childOfChild.semanticsOwner, isNull);

    manifold.semanticsEnabled = true;

    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNull);

    child.adoptChild(childOfChild);

    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNotNull);
  });

  test('cannot drop unattached child', () {
    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child = PipelineOwner();
    expect(() => root.dropChild(child), throwsAssertionError);
  });

  test('cannot drop child attached to other parent', () {
    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child = PipelineOwner();
    final PipelineOwner otherRoot = PipelineOwner();
    otherRoot.adoptChild(child);
    expect(() => root.dropChild(child), throwsAssertionError);
  });

  test('dropping destroys semantics owner if necessary', () {
    final TestPipelineManifold manifold = TestPipelineManifold()..semanticsEnabled = true;
    final PipelineOwner root = PipelineOwner(onSemanticsUpdate: (_) {});
    final PipelineOwner child = PipelineOwner(onSemanticsUpdate: (_) {});
    final PipelineOwner childOfChild = PipelineOwner(onSemanticsUpdate: (_) {});
    root.attach(manifold);
    root.adoptChild(child);
    child.adoptChild(childOfChild);

    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNotNull);

    child.dropChild(childOfChild);

    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNotNull); // Retained in case we get re-attached.

    final SemanticsHandle childSemantics = child.ensureSemantics();
    root.dropChild(child);

    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNotNull); // Retained in case we get re-attached.

    childSemantics.dispose();

    expect(root.semanticsOwner, isNotNull);
    expect(child.semanticsOwner, isNull);
    expect(childOfChild.semanticsOwner, isNotNull);

    manifold.semanticsEnabled = false;

    expect(root.semanticsOwner, isNull);
    expect(childOfChild.semanticsOwner, isNotNull);

    root.adoptChild(childOfChild);
    expect(root.semanticsOwner, isNull);
    expect(childOfChild.semanticsOwner, isNull); // Disposed on re-attachment.

    manifold.semanticsEnabled = true;
    expect(root.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNotNull);

    root.dropChild(childOfChild);

    expect(root.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNotNull);

    childOfChild.dispose();

    expect(root.semanticsOwner, isNotNull);
    expect(childOfChild.semanticsOwner, isNull); // Disposed on dispose.
  });

  test('can adopt/drop children during own layout', () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child1 = PipelineOwner();
    final PipelineOwner child2 = PipelineOwner();

    final TestRenderObject rootRenderObject = TestRenderObject(
      onLayout: () {
        child1.dropChild(child2);
        root.dropChild(child1);
        root.adoptChild(child2);
        child2.adoptChild(child1);
      },
    );

    root.rootNode = rootRenderObject;
    rootRenderObject.scheduleInitialLayout();

    root.adoptChild(child1);
    child1.adoptChild(child2);
    root.attach(manifold);
    expect(_treeWalk(root), <PipelineOwner>[root, child1, child2]);

    root.flushLayout();

    expect(_treeWalk(root), <PipelineOwner>[root, child2, child1]);
  });

  test('cannot adopt/drop children during child layout', () {
    final TestPipelineManifold manifold = TestPipelineManifold();

    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child1 = PipelineOwner();
    final PipelineOwner child2 = PipelineOwner();
    final PipelineOwner child3 = PipelineOwner();

    Object? droppingError;
    Object? adoptingError;

    final TestRenderObject childRenderObject = TestRenderObject(
      onLayout: () {
        child1.dropChild(child2);
        child1.adoptChild(child3);
        try {
          root.dropChild(child1);
        } catch (e) {
          droppingError = e;
        }
        try {
          root.adoptChild(child2);
        } catch (e) {
          adoptingError = e;
        }
      },
    );

    child1.rootNode = childRenderObject;
    childRenderObject.scheduleInitialLayout();

    root.adoptChild(child1);
    child1.adoptChild(child2);
    root.attach(manifold);
    expect(_treeWalk(root), <PipelineOwner>[root, child1, child2]);

    root.flushLayout();

    expect(
      adoptingError,
      isAssertionError.having(
        (AssertionError e) => e.message,
        'message',
        contains('Cannot modify child list after layout.'),
      ),
    );
    expect(
      droppingError,
      isAssertionError.having(
        (AssertionError e) => e.message,
        'message',
        contains('Cannot modify child list after layout.'),
      ),
    );
  });

  test('visitChildren visits all children', () {
    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child1 = PipelineOwner();
    final PipelineOwner child2 = PipelineOwner();
    final PipelineOwner child3 = PipelineOwner();
    final PipelineOwner childOfChild3 = PipelineOwner();

    root.adoptChild(child1);
    root.adoptChild(child2);
    root.adoptChild(child3);
    child3.adoptChild(childOfChild3);

    final List<PipelineOwner> children = <PipelineOwner>[];
    root.visitChildren((PipelineOwner child) {
      children.add(child);
    });
    expect(children, <PipelineOwner>[child1, child2, child3]);

    children.clear();
    child3.visitChildren((PipelineOwner child) {
      children.add(child);
    });
    expect(children.single, childOfChild3);
  });

  test('printing pipeline owner tree smoke test', () {
    final PipelineOwner root = PipelineOwner();
    final PipelineOwner child1 = PipelineOwner()..rootNode = FakeRenderView();
    final PipelineOwner childOfChild1 = PipelineOwner()..rootNode = FakeRenderView();
    final PipelineOwner child2 = PipelineOwner()..rootNode = FakeRenderView();
    final PipelineOwner childOfChild2 = PipelineOwner()..rootNode = FakeRenderView();

    root.adoptChild(child1);
    child1.adoptChild(childOfChild1);
    root.adoptChild(child2);
    child2.adoptChild(childOfChild2);

    expect(
      root.toStringDeep(),
      equalsIgnoringHashCodes(
        'PipelineOwner#00000\n'
        ' ├─PipelineOwner#00000\n'
        ' │ │ rootNode: FakeRenderView#00000 NEEDS-LAYOUT NEEDS-PAINT\n'
        ' │ │\n'
        ' │ └─PipelineOwner#00000\n'
        ' │     rootNode: FakeRenderView#00000 NEEDS-LAYOUT NEEDS-PAINT\n'
        ' │\n'
        ' └─PipelineOwner#00000\n'
        '   │ rootNode: FakeRenderView#00000 NEEDS-LAYOUT NEEDS-PAINT\n'
        '   │\n'
        '   └─PipelineOwner#00000\n'
        '       rootNode: FakeRenderView#00000 NEEDS-LAYOUT NEEDS-PAINT\n',
      ),
    );
  });
}

class TestPipelineManifold extends ChangeNotifier implements PipelineManifold {
  int requestVisualUpdateCount = 0;

  @override
  void requestVisualUpdate() {
    requestVisualUpdateCount++;
  }

  @override
  bool get semanticsEnabled => _semanticsEnabled;
  bool _semanticsEnabled = false;
  set semanticsEnabled(bool value) {
    if (value == _semanticsEnabled) {
      return;
    }
    _semanticsEnabled = value;
    notifyListeners();
  }
}

class TestRenderObject extends RenderObject {
  TestRenderObject({this.onLayout, this.onPaint, this.onSemantics});

  final VoidCallback? onLayout;
  final VoidCallback? onPaint;
  final VoidCallback? onSemantics;

  @override
  bool get isRepaintBoundary => true;

  @override
  void debugAssertDoesMeetConstraints() {}

  @override
  Rect get paintBounds => Rect.zero;

  @override
  void performLayout() {
    onLayout?.call();
  }

  @override
  void paint(PaintingContext context, Offset offset) {
    onPaint?.call();
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    onSemantics?.call();
  }

  @override
  void performResize() {}

  @override
  Rect get semanticBounds => Rect.zero;
}

List<PipelineOwner> _treeWalk(PipelineOwner root) {
  final List<PipelineOwner> results = <PipelineOwner>[root];

  void visitor(PipelineOwner child) {
    results.add(child);
    child.visitChildren(visitor);
  }

  root.visitChildren(visitor);
  return results;
}

class FakeRenderView extends RenderBox {}
