// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'restoration.dart';

void main() {
  group('UnmanagedRestorationScope', () {
    testWidgets('makes bucket available to descendants', (WidgetTester tester) async {
      final RestorationBucket bucket1 = RestorationBucket.empty(
        restorationId: 'foo',
        debugOwner: 'owner',
      );
      addTearDown(bucket1.dispose);

      await tester.pumpWidget(UnmanagedRestorationScope(bucket: bucket1, child: const BucketSpy()));

      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket, bucket1);

      // Notifies when bucket changes.
      final RestorationBucket bucket2 = RestorationBucket.empty(
        restorationId: 'foo2',
        debugOwner: 'owner',
      );
      addTearDown(bucket2.dispose);

      await tester.pumpWidget(UnmanagedRestorationScope(bucket: bucket2, child: const BucketSpy()));
      expect(state.bucket, bucket2);
    });

    testWidgets('null bucket disables restoration', (WidgetTester tester) async {
      await tester.pumpWidget(const UnmanagedRestorationScope(child: BucketSpy()));
      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket, isNull);
    });
  });

  group('RestorationScope', () {
    testWidgets('asserts when none is found', (WidgetTester tester) async {
      late BuildContext capturedContext;
      await tester.pumpWidget(
        WidgetsApp(
          color: const Color(0xD0FF0000),
          builder: (_, __) {
            return RestorationScope(
              restorationId: 'test',
              child: Builder(
                builder: (BuildContext context) {
                  capturedContext = context;
                  return Container();
                },
              ),
            );
          },
        ),
      );
      expect(
        () {
          RestorationScope.of(capturedContext);
        },
        throwsA(
          isA<FlutterError>().having(
            (FlutterError error) => error.message,
            'message',
            contains('State restoration must be enabled for a RestorationScope'),
          ),
        ),
      );

      await tester.pumpWidget(
        WidgetsApp(
          restorationScopeId: 'test scope',
          color: const Color(0xD0FF0000),
          builder: (_, __) {
            return RestorationScope(
              restorationId: 'test',
              child: Builder(
                builder: (BuildContext context) {
                  capturedContext = context;
                  return Container();
                },
              ),
            );
          },
        ),
      );
      final UnmanagedRestorationScope scope = tester.widget(
        find.byType(UnmanagedRestorationScope).last,
      );
      expect(RestorationScope.of(capturedContext), scope.bucket);
    });

    testWidgets('makes bucket available to descendants', (WidgetTester tester) async {
      const String id = 'hello world 1234';
      final MockRestorationManager manager = MockRestorationManager();
      addTearDown(manager.dispose);
      final Map<String, dynamic> rawData = <String, dynamic>{};
      final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);
      addTearDown(root.dispose);
      expect(rawData, isEmpty);

      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: id, child: BucketSpy()),
        ),
      );
      manager.doSerialization();

      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket!.restorationId, id);
      expect((rawData[childrenMapKey] as Map<Object?, Object?>).containsKey(id), isTrue);
    });

    testWidgets('bucket for descendants contains data claimed from parent', (
      WidgetTester tester,
    ) async {
      final MockRestorationManager manager = MockRestorationManager();
      addTearDown(manager.dispose);
      final RestorationBucket root = RestorationBucket.root(
        manager: manager,
        rawData: _createRawDataSet(),
      );
      addTearDown(root.dispose);

      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: 'child1', child: BucketSpy()),
        ),
      );
      manager.doSerialization();

      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket!.restorationId, 'child1');
      expect(state.bucket!.read<int>('foo'), 22);
    });

    testWidgets('renames existing bucket when new ID is provided', (WidgetTester tester) async {
      final MockRestorationManager manager = MockRestorationManager();
      addTearDown(manager.dispose);
      final RestorationBucket root = RestorationBucket.root(
        manager: manager,
        rawData: _createRawDataSet(),
      );
      addTearDown(root.dispose);

      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: 'child1', child: BucketSpy()),
        ),
      );
      manager.doSerialization();

      // Claimed existing bucket with data.
      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket!.restorationId, 'child1');
      expect(state.bucket!.read<int>('foo'), 22);
      final RestorationBucket bucket = state.bucket!;

      // Rename the existing bucket.
      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: 'something else', child: BucketSpy()),
        ),
      );
      manager.doSerialization();

      expect(state.bucket!.restorationId, 'something else');
      expect(state.bucket!.read<int>('foo'), 22);
      expect(state.bucket, same(bucket));
    });

    testWidgets('Disposing a scope removes its data', (WidgetTester tester) async {
      final MockRestorationManager manager = MockRestorationManager();
      addTearDown(manager.dispose);
      final Map<String, dynamic> rawData = _createRawDataSet();
      final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);
      addTearDown(root.dispose);

      expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child1'), isTrue);
      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: 'child1', child: BucketSpy()),
        ),
      );
      manager.doSerialization();
      expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child1'), isTrue);

      await tester.pumpWidget(UnmanagedRestorationScope(bucket: root, child: Container()));
      manager.doSerialization();

      expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child1'), isFalse);
    });

    testWidgets('no bucket for descendants when id is null', (WidgetTester tester) async {
      final MockRestorationManager manager = MockRestorationManager();
      addTearDown(manager.dispose);
      final RestorationBucket root = RestorationBucket.root(
        manager: manager,
        rawData: <String, dynamic>{},
      );
      addTearDown(root.dispose);

      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: null, child: BucketSpy()),
        ),
      );
      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket, isNull);

      // Change id to non-null.
      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: 'foo', child: BucketSpy()),
        ),
      );
      manager.doSerialization();
      expect(state.bucket, isNotNull);
      expect(state.bucket!.restorationId, 'foo');

      // Change id back to null.
      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: const RestorationScope(restorationId: null, child: BucketSpy()),
        ),
      );
      manager.doSerialization();
      expect(state.bucket, isNull);
    });

    testWidgets('no bucket for descendants when scope is null', (WidgetTester tester) async {
      final Key scopeKey = GlobalKey();

      await tester.pumpWidget(
        RestorationScope(key: scopeKey, restorationId: 'foo', child: const BucketSpy()),
      );
      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket, isNull);

      // Move it under a valid scope.
      final MockRestorationManager manager = MockRestorationManager();
      addTearDown(manager.dispose);
      final RestorationBucket root = RestorationBucket.root(
        manager: manager,
        rawData: <String, dynamic>{},
      );
      addTearDown(root.dispose);

      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: RestorationScope(key: scopeKey, restorationId: 'foo', child: const BucketSpy()),
        ),
      );
      manager.doSerialization();
      expect(state.bucket, isNotNull);
      expect(state.bucket!.restorationId, 'foo');

      // Move out of scope again.
      await tester.pumpWidget(
        RestorationScope(key: scopeKey, restorationId: 'foo', child: const BucketSpy()),
      );
      manager.doSerialization();
      expect(state.bucket, isNull);
    });

    testWidgets('no bucket for descendants when scope and id are null', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(const RestorationScope(restorationId: null, child: BucketSpy()));
      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket, isNull);
    });

    testWidgets('moving scope moves its data', (WidgetTester tester) async {
      final MockRestorationManager manager = MockRestorationManager();
      addTearDown(manager.dispose);
      final Map<String, dynamic> rawData = <String, dynamic>{};
      final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);
      addTearDown(root.dispose);
      final Key scopeKey = GlobalKey();

      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: Row(
            textDirection: TextDirection.ltr,
            children: <Widget>[
              RestorationScope(
                restorationId: 'fixed',
                child: RestorationScope(
                  key: scopeKey,
                  restorationId: 'moving-child',
                  child: const BucketSpy(),
                ),
              ),
            ],
          ),
        ),
      );
      manager.doSerialization();
      final BucketSpyState state = tester.state(find.byType(BucketSpy));
      expect(state.bucket!.restorationId, 'moving-child');
      expect(
        (((rawData[childrenMapKey] as Map<Object?, Object?>)['fixed']!
                    as Map<String, dynamic>)[childrenMapKey]
                as Map<Object?, Object?>)
            .containsKey('moving-child'),
        isTrue,
      );
      final RestorationBucket bucket = state.bucket!;

      state.bucket!.write('value', 11);
      manager.doSerialization();

      // Move scope.
      await tester.pumpWidget(
        UnmanagedRestorationScope(
          bucket: root,
          child: Row(
            textDirection: TextDirection.ltr,
            children: <Widget>[
              RestorationScope(restorationId: 'fixed', child: Container()),
              RestorationScope(
                key: scopeKey,
                restorationId: 'moving-child',
                child: const BucketSpy(),
              ),
            ],
          ),
        ),
      );
      manager.doSerialization();
      expect(state.bucket!.restorationId, 'moving-child');
      expect(state.bucket, same(bucket));
      expect(state.bucket!.read<int>('value'), 11);

      expect((rawData[childrenMapKey] as Map<Object?, Object?>)['fixed'], isEmpty);
      expect(
        (rawData[childrenMapKey] as Map<Object?, Object?>).containsKey('moving-child'),
        isTrue,
      );
    });
  });
}

Map<String, dynamic> _createRawDataSet() {
  return <String, dynamic>{
    valuesMapKey: <String, dynamic>{'value1': 10, 'value2': 'Hello'},
    childrenMapKey: <String, dynamic>{
      'child1': <String, dynamic>{
        valuesMapKey: <String, dynamic>{'foo': 22},
      },
      'child2': <String, dynamic>{
        valuesMapKey: <String, dynamic>{'bar': 33},
      },
    },
  };
}
