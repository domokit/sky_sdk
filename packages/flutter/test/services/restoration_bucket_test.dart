// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'restoration.dart';

void main() {
  test('root bucket values', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket bucket = RestorationBucket.root(manager: manager, rawData: rawData);

    expect(bucket.restorationId, 'root');
    expect(bucket.debugOwner, manager);

    // Bucket contains expected values from rawData.
    expect(bucket.read<int>('value1'), 10);
    expect(bucket.read<String>('value2'), 'Hello');
    expect(bucket.read<String>('value3'), isNull); // Does not exist.
    expect(manager.updateScheduled, isFalse);

    // Can overwrite existing value.
    bucket.write<int>('value1', 22);
    expect(manager.updateScheduled, isTrue);
    expect(bucket.read<int>('value1'), 22);
    manager.doSerialization();
    expect((rawData[valuesMapKey] as Map<String, dynamic>)['value1'], 22);
    expect(manager.updateScheduled, isFalse);

    // Can add a new value.
    bucket.write<bool>('value3', true);
    expect(manager.updateScheduled, isTrue);
    expect(bucket.read<bool>('value3'), true);
    manager.doSerialization();
    expect((rawData[valuesMapKey] as Map<String, dynamic>)['value3'], true);
    expect(manager.updateScheduled, isFalse);

    // Can remove existing value.
    expect(bucket.remove<int>('value1'), 22);
    expect(manager.updateScheduled, isTrue);
    expect(bucket.read<int>('value1'), isNull); // Does not exist anymore.
    manager.doSerialization();
    expect((rawData[valuesMapKey] as Map<String, dynamic>).containsKey('value1'), isFalse);
    expect(manager.updateScheduled, isFalse);

    // Removing non-existing value is no-op.
    expect(bucket.remove<Object>('value4'), isNull);
    expect(manager.updateScheduled, isFalse);

    // Can store null.
    bucket.write<bool?>('value4', null);
    expect(manager.updateScheduled, isTrue);
    expect(bucket.read<int>('value4'), null);
    manager.doSerialization();
    expect((rawData[valuesMapKey] as Map<String, dynamic>).containsKey('value4'), isTrue);
    expect((rawData[valuesMapKey] as Map<String, dynamic>)['value4'], null);
    expect(manager.updateScheduled, isFalse);
  });

  test('child bucket values', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rootRawData = _createRawDataSet();
    final Object debugOwner = Object();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rootRawData);
    final RestorationBucket child = RestorationBucket.child(
      restorationId: 'child1',
      parent: root,
      debugOwner: debugOwner,
    );

    expect(child.restorationId, 'child1');
    expect(child.debugOwner, debugOwner);

    // Bucket contains expected values from rawData.
    expect(child.read<int>('foo'), 22);
    expect(child.read<String>('bar'), isNull); // Does not exist.
    expect(manager.updateScheduled, isFalse);

    // Can overwrite existing value.
    child.write<int>('foo', 44);
    expect(manager.updateScheduled, isTrue);
    expect(child.read<int>('foo'), 44);
    manager.doSerialization();
    expect(
      (((rootRawData[childrenMapKey] as Map<String, dynamic>)['child1']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<String, dynamic>)['foo'],
      44,
    );
    expect(manager.updateScheduled, isFalse);

    // Can add a new value.
    child.write<bool>('value3', true);
    expect(manager.updateScheduled, isTrue);
    expect(child.read<bool>('value3'), true);
    manager.doSerialization();
    expect(
      (((rootRawData[childrenMapKey] as Map<String, dynamic>)['child1']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<String, dynamic>)['value3'],
      true,
    );
    expect(manager.updateScheduled, isFalse);

    // Can remove existing value.
    expect(child.remove<int>('foo'), 44);
    expect(manager.updateScheduled, isTrue);
    expect(child.read<int>('foo'), isNull); // Does not exist anymore.
    manager.doSerialization();
    expect(
      ((rootRawData[childrenMapKey] as Map<String, dynamic>)['child1'] as Map<String, dynamic>)
          .containsKey('foo'),
      isFalse,
    );
    expect(manager.updateScheduled, isFalse);

    // Removing non-existing value is no-op.
    expect(child.remove<Object>('value4'), isNull);
    expect(manager.updateScheduled, isFalse);

    // Can store null.
    child.write<bool?>('value4', null);
    expect(manager.updateScheduled, isTrue);
    expect(child.read<int>('value4'), null);
    manager.doSerialization();
    expect(
      (((rootRawData[childrenMapKey] as Map<String, dynamic>)['child1']
                  as Map<String, dynamic>)[valuesMapKey]
              as Map<String, dynamic>)
          .containsKey('value4'),
      isTrue,
    );
    expect(
      (((rootRawData[childrenMapKey] as Map<String, dynamic>)['child1']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<String, dynamic>)['value4'],
      null,
    );
    expect(manager.updateScheduled, isFalse);
  });

  test('claim child with existing data', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket bucket = RestorationBucket.root(manager: manager, rawData: rawData);

    final Object debugOwner = Object();
    final RestorationBucket child = bucket.claimChild('child1', debugOwner: debugOwner);

    expect(manager.updateScheduled, isFalse);
    expect(child.restorationId, 'child1');
    expect(child.debugOwner, debugOwner);

    expect(child.read<int>('foo'), 22);
    child.write('bar', 44);
    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(
      (((rawData[childrenMapKey] as Map<String, dynamic>)['child1']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<String, dynamic>)['bar'],
      44,
    );
    expect(manager.updateScheduled, isFalse);
  });

  test('claim child with no existing data', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket bucket = RestorationBucket.root(manager: manager, rawData: rawData);

    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child2'), isFalse);

    final Object debugOwner = Object();
    final RestorationBucket child = bucket.claimChild('child2', debugOwner: debugOwner);

    expect(manager.updateScheduled, isTrue);
    expect(child.restorationId, 'child2');
    expect(child.debugOwner, debugOwner);

    child.write('foo', 55);
    expect(child.read<int>('foo'), 55);
    manager.doSerialization();

    expect(manager.updateScheduled, isFalse);
    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child2'), isTrue);
    expect(
      (((rawData[childrenMapKey] as Map<String, dynamic>)['child2']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<Object?, Object?>)['foo'],
      55,
    );
  });

  test('claim child that is already claimed throws if not given up', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket bucket = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = bucket.claimChild('child1', debugOwner: 'FirstClaim');

    expect(manager.updateScheduled, isFalse);
    expect(child1.restorationId, 'child1');
    expect(child1.read<int>('foo'), 22);

    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    final RestorationBucket child2 = bucket.claimChild('child1', debugOwner: 'SecondClaim');
    expect(child2.restorationId, 'child1');
    expect(child2.read<int>('foo'), isNull); // Value does not exist in this child.

    // child1 is not given up before running finalizers.
    expect(
      () => manager.doSerialization(),
      throwsA(
        isFlutterError.having(
          (FlutterError error) => error.message,
          'message',
          equals(
            'Multiple owners claimed child RestorationBuckets with the same IDs.\n'
            'The following IDs were claimed multiple times from the parent RestorationBucket(restorationId: root, owner: MockManager):\n'
            ' * "child1" was claimed by:\n'
            '   * SecondClaim\n'
            '   * FirstClaim (current owner)',
          ),
        ),
      ),
    );
  });

  test('claim child that is already claimed does not throw if given up', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket bucket = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = bucket.claimChild('child1', debugOwner: 'FirstClaim');

    expect(manager.updateScheduled, isFalse);
    expect(child1.restorationId, 'child1');
    expect(child1.read<int>('foo'), 22);

    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    final RestorationBucket child2 = bucket.claimChild('child1', debugOwner: 'SecondClaim');
    expect(child2.restorationId, 'child1');
    expect(child2.read<int>('foo'), isNull); // Value does not exist in this child.
    child2.write<int>('bar', 55);

    // give up child1.
    child1.dispose();
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);
    expect(
      (((rawData[childrenMapKey] as Map<String, dynamic>)['child1']
                  as Map<String, dynamic>)[valuesMapKey]
              as Map<Object?, Object?>)
          .containsKey('foo'),
      isFalse,
    );
    expect(
      (((rawData[childrenMapKey] as Map<String, dynamic>)['child1']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<Object?, Object?>)['bar'],
      55,
    );
  });

  test('claiming a claimed child twice and only giving it up once throws', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket bucket = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = bucket.claimChild('child1', debugOwner: 'FirstClaim');
    expect(child1.restorationId, 'child1');
    final RestorationBucket child2 = bucket.claimChild('child1', debugOwner: 'SecondClaim');
    expect(child2.restorationId, 'child1');
    child1.dispose();
    final RestorationBucket child3 = bucket.claimChild('child1', debugOwner: 'ThirdClaim');
    expect(child3.restorationId, 'child1');
    expect(manager.updateScheduled, isTrue);
    expect(() => manager.doSerialization(), throwsFlutterError);
  });

  test('unclaiming and then claiming same id gives fresh bucket', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket bucket = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = bucket.claimChild('child1', debugOwner: 'FirstClaim');
    expect(manager.updateScheduled, isFalse);
    expect(child1.read<int>('foo'), 22);
    child1.dispose();
    expect(manager.updateScheduled, isTrue);
    final RestorationBucket child2 = bucket.claimChild('child1', debugOwner: 'SecondClaim');
    expect(child2.read<int>('foo'), isNull);
  });

  test('cleans up raw data if last value/child is dropped', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    expect(rawData.containsKey(childrenMapKey), isTrue);
    final RestorationBucket child = root.claimChild('child1', debugOwner: 'owner');
    child.dispose();
    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(rawData.containsKey(childrenMapKey), isFalse);

    expect(rawData.containsKey(valuesMapKey), isTrue);
    expect(root.remove<int>('value1'), 10);
    expect(root.remove<String>('value2'), 'Hello');
    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(rawData.containsKey(valuesMapKey), isFalse);
  });

  test('dispose deletes data', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = root.claimChild('child1', debugOwner: 'owner1');
    child1.claimChild('child1OfChild1', debugOwner: 'owner1.1');
    child1.claimChild('child2OfChild1', debugOwner: 'owner1.2');
    final RestorationBucket child2 = root.claimChild('child2', debugOwner: 'owner2');

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child1'), isTrue);
    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child2'), isTrue);

    child1.dispose();
    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child1'), isFalse);

    child2.dispose();
    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect(rawData.containsKey(childrenMapKey), isFalse);
  });

  test('rename is no-op if same id', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child = root.claimChild('child1', debugOwner: 'owner1');

    expect(manager.updateScheduled, isFalse);
    expect(child.restorationId, 'child1');
    child.rename('child1');
    expect(manager.updateScheduled, isFalse);
    expect(child.restorationId, 'child1');
    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child1'), isTrue);
  });

  test('rename to unused id', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child = root.claimChild('child1', debugOwner: 'owner1');
    final Object rawChildData =
        (rawData[childrenMapKey] as Map<String, dynamic>)['child1'] as Object;
    expect(rawChildData, isNotNull);

    expect(manager.updateScheduled, isFalse);
    expect(child.restorationId, 'child1');
    child.rename('new-name');
    expect(child.restorationId, 'new-name');

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect((rawData[childrenMapKey] as Map<Object?, Object?>).containsKey('child1'), isFalse);
    expect((rawData[childrenMapKey] as Map<Object?, Object?>)['new-name'], rawChildData);
  });

  test('rename to used id throws if id is not given up', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = root.claimChild('child1', debugOwner: 'owner1');
    final RestorationBucket child2 = root.claimChild('child2', debugOwner: 'owner1');
    manager.doSerialization();

    expect(child1.restorationId, 'child1');
    expect(child2.restorationId, 'child2');
    child2.rename('child1');
    expect(child2.restorationId, 'child1');

    expect(manager.updateScheduled, isTrue);
    expect(() => manager.doSerialization(), throwsFlutterError);
  });

  test('rename to used id does not throw if id is given up', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = root.claimChild('child1', debugOwner: 'owner1');
    final RestorationBucket child2 = root.claimChild('child2', debugOwner: 'owner1');
    manager.doSerialization();

    final Object rawChild1Data =
        (rawData[childrenMapKey] as Map<String, dynamic>)['child1'] as Object;
    expect(rawChild1Data, isNotNull);
    final Object rawChild2Data =
        (rawData[childrenMapKey] as Map<String, dynamic>)['child2'] as Object;
    expect(rawChild2Data, isNotNull);

    expect(child1.restorationId, 'child1');
    expect(child2.restorationId, 'child2');
    child2.rename('child1');
    expect(child2.restorationId, 'child1');
    expect(child1.restorationId, 'child1');

    child1.dispose();

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect((rawData[childrenMapKey] as Map<String, dynamic>)['child1'], rawChild2Data);
    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child2'), isFalse);
  });

  test('renaming a to be added child', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final Object rawChild1Data =
        (rawData[childrenMapKey] as Map<String, dynamic>)['child1'] as Object;
    expect(rawChild1Data, isNotNull);

    final RestorationBucket child1 = root.claimChild('child1', debugOwner: 'owner1');
    final RestorationBucket child2 = root.claimChild('child1', debugOwner: 'owner1');

    child2.rename('foo');

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect(child1.restorationId, 'child1');
    expect(child2.restorationId, 'foo');

    expect((rawData[childrenMapKey] as Map<String, dynamic>)['child1'], rawChild1Data);
    expect((rawData[childrenMapKey] as Map<String, dynamic>)['foo'], isEmpty); // new bucket
  });

  test('adopt is no-op if same parent', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = root.claimChild('child1', debugOwner: 'owner1');

    root.adoptChild(child1);
    expect(manager.updateScheduled, isFalse);
    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('child1'), isTrue);
  });

  test('adopt fresh child', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child = RestorationBucket.empty(
      restorationId: 'fresh-child',
      debugOwner: 'owner1',
    );

    root.adoptChild(child);
    expect(manager.updateScheduled, isTrue);

    child.write('value', 22);

    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect((rawData[childrenMapKey] as Map<String, dynamic>).containsKey('fresh-child'), isTrue);
    expect(
      (((rawData[childrenMapKey] as Map<String, dynamic>)['fresh-child']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<Object?, Object?>)['value'],
      22,
    );

    child.write('bar', 'blabla');
    expect(manager.updateScheduled, isTrue);
  });

  test('adopt child that already had a parent', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child = root.claimChild('child1', debugOwner: 'owner1');
    final RestorationBucket childOfChild = child.claimChild('childOfChild', debugOwner: 'owner2');
    childOfChild.write<String>('foo', 'bar');

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    final Object childOfChildData =
        (((rawData[childrenMapKey] as Map<String, dynamic>)['child1']
                as Map<String, dynamic>)[childrenMapKey]
            as Map<Object?, Object?>)['childOfChild']!;
    expect(childOfChildData, isNotEmpty);

    root.adoptChild(childOfChild);
    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect(
      ((rawData[childrenMapKey] as Map<String, dynamic>)['child1'] as Map<String, dynamic>)
          .containsKey(childrenMapKey),
      isFalse,
    ); // child1 has no children anymore.
    expect((rawData[childrenMapKey] as Map<String, dynamic>)['childOfChild'], childOfChildData);
  });

  test('adopting child throws if id is already in use and not given up', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child = root.claimChild('child1', debugOwner: 'owner1');
    final RestorationBucket childOfChild = child.claimChild('child1', debugOwner: 'owner2');
    childOfChild.write<String>('foo', 'bar');

    root.adoptChild(childOfChild);
    expect(manager.updateScheduled, isTrue);
    expect(() => manager.doSerialization(), throwsFlutterError);
  });

  test('adopting child does not throw if id is already in use and given up', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child = root.claimChild('child1', debugOwner: 'owner1');
    final RestorationBucket childOfChild = child.claimChild('child1', debugOwner: 'owner2');
    childOfChild.write<String>('foo', 'bar');

    final Object childOfChildData =
        (((rawData[childrenMapKey] as Map<String, dynamic>)['child1']
                as Map<String, dynamic>)[childrenMapKey]
            as Map<Object?, Object?>)['child1']!;
    expect(childOfChildData, isNotEmpty);

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    root.adoptChild(childOfChild);
    expect(manager.updateScheduled, isTrue);
    child.dispose();
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect((rawData[childrenMapKey] as Map<String, dynamic>)['child1'], childOfChildData);
  });

  test('adopting a to-be-added child under an already in use id', () {
    final MockRestorationManager manager = MockRestorationManager();
    final Map<String, dynamic> rawData = _createRawDataSet();
    final RestorationBucket root = RestorationBucket.root(manager: manager, rawData: rawData);

    final RestorationBucket child1 = root.claimChild('child1', debugOwner: 'owner1');
    final RestorationBucket child2 = root.claimChild('child2', debugOwner: 'owner1');

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    final RestorationBucket child1OfChild1 = child1.claimChild('child2', debugOwner: 'owner2');
    child1OfChild1.write<String>('hello', 'world');
    final RestorationBucket child2OfChild1 = child1.claimChild('child2', debugOwner: 'owner2');
    child2OfChild1.write<String>('foo', 'bar');

    root.adoptChild(child2OfChild1);
    child2.dispose();

    expect(manager.updateScheduled, isTrue);
    manager.doSerialization();
    expect(manager.updateScheduled, isFalse);

    expect(
      (((rawData[childrenMapKey] as Map<String, dynamic>)['child2']
              as Map<String, dynamic>)[valuesMapKey]
          as Map<Object?, Object?>)['foo'],
      'bar',
    );
    expect(
      (((((rawData[childrenMapKey] as Map<String, dynamic>)['child1']
                      as Map<String, dynamic>)[childrenMapKey]
                  as Map<Object?, Object?>)['child2']!
              as Map<String, dynamic>)[valuesMapKey]
          as Map<Object?, Object?>)['hello'],
      'world',
    );
  });

  test('throws when used after dispose', () {
    final RestorationBucket bucket = RestorationBucket.empty(
      restorationId: 'foo',
      debugOwner: null,
    );
    bucket.dispose();

    expect(() => bucket.debugOwner, throwsFlutterError);
    expect(() => bucket.restorationId, throwsFlutterError);
    expect(() => bucket.read<int>('foo'), throwsFlutterError);
    expect(() => bucket.write('foo', 10), throwsFlutterError);
    expect(() => bucket.remove<int>('foo'), throwsFlutterError);
    expect(() => bucket.contains('foo'), throwsFlutterError);
    expect(() => bucket.claimChild('child', debugOwner: null), throwsFlutterError);
    final RestorationBucket child = RestorationBucket.empty(
      restorationId: 'child',
      debugOwner: null,
    );
    expect(() => bucket.adoptChild(child), throwsFlutterError);
    expect(() => bucket.rename('bar'), throwsFlutterError);
    expect(() => bucket.dispose(), throwsFlutterError);
  });

  test('$RestorationBucket dispatches memory events', () async {
    await expectLater(
      await memoryEvents(
        () => RestorationBucket.empty(restorationId: 'child1', debugOwner: null).dispose(),
        RestorationBucket,
      ),
      areCreateAndDispose,
    );

    final MockRestorationManager manager1 = MockRestorationManager();
    addTearDown(manager1.dispose);
    await expectLater(
      await memoryEvents(
        () => RestorationBucket.root(manager: manager1, rawData: null).dispose(),
        RestorationBucket,
      ),
      areCreateAndDispose,
    );

    final MockRestorationManager manager2 = MockRestorationManager();
    addTearDown(manager2.dispose);
    final RestorationBucket parent = RestorationBucket.root(
      manager: manager2,
      rawData: _createRawDataSet(),
    );
    addTearDown(parent.dispose);
    await expectLater(
      await memoryEvents(
        () =>
            RestorationBucket.child(
              restorationId: 'child1',
              parent: parent,
              debugOwner: null,
            ).dispose(),
        RestorationBucket,
      ),
      areCreateAndDispose,
    );
  });
}

Map<String, dynamic> _createRawDataSet() {
  return <String, dynamic>{
    valuesMapKey: <String, dynamic>{'value1': 10, 'value2': 'Hello'},
    childrenMapKey: <String, dynamic>{
      'child1': <String, dynamic>{
        valuesMapKey: <String, dynamic>{'foo': 22},
      },
    },
  };
}
