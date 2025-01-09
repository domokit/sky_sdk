// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class MockRestorationManager extends TestRestorationManager {
  MockRestorationManager({this.enableChannels = false});

  bool get updateScheduled => _updateScheduled;
  bool _updateScheduled = false;

  final List<RestorationBucket> _buckets = <RestorationBucket>[];

  final bool enableChannels;

  @override
  void initChannels() {
    if (enableChannels) {
      super.initChannels();
    }
  }

  @override
  void scheduleSerializationFor(RestorationBucket bucket) {
    _updateScheduled = true;
    _buckets.add(bucket);
  }

  @override
  bool unscheduleSerializationFor(RestorationBucket bucket) {
    _updateScheduled = true;
    return _buckets.remove(bucket);
  }

  void doSerialization() {
    _updateScheduled = false;
    for (final RestorationBucket bucket in _buckets) {
      bucket.finalize();
    }
    _buckets.clear();
  }

  @override
  void restoreFrom(TestRestorationData data) {
    // Ignore in mock.
  }

  int rootBucketAccessed = 0;

  @override
  Future<RestorationBucket?> get rootBucket {
    rootBucketAccessed++;
    return _rootBucket;
  }

  late Future<RestorationBucket?> _rootBucket;
  set rootBucket(Future<RestorationBucket?> value) {
    _rootBucket = value;
    _isRestoring = true;
    ServicesBinding.instance.addPostFrameCallback((Duration _) {
      _isRestoring = false;
    });
    notifyListeners();
  }

  @override
  bool get isReplacing => _isRestoring;
  bool _isRestoring = false;

  @override
  Future<void> sendToEngine(Uint8List encodedData) {
    throw UnimplementedError('unimplemented in mock');
  }

  @override
  String toString() => 'MockManager';
}

const String childrenMapKey = 'c';
const String valuesMapKey = 'v';
