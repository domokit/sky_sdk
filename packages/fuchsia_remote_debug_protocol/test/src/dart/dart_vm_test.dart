// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:fuchsia_remote_debug_protocol/src/dart/dart_vm.dart';
import 'package:test/fake.dart';
import 'package:test/test.dart';
import 'package:vm_service/vm_service.dart' as vms;

void main() {
  group('DartVm.connect', () {
    tearDown(() {
      restoreVmServiceConnectionFunction();
    });

    test('disconnect closes peer', () async {
      final FakeVmService service = FakeVmService();
      Future<vms.VmService> fakeServiceFunction(Uri uri, {Duration? timeout}) {
        return Future<vms.VmService>(() => service);
      }

      fuchsiaVmServiceConnectionFunction = fakeServiceFunction;
      final DartVm vm = await DartVm.connect(Uri.parse('http://this.whatever/ws'));
      expect(vm, isNot(null));
      await vm.stop();
      expect(service.disposed, true);
    });
  });

  group('DartVm.getAllFlutterViews', () {
    late FakeVmService fakeService;

    setUp(() {
      fakeService = FakeVmService();
    });

    tearDown(() {
      restoreVmServiceConnectionFunction();
    });

    test('basic flutter view parsing', () async {
      final Map<String, dynamic> flutterViewCannedResponses = <String, dynamic>{
        'views': <Map<String, dynamic>>[
          <String, dynamic>{'type': 'FlutterView', 'id': 'flutterView0'},
          <String, dynamic>{
            'type': 'FlutterView',
            'id': 'flutterView1',
            'isolate': <String, dynamic>{
              'type': '@Isolate',
              'fixedId': 'true',
              'id': 'isolates/1',
              'name': 'file://flutterBinary1',
              'number': '1',
            },
          },
          <String, dynamic>{
            'type': 'FlutterView',
            'id': 'flutterView2',
            'isolate': <String, dynamic>{
              'type': '@Isolate',
              'fixedId': 'true',
              'id': 'isolates/2',
              'name': 'file://flutterBinary2',
              'number': '2',
            },
          },
        ],
      };

      Future<vms.VmService> fakeVmConnectionFunction(Uri uri, {Duration? timeout}) {
        fakeService.flutterListViews = vms.Response.parse(flutterViewCannedResponses);
        return Future<vms.VmService>(() => fakeService);
      }

      fuchsiaVmServiceConnectionFunction = fakeVmConnectionFunction;
      final DartVm vm = await DartVm.connect(Uri.parse('http://whatever.com/ws'));
      expect(vm, isNot(null));
      final List<FlutterView> views = await vm.getAllFlutterViews();
      expect(views.length, 3);
      // Check ID's as they cannot be null.
      expect(views[0].id, 'flutterView0');
      expect(views[1].id, 'flutterView1');
      expect(views[2].id, 'flutterView2');

      // Verify names.
      expect(views[0].name, equals(null));
      expect(views[1].name, 'file://flutterBinary1');
      expect(views[2].name, 'file://flutterBinary2');
    });

    test('basic flutter view parsing with casting checks', () async {
      final Map<String, dynamic> flutterViewCannedResponses = <String, dynamic>{
        'views': <dynamic>[
          <String, dynamic>{'type': 'FlutterView', 'id': 'flutterView0'},
          <String, dynamic>{
            'type': 'FlutterView',
            'id': 'flutterView1',
            'isolate': <String, dynamic>{
              'type': '@Isolate',
              'fixedId': 'true',
              'id': 'isolates/1',
              'name': 'file://flutterBinary1',
              'number': '1',
            },
          },
          <String, dynamic>{
            'type': 'FlutterView',
            'id': 'flutterView2',
            'isolate': <String, dynamic>{
              'type': '@Isolate',
              'fixedId': 'true',
              'id': 'isolates/2',
              'name': 'file://flutterBinary2',
              'number': '2',
            },
          },
        ],
      };

      Future<vms.VmService> fakeVmConnectionFunction(Uri uri, {Duration? timeout}) {
        fakeService.flutterListViews = vms.Response.parse(flutterViewCannedResponses);
        return Future<vms.VmService>(() => fakeService);
      }

      fuchsiaVmServiceConnectionFunction = fakeVmConnectionFunction;
      final DartVm vm = await DartVm.connect(Uri.parse('http://whatever.com/ws'));
      expect(vm, isNot(null));
      final List<FlutterView> views = await vm.getAllFlutterViews();
      expect(views.length, 3);
      // Check ID's as they cannot be null.
      expect(views[0].id, 'flutterView0');
      expect(views[1].id, 'flutterView1');
      expect(views[2].id, 'flutterView2');

      // Verify names.
      expect(views[0].name, equals(null));
      expect(views[1].name, 'file://flutterBinary1');
      expect(views[2].name, 'file://flutterBinary2');
    });

    test('invalid flutter view missing ID', () async {
      final Map<String, dynamic> flutterViewCannedResponseMissingId = <String, dynamic>{
        'views': <Map<String, dynamic>>[
          // Valid flutter view.
          <String, dynamic>{
            'type': 'FlutterView',
            'id': 'flutterView1',
            'isolate': <String, dynamic>{
              'type': '@Isolate',
              'name': 'IsolateThing',
              'fixedId': 'true',
              'id': 'isolates/1',
              'number': '1',
            },
          },

          // Missing ID.
          <String, dynamic>{'type': 'FlutterView'},
        ],
      };

      Future<vms.VmService> fakeVmConnectionFunction(Uri uri, {Duration? timeout}) {
        fakeService.flutterListViews = vms.Response.parse(flutterViewCannedResponseMissingId);
        return Future<vms.VmService>(() => fakeService);
      }

      fuchsiaVmServiceConnectionFunction = fakeVmConnectionFunction;
      final DartVm vm = await DartVm.connect(Uri.parse('http://whatever.com/ws'));
      expect(vm, isNot(null));
      Future<void> failingFunction() async {
        await vm.getAllFlutterViews();
      }

      // Both views should be invalid as they were missing required fields.
      expect(failingFunction, throwsA(isA<RpcFormatError>()));
    });

    test('get isolates by pattern', () async {
      final List<vms.IsolateRef> isolates = <vms.IsolateRef>[
        vms.IsolateRef.parse(<String, dynamic>{
          'type': '@Isolate',
          'fixedId': 'true',
          'id': 'isolates/1',
          'name': 'file://thingThatWillNotMatch:main()',
          'number': '1',
        })!,
        vms.IsolateRef.parse(<String, dynamic>{
          'type': '@Isolate',
          'fixedId': 'true',
          'id': 'isolates/2',
          'name': '0:dart_name_pattern()',
          'number': '2',
        })!,
        vms.IsolateRef.parse(<String, dynamic>{
          'type': '@Isolate',
          'fixedId': 'true',
          'id': 'isolates/3',
          'name': 'flutterBinary.cm',
          'number': '3',
        })!,
        vms.IsolateRef.parse(<String, dynamic>{
          'type': '@Isolate',
          'fixedId': 'true',
          'id': 'isolates/4',
          'name': '0:some_other_dart_name_pattern()',
          'number': '4',
        })!,
      ];

      Future<vms.VmService> fakeVmConnectionFunction(Uri uri, {Duration? timeout}) {
        fakeService.vm = FakeVM(isolates: isolates);
        return Future<vms.VmService>(() => fakeService);
      }

      fuchsiaVmServiceConnectionFunction = fakeVmConnectionFunction;
      final DartVm vm = await DartVm.connect(Uri.parse('http://whatever.com/ws'));
      expect(vm, isNot(null));
      final List<IsolateRef> matchingFlutterIsolates = await vm.getMainIsolatesByPattern(
        'flutterBinary.cm',
      );
      expect(matchingFlutterIsolates.length, 1);
      final List<IsolateRef> allIsolates = await vm.getMainIsolatesByPattern('');
      expect(allIsolates.length, 4);
    });

    test('invalid flutter view missing ID', () async {
      final Map<String, dynamic> flutterViewCannedResponseMissingIsolateName = <String, dynamic>{
        'views': <Map<String, dynamic>>[
          // Missing isolate name.
          <String, dynamic>{
            'type': 'FlutterView',
            'id': 'flutterView1',
            'isolate': <String, dynamic>{
              'type': '@Isolate',
              'fixedId': 'true',
              'id': 'isolates/1',
              'number': '1',
            },
          },
        ],
      };

      Future<vms.VmService> fakeVmConnectionFunction(Uri uri, {Duration? timeout}) {
        fakeService.flutterListViews = vms.Response.parse(
          flutterViewCannedResponseMissingIsolateName,
        );
        return Future<vms.VmService>(() => fakeService);
      }

      fuchsiaVmServiceConnectionFunction = fakeVmConnectionFunction;
      final DartVm vm = await DartVm.connect(Uri.parse('http://whatever.com/ws'));
      expect(vm, isNot(null));
      Future<void> failingFunction() async {
        await vm.getAllFlutterViews();
      }

      // Both views should be invalid as they were missing required fields.
      expect(failingFunction, throwsA(isA<RpcFormatError>()));
    });
  });
}

class FakeVmService extends Fake implements vms.VmService {
  bool disposed = false;
  vms.Response? flutterListViews;
  vms.VM? vm;

  @override
  Future<vms.VM> getVM() async => vm!;

  @override
  Future<void> dispose() async {
    disposed = true;
  }

  @override
  Future<vms.Response> callMethod(
    String method, {
    String? isolateId,
    Map<String, dynamic>? args,
  }) async {
    if (method == '_flutter.listViews') {
      return flutterListViews!;
    }
    throw UnimplementedError(method);
  }

  @override
  Future<void> onDone = Future<void>.value();
}

class FakeVM extends Fake implements vms.VM {
  FakeVM({this.isolates = const <vms.IsolateRef>[]});

  @override
  List<vms.IsolateRef>? isolates;
}
